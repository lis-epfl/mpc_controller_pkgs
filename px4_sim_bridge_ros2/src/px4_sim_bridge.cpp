/**
 * @file px4_sim_bridge.cpp
 * @brief Implementation of the PX4 simulation bridge
 */

#include "px4_sim_bridge_ros2/px4_sim_bridge.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace px4_sim_bridge_ros2 {

Px4SimBridge::Px4SimBridge(const rclcpp::NodeOptions &options)
    : Node("px4_sim_bridge", options) {

  // Declare parameters
  this->declare_parameter<bool>("auto_arm", true);
  this->declare_parameter<bool>("auto_offboard", true);
  this->declare_parameter<double>("takeoff_altitude", 1.0);
  this->declare_parameter<double>("state_machine_rate", 20.0);

  // Get parameters
  this->get_parameter("auto_arm", auto_arm_);
  this->get_parameter("auto_offboard", auto_offboard_);
  this->get_parameter("takeoff_altitude", takeoff_altitude_);
  this->get_parameter("state_machine_rate", state_machine_rate_);

  // Create QoS profiles
  rclcpp::QoS qos(10);
  rclcpp::QoS sensor_qos(1);
  sensor_qos.best_effort();

  // Subscribers
  vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", sensor_qos,
      std::bind(&Px4SimBridge::vehicleStatusCallback, this,
                std::placeholders::_1));

  // Monitor rate commands
  rate_cmd_monitor_sub_ =
      this->create_subscription<px4_msgs::msg::VehicleRatesSetpoint>(
          "/fmu/in/vehicle_rates_setpoint", qos,
          std::bind(&Px4SimBridge::rateCommandMonitorCallback, this,
                    std::placeholders::_1));

  // Monitor torque commands
  torque_cmd_monitor_sub_ =
      this->create_subscription<px4_msgs::msg::VehicleTorqueSetpoint>(
          "/fmu/in/vehicle_torque_setpoint", qos,
          std::bind(&Px4SimBridge::torqueCommandMonitorCallback, this,
                    std::placeholders::_1));

  // Monitor motor command
  motor_cmd_monitor_sub_ =
      this->create_subscription<px4_msgs::msg::ActuatorMotors>(
          "/fmu/in/actuator_motors", qos,
          std::bind(&Px4SimBridge::motorCommandMonitorCallback, this,
                    std::placeholders::_1));

  // Publishers
  offboard_mode_pub_ =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>(
          "/fmu/in/offboard_control_mode", qos);

  vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", qos);

  trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", qos);

  // Services
  arm_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/arm", std::bind(&Px4SimBridge::armCallback, this,
                         std::placeholders::_1, std::placeholders::_2));

  disarm_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/disarm", std::bind(&Px4SimBridge::disarmCallback, this,
                            std::placeholders::_1, std::placeholders::_2));

  // State machine timer
  int period_ms = static_cast<int>(1000.0 / state_machine_rate_);
  state_machine_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&Px4SimBridge::stateMachineCallback, this));

  last_rate_command_time_ = this->get_clock()->now();
  last_torque_command_time_ = this->get_clock()->now();
  last_motor_command_time_ = this->get_clock()->now();

  RCLCPP_INFO(this->get_logger(), "PX4 Simulation Bridge initialized");
  RCLCPP_INFO(this->get_logger(), "Auto-arm: %s",
              auto_arm_ ? "enabled" : "disabled");
  RCLCPP_INFO(this->get_logger(), "Auto-offboard: %s",
              auto_offboard_ ? "enabled" : "disabled");
  RCLCPP_INFO(this->get_logger(), "Takeoff altitude: %.2f m",
              takeoff_altitude_);
}

void Px4SimBridge::vehicleStatusCallback(
    const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
  armed_ = (msg->arming_state == 2);       // ARMING_STATE_ARMED
  offboard_mode_ = (msg->nav_state == 14); // NAVIGATION_STATE_OFFBOARD
}

void Px4SimBridge::rateCommandMonitorCallback(
    const px4_msgs::msg::VehicleRatesSetpoint::SharedPtr msg) {
  (void)msg; // Unused - we only care about timing
  last_rate_command_time_ = this->get_clock()->now();
  // Note: We don't republish - MPC controller publishes directly to PX4
}

void Px4SimBridge::torqueCommandMonitorCallback(
    const px4_msgs::msg::VehicleTorqueSetpoint::SharedPtr msg) {
  (void)msg; // Unused - we only care about timing
  last_torque_command_time_ = this->get_clock()->now();
}

void Px4SimBridge::motorCommandMonitorCallback(
    const px4_msgs::msg::ActuatorMotors::SharedPtr msg) {
  (void)msg; // Unused - we only care about timing
  last_motor_command_time_ = this->get_clock()->now();
}

void Px4SimBridge::stateMachineCallback() {
  auto now = this->get_clock()->now();
  uint64_t timestamp = now.nanoseconds() / 1000;

  // Check if we're receiving rate commands
  bool receiving_rate_commands =
      (now - last_rate_command_time_).seconds() < 0.5;
  bool receiving_torque_commands =
      (now - last_torque_command_time_).seconds() < 0.5;
  bool receiving_motor_commands =
      (now - last_motor_command_time_).seconds() < 0.5;
  bool receiving_commands = receiving_rate_commands ||
                            receiving_motor_commands ||
                            receiving_torque_commands;

  switch (state_) {
  case SimState::INIT:
    if (auto_arm_ && receiving_commands) {
      RCLCPP_INFO(this->get_logger(), "Initiating arming sequence");
      publishVehicleCommand(400, 1.0, 21196.0); // Force arm
      state_ = SimState::ARMING;
      wait_counter_ = 0;
    }
    break;

  case SimState::ARMING:
    wait_counter_++;

    // Retry arming every second
    if (!armed_ && wait_counter_ % 20 == 0) {
      RCLCPP_INFO(this->get_logger(), "Retrying arm command...");
      publishVehicleCommand(400, 1.0, 21196.0);
    }

    if (armed_) {
      RCLCPP_INFO(this->get_logger(), "Vehicle armed successfully");
      state_ = SimState::SWITCH_OFFBOARD;
      wait_counter_ = 0;
    } else if (wait_counter_ > 100) { // 5 second timeout
      RCLCPP_WARN(this->get_logger(), "Arming timeout, returning to INIT");
      state_ = SimState::INIT;
      wait_counter_ = 0;
    }
    break;

  case SimState::SWITCH_OFFBOARD:
    // Publish position setpoint to satisfy offboard requirements
    publishTrajectorySetpoint(0.0, 0.0, -takeoff_altitude_, 0.0);
    publishOffboardControlMode(ControlMode::POSITION);

    wait_counter_++;

    // Attempt to switch to offboard mode
    if (wait_counter_ == 20) {
      RCLCPP_INFO(this->get_logger(), "Switching to offboard mode");
      publishVehicleCommand(176, 1.0, 6.0); // DO_SET_MODE to offboard
    }

    // Check if successful
    if (wait_counter_ > 25 && offboard_mode_) {
      RCLCPP_INFO(this->get_logger(), "Offboard mode active, starting takeoff");
      state_ = SimState::TAKEOFF;
      wait_counter_ = 0;
    }

    // Retry periodically
    if (wait_counter_ > 40 && !offboard_mode_ && (wait_counter_ % 20 == 0)) {
      RCLCPP_WARN(this->get_logger(), "Retrying offboard mode switch...");
      publishVehicleCommand(176, 1.0, 6.0);
    }

    if (wait_counter_ > 200) { // 10 second timeout
      RCLCPP_ERROR(this->get_logger(), "Failed to switch to offboard mode");
      state_ = SimState::INIT;
      wait_counter_ = 0;
    }
    break;

  case SimState::TAKEOFF:
    // Continue publishing position setpoint during takeoff
    publishTrajectorySetpoint(0.0, 0.0, -takeoff_altitude_, 0.0);
    publishOffboardControlMode(ControlMode::POSITION);

    wait_counter_++;

    // Wait for vehicle to reach altitude (10 seconds)
    if (wait_counter_ > 200) {
      RCLCPP_INFO(this->get_logger(),
                  "Takeoff complete, ready for rate control commands");
      state_ = SimState::READY;
      wait_counter_ = 0;
    }
    break;

  case SimState::READY:
    if (receiving_commands) {
      // Determine which control mode based on what commands we're receiving
      if (receiving_torque_commands) {
        // Torque + thrust commands
        publishOffboardControlMode(ControlMode::THRUST_TORQUE);
      } else if (receiving_motor_commands) {
        // Direct motor commands (INDI)
        publishOffboardControlMode(ControlMode::DIRECT_ACTUATOR);
      } else if (receiving_rate_commands) {
        // Rate commands
        publishOffboardControlMode(ControlMode::BODY_RATE);
      }
    } else {
      // No commands, hold position
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "No rate/motor/torque commands received, holding position");
      publishTrajectorySetpoint(0.0, 0.0, -takeoff_altitude_, 0.0);
      publishOffboardControlMode(ControlMode::POSITION);
    }
    break;
  }
}

void Px4SimBridge::publishVehicleCommand(uint16_t command, float param1,
                                         float param2) {
  auto msg = std::make_unique<px4_msgs::msg::VehicleCommand>();
  msg->timestamp = this->get_clock()->now().nanoseconds() / 1000;
  msg->command = command;
  msg->param1 = param1;
  msg->param2 = param2;
  msg->target_system = 1;
  msg->target_component = 1;
  msg->source_system = 1;
  msg->source_component = 1;
  msg->from_external = true;

  vehicle_command_pub_->publish(std::move(msg));
}

void Px4SimBridge::publishOffboardControlMode(ControlMode mode) {
  auto msg = std::make_unique<px4_msgs::msg::OffboardControlMode>();
  msg->timestamp = this->get_clock()->now().nanoseconds() / 1000;

  // Set all to false first
  msg->position = false;
  msg->velocity = false;
  msg->acceleration = false;
  msg->attitude = false;
  msg->body_rate = false;
  msg->thrust_and_torque = false;
  msg->direct_actuator = false;

  // Set the appropriate flag based on mode
  switch (mode) {
  case ControlMode::POSITION:
    msg->position = true;
    break;
  case ControlMode::BODY_RATE:
    msg->body_rate = true;
    break;
  case ControlMode::THRUST_TORQUE:
    msg->thrust_and_torque = true;
    break;
  case ControlMode::DIRECT_ACTUATOR:
    msg->direct_actuator = true;
    break;
  }

  offboard_mode_pub_->publish(std::move(msg));
}

void Px4SimBridge::publishTrajectorySetpoint(float x, float y, float z,
                                             float yaw) {
  auto msg = std::make_unique<px4_msgs::msg::TrajectorySetpoint>();
  msg->timestamp = this->get_clock()->now().nanoseconds() / 1000;
  msg->position[0] = x; // North
  msg->position[1] = y; // East
  msg->position[2] = z; // Down (negative for up in NED)
  msg->yaw = yaw;

  trajectory_pub_->publish(std::move(msg));
}

void Px4SimBridge::armCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

  (void)request; // Unused

  if (armed_) {
    response->success = true;
    response->message = "Already armed";
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Manual arm requested");
  publishVehicleCommand(400, 1.0, 21196.0); // Force arm

  // Wait and check
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  response->success = armed_;
  response->message =
      armed_ ? "Armed successfully" : "Arm command sent, check status";
}

void Px4SimBridge::disarmCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

  (void)request; // Unused

  RCLCPP_INFO(this->get_logger(), "Disarm requested");
  publishVehicleCommand(400, 0.0); // Disarm

  response->success = true;
  response->message = "Disarm command sent";

  // Reset state machine
  state_ = SimState::INIT;
  wait_counter_ = 0;
}

} // namespace px4_sim_bridge_ros2
