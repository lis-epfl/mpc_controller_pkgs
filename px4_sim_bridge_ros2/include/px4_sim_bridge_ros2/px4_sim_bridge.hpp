#ifndef PX4_SIM_BRIDGE_ROS2_HPP_
#define PX4_SIM_BRIDGE_ROS2_HPP_

#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_torque_setpoint.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace px4_sim_bridge_ros2 {

/**
 * @brief State machine states for simulation management
 */
enum class SimState {
  INIT,            // Initial state, waiting for conditions
  ARMING,          // Attempting to arm the vehicle
  SWITCH_OFFBOARD, // Switching to offboard mode
  TAKEOFF,         // Executing takeoff sequence
  READY            // Ready for external control
};

enum class ControlMode {
  POSITION,
  BODY_RATE,
  THRUST_TORQUE,
  DIRECT_ACTUATOR
};

/**
 * @brief PX4 Simulation Bridge
 *
 * Manages the state machine for:
 * - Arming the vehicle
 * - Switching to offboard mode
 * - Executing takeoff sequence
 * - Forwarding rate commands to PX4
 *
 * Does NOT perform coordinate transformations or topic remapping.
 */
class Px4SimBridge : public rclcpp::Node {
public:
  explicit Px4SimBridge(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~Px4SimBridge() = default;

private:
  // Callbacks
  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void rateCommandMonitorCallback(
      const px4_msgs::msg::VehicleRatesSetpoint::SharedPtr msg);
  void torqueCommandMonitorCallback(
      const px4_msgs::msg::VehicleTorqueSetpoint::SharedPtr msg);
  void motorCommandMonitorCallback(
      const px4_msgs::msg::ActuatorMotors::SharedPtr msg);
  void stateMachineCallback();

  // Service callbacks
  void
  armCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
              std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void
  disarmCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Helper functions
  void publishVehicleCommand(uint16_t command, float param1 = 0.0,
                             float param2 = 0.0);
  void publishOffboardControlMode(ControlMode mode = ControlMode::POSITION);
  void publishTrajectorySetpoint(float x, float y, float z, float yaw);

  // Subscriptions
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
      vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr
      rate_cmd_monitor_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr
      torque_cmd_monitor_sub_;
  rclcpp::Subscription<px4_msgs::msg::ActuatorMotors>::SharedPtr
      motor_cmd_monitor_sub_;

  // Publishers
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
      vehicle_command_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
      trajectory_pub_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_service_;

  // Timer for state machine
  rclcpp::TimerBase::SharedPtr state_machine_timer_;

  // State tracking
  SimState state_ = SimState::INIT;
  bool armed_ = false;
  bool offboard_mode_ = false;
  int wait_counter_ = 0;
  rclcpp::Time last_rate_command_time_;
  rclcpp::Time last_torque_command_time_;
  rclcpp::Time last_motor_command_time_;

  // Parameters
  bool auto_arm_ = true;
  bool auto_offboard_ = true;
  double takeoff_altitude_ = 1.0;    // meters
  double state_machine_rate_ = 20.0; // Hz
};

} // namespace px4_sim_bridge_ros2

#endif // PX4_SIM_BRIDGE_ROS2_HPP_
