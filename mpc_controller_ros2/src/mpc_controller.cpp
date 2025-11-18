/**
 * @file mpc_controller.cpp
 * @brief Implementation of the MPC controller class
 */

#include "mpc_controller_ros2/mpc_controller.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <numeric>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr double RPM_TO_RAD_S = 2.0 * M_PI / 60.0;
constexpr double TINY_DT = 1e-6; // For division-by-zero protection

using namespace std::chrono_literals;

namespace mpc_controller_ros2 {

MpcController::MpcController(const rclcpp::NodeOptions &options)
    : Node("mpc_controller", options) {
  declareAndLoadParams();
  initializeSolver();

  // Initialize timestamps to prevent zero dt on first callback
  last_indi_run_time_ = this->get_clock()->now();

  // Initialize filter state vectors
  omega_butter_states_.resize(3);
  rotor_butter_states_.resize(4);

  rclcpp::QoS qos(10);
  rclcpp::QoS sensor_qos(1);
  sensor_qos.best_effort();

  // Subscribe to PX4 topics directly
  odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", sensor_qos,
      std::bind(&MpcController::odometryCallback, this, std::placeholders::_1));

  gyro_sub_ = this->create_subscription<px4_msgs::msg::SensorGyro>(
      "/fmu/out/sensor_gyro", sensor_qos,
      std::bind(&MpcController::gyroCallback, this, std::placeholders::_1));

  traj_sub_ =
      this->create_subscription<mpc_controller_ros2_msgs::msg::Trajectory>(
          traj_topic_, qos,
          std::bind(&MpcController::trajectoryCallback, this,
                    std::placeholders::_1));

  esc_status_sub_ = this->create_subscription<px4_msgs::msg::EscStatus>(
      "/fmu/out/esc_status", sensor_qos,
      std::bind(&MpcController::escStatusCallback, this,
                std::placeholders::_1));

  enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "~/enable", qos,
      std::bind(&MpcController::enableControllerCallback, this,
                std::placeholders::_1));

  // Publishers for PX4 commands
  rates_pub_ = this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>(
      "/fmu/in/vehicle_rates_setpoint", qos);

  torque_pub_ = this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
      "/fmu/in/vehicle_torque_setpoint", qos);

  thrust_pub_ = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
      "/fmu/in/vehicle_thrust_setpoint", qos);

  motors_pub_ = this->create_publisher<px4_msgs::msg::ActuatorMotors>(
      "/fmu/in/actuator_motors", qos);

  trajectory_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("~/mpc_trajectory", qos);

  // Timer for the MPC outer loop
  mpc_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / control_freq_)),
      std::bind(&MpcController::mpcControlLoop, this));

  // If using TORQUE controller with INDI, start the high-frequency INDI inner
  // loop
  if (controller_type_ == "TORQUE" && indi_enabled_) {
    indi_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / indi_freq_)),
        std::bind(&MpcController::indiControlLoop, this));
    RCLCPP_INFO(this->get_logger(), "INDI loop started at %.1f Hz", indi_freq_);
  }

  trajectory_logger_ = std::make_unique<TrajectoryLogger>(log_directory_);

  // Start logging immediately if enabled
  if (enable_logging_ && controller_enabled_) {
    trajectory_logger_->startNewLog();
    RCLCPP_INFO(this->get_logger(), "Trajectory logging started: %s",
                trajectory_logger_->getCurrentLogPath().c_str());
  }

  RCLCPP_INFO(this->get_logger(),
              "MPC Controller Node initialized with direct PX4 interface");
  RCLCPP_INFO(this->get_logger(), "Controller type: %s",
              controller_type_.c_str());
  RCLCPP_INFO(this->get_logger(), "Controller initially %s",
              controller_enabled_ ? "ENABLED" : "DISABLED");
  RCLCPP_INFO(this->get_logger(), "MPC Frequency: %.1f Hz", control_freq_);

  if (controller_type_ == "TORQUE") {
    if (!indi_enabled_) {
      RCLCPP_INFO(this->get_logger(), "Using direct torque/thrust commands");
    } else {
      if (use_direct_torque_) {
        RCLCPP_INFO(this->get_logger(),
                    "Using INDI with torque/thrust commands at %.1f Hz",
                    indi_freq_);
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "Using INDI with motor commands at %.1f Hz", indi_freq_);
      }
    }
  }
}

MpcController::~MpcController() {
  RCLCPP_INFO(this->get_logger(), "=== Final Controller Statistics ===");

  // Print MPC statistics
  printTimingStatistics("MPC", computation_times_ms_, control_freq_);

  // Print INDI statistics if applicable
  if (controller_type_ == "TORQUE" && indi_enabled_) {
    std::lock_guard<std::mutex> lock(indi_stats_mutex_);
    printTimingStatistics("INDI", indi_computation_times_ms_, indi_freq_);
  }

  if (enable_logging_ && trajectory_logger_) {
    trajectory_logger_->stopLogging();
  }
}

void MpcController::enableControllerCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {

  bool should_start_logging = false;

  {
    std::lock_guard<std::mutex> lock(controller_enabled_mutex_);
    if (msg->data) {
      if (!controller_enabled_) {
        RCLCPP_INFO(this->get_logger(), "Controller ENABLED via topic");
        controller_enabled_ = true;
        last_indi_run_time_ = this->get_clock()->now();
        should_start_logging = enable_logging_;
        {
          std::lock_guard<std::mutex> lock(x_init_mutex_);
          x_init_.clear();
        }
        {
          std::lock_guard<std::mutex> lock(trajectory_mutex_);
          trajectory_received_ = false;
          ref_trajectory_.clear();
        }
      }
    } else {
      if (controller_enabled_) {
        RCLCPP_WARN(this->get_logger(), "Controller DISABLED via topic");
        controller_enabled_ = false;
        should_start_logging = false; // Will trigger stop logging below
      }
    }
  }

  if (msg->data && should_start_logging && trajectory_logger_) {
    trajectory_logger_->startNewLog();
    RCLCPP_INFO(this->get_logger(), "Started new log file: %s",
                trajectory_logger_->getCurrentLogPath().c_str());
  } else if (!msg->data && enable_logging_ && trajectory_logger_) {
    trajectory_logger_->stopLogging();
    RCLCPP_INFO(this->get_logger(), "Stopped logging");
  }
}

void MpcController::printTimingStatistics(const std::string &name,
                                          const std::vector<double> &times_ms,
                                          double expected_freq_hz) {
  if (times_ms.empty()) {
    RCLCPP_INFO(this->get_logger(), "%s: No samples recorded yet",
                name.c_str());
    return;
  }

  // Basic statistics
  double sum = std::accumulate(times_ms.begin(), times_ms.end(), 0.0);
  double mean = sum / times_ms.size();

  auto minmax = std::minmax_element(times_ms.begin(), times_ms.end());
  double min = *minmax.first;
  double max = *minmax.second;

  // Standard deviation
  double sq_sum = 0.0;
  for (double time : times_ms) {
    sq_sum += (time - mean) * (time - mean);
  }
  double std_dev = std::sqrt(sq_sum / times_ms.size());

  // Percentiles
  std::vector<double> sorted = times_ms;
  std::sort(sorted.begin(), sorted.end());

  size_t p50_idx = sorted.size() / 2;
  size_t p95_idx = static_cast<size_t>(sorted.size() * 0.95);
  size_t p99_idx = static_cast<size_t>(sorted.size() * 0.99);

  double p50 = sorted[p50_idx];
  double p95 = (p95_idx < sorted.size()) ? sorted[p95_idx] : sorted.back();
  double p99 = (p99_idx < sorted.size()) ? sorted[p99_idx] : sorted.back();

  // Timing violations (exceeding 80% of period)
  double expected_period_ms = 1000.0 / expected_freq_hz;
  size_t violations = 0;
  for (double time : times_ms) {
    if (time > expected_period_ms * 0.8) {
      violations++;
    }
  }

  double actual_freq = 1000.0 / mean;

  RCLCPP_INFO(this->get_logger(),
              "%s Statistics:\n"
              "  Timing: Mean=%.3fms, StdDev=%.3fms, Min=%.3fms, Max=%.3fms\n"
              "  Percentiles: 50th=%.3fms, 95th=%.3fms, 99th=%.3fms\n"
              "  Samples: %zu, Violations: %zu (%.1f%%)\n"
              "  Frequency: Expected=%.1fHz, Actual=%.1fHz (%.1f%% of target)",
              name.c_str(), mean, std_dev, min, max, p50, p95, p99,
              times_ms.size(), violations, 100.0 * violations / times_ms.size(),
              expected_freq_hz, actual_freq,
              (actual_freq / expected_freq_hz) * 100.0);
}

void MpcController::declareAndLoadParams() {
  // Controller parameters
  this->declare_parameter<std::string>("controller.type", "RATE");
  this->declare_parameter<bool>("controller.enabled", false);
  this->declare_parameter<bool>("controller.verbose", false);
  this->declare_parameter<double>("indi.frequency", 300.0);
  this->declare_parameter<bool>("indi.enabled", false);
  this->declare_parameter<bool>("indi.use_direct_torque", false);

  // Drone parameters
  this->declare_parameter<double>("drone.mass", 1.5);
  this->declare_parameter<double>("drone.Ixx", 0.029);
  this->declare_parameter<double>("drone.Iyy", 0.029);
  this->declare_parameter<double>("drone.Izz", 0.055);
  this->declare_parameter<double>("drone.kx", 0.1);
  this->declare_parameter<double>("drone.ky", 0.1);
  this->declare_parameter<double>("drone.kz", 0.1);
  this->declare_parameter<double>("drone.kh", 0.0);
  this->declare_parameter<double>("drone.thrust_min", 0.1);
  this->declare_parameter<double>("drone.thrust_max", 20.0);
  this->declare_parameter<double>("drone.rate_max_xy", 4.0);
  this->declare_parameter<double>("drone.rate_max_z", 2.0);
  this->declare_parameter<double>("drone.torque_max_xy", 0.5);
  this->declare_parameter<double>("drone.torque_max_z", 0.1);

  // MPC parameters
  this->declare_parameter<double>("mpc.horizon_decay_factor", 0.1);
  this->declare_parameter<std::string>("mpc.horizon_decay_type", "linear");
  this->declare_parameter<double>("mpc.control_frequency", 100.0);
  this->declare_parameter<double>("mpc.computation_delay", 0.01);
  this->declare_parameter<std::vector<double>>(
      "mpc.Q_rate_diag", std::vector<double>{100.0, 100.0, 100.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0, 0.0});
  this->declare_parameter<std::vector<double>>(
      "mpc.R_rate_diag", std::vector<double>{0.1, 0.05, 0.05, 0.02});
  this->declare_parameter<std::vector<double>>(
      "mpc.Q_torque_diag",
      std::vector<double>{100.0, 100.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.01, 0.01, 0.01});
  this->declare_parameter<std::vector<double>>(
      "mpc.R_torque_diag", std::vector<double>{0.1, 0.05, 0.05, 0.02});
  this->declare_parameter<double>("mpc.command_scale_min", 0.8);
  this->declare_parameter<double>("mpc.command_scale_max", 1.2);
  this->declare_parameter<double>("mpc.command_scale_gain",
                                  0.0); // Default 0 to disable if not tuned

  // Topic configuration - only trajectory is configurable
  this->declare_parameter<std::string>("topics.trajectory",
                                       "/planner/trajectory");

  // Physical parameters for INDI
  this->declare_parameter<double>("drone.arm_length", 0.14);
  this->declare_parameter<double>("drone.arm_angle_deg", 45.0);
  this->declare_parameter<double>("drone.rotor_inertia", 2.37e-8);
  this->declare_parameter<double>("drone.thrust_coefficient", 1.51e-6);
  this->declare_parameter<double>("drone.torque_coefficient", 2.37e-8);
  this->declare_parameter<std::string>("indi.filter_type", "butterworth");
  this->declare_parameter<double>("indi.filter_cutoff_hz", 12.0);
  this->declare_parameter<double>("drone.max_rotor_rpm", 25000.0);
  this->declare_parameter<double>("drone.esc_scale", 1.0);

  // Logging parameters
  this->declare_parameter<bool>("logging.enabled", false);
  this->declare_parameter<std::string>("logging.directory", "/tmp/mpc_logs");

  // Load all parameters
  this->get_parameter("controller.type", controller_type_);
  this->get_parameter("controller.enabled", controller_enabled_);
  this->get_parameter("controller.verbose", verbose_);
  this->get_parameter("indi.frequency", indi_freq_);
  this->get_parameter("indi.enabled", indi_enabled_);
  this->get_parameter("indi.use_direct_torque", use_direct_torque_);
  this->get_parameter("mpc.horizon_decay_factor", horizon_decay_factor_);
  this->get_parameter("mpc.horizon_decay_type", horizon_decay_type_);
  this->get_parameter("mpc.control_frequency", control_freq_);
  this->get_parameter("mpc.computation_delay", mpc_computation_delay_);
  this->get_parameter("topics.trajectory", traj_topic_);

  this->get_parameter("drone.mass", mass_);
  this->get_parameter("drone.Ixx", Ixx_);
  this->get_parameter("drone.Iyy", Iyy_);
  this->get_parameter("drone.Izz", Izz_);
  this->get_parameter("drone.kx", kx_);
  this->get_parameter("drone.ky", ky_);
  this->get_parameter("drone.kz", kz_);
  this->get_parameter("drone.kh", kh_);
  this->get_parameter("drone.thrust_min", thrust_min_);
  this->get_parameter("drone.thrust_max", thrust_max_);
  this->get_parameter("drone.rate_max_xy", rate_max_xy_);
  this->get_parameter("drone.rate_max_z", rate_max_z_);
  this->get_parameter("drone.torque_max_xy", torque_max_xy_);
  this->get_parameter("drone.torque_max_z", torque_max_z_);
  this->get_parameter("drone.arm_length", arm_length_);
  double arm_angle_deg;
  this->get_parameter("drone.arm_angle_deg", arm_angle_deg);
  arm_angle_rad_ = arm_angle_deg * M_PI / 180.0;
  this->get_parameter("drone.rotor_inertia", rotor_inertia_);
  this->get_parameter("drone.thrust_coefficient", thrust_coeff_);
  this->get_parameter("drone.torque_coefficient", torque_coeff_);
  this->get_parameter("indi.filter_type", indi_filter_type_);
  this->get_parameter("indi.filter_cutoff_hz", indi_cutoff_freq_);
  this->get_parameter("drone.max_rotor_rpm", max_rotor_rpm_);
  this->get_parameter("drone.esc_scale", esc_scale_);
  max_rotor_speed_rad_s_ = max_rotor_rpm_ * RPM_TO_RAD_S;

  this->get_parameter("logging.enabled", enable_logging_);
  this->get_parameter("logging.directory", log_directory_);

  Q_rate_diag_ = this->get_parameter("mpc.Q_rate_diag").as_double_array();
  R_rate_diag_ = this->get_parameter("mpc.R_rate_diag").as_double_array();
  Q_torque_diag_ = this->get_parameter("mpc.Q_torque_diag").as_double_array();
  R_torque_diag_ = this->get_parameter("mpc.R_torque_diag").as_double_array();

  std::vector<double> default_gyro_transform = {1.0, 0.0, 0.0, 0.0, -1.0,
                                                0.0, 0.0, 0.0, -1.0};
  this->declare_parameter<std::vector<double>>("drone.gyro_transform_matrix",
                                               default_gyro_transform);

  this->get_parameter("mpc.command_scale_min", command_scale_min_);
  this->get_parameter("mpc.command_scale_max", command_scale_max_);
  this->get_parameter("mpc.command_scale_gain", command_scale_gain_);

  std::vector<double> gyro_transform_vec;
  this->get_parameter("drone.gyro_transform_matrix", gyro_transform_vec);

  if (gyro_transform_vec.size() == 9) {
    // Load as row-major, which Eigen::Map handles by default
    gyro_transform_matrix_ =
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            gyro_transform_vec.data());
    RCLCPP_INFO_STREAM(this->get_logger(), "Loaded Gyro Transform Matrix:\n"
                                               << gyro_transform_matrix_);
  } else {
    RCLCPP_ERROR(
        this->get_logger(),
        "drone.gyro_transform_matrix must have 9 elements! Using default.");
    gyro_transform_matrix_ =
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            default_gyro_transform.data());
  }
}

void MpcController::odometryCallback(
    px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  int nx = solver_->getNx();
  if (static_cast<int>(current_state_.size()) != nx) {
    current_state_.resize(nx, 0.0);
  }

  // Position: FRD to FLU
  current_state_[0] = msg->position[0];
  current_state_[1] = -msg->position[1];
  current_state_[2] = -msg->position[2];

  // Velocity: FRD to FLU
  current_state_[3] = msg->velocity[0];
  current_state_[4] = -msg->velocity[1];
  current_state_[5] = -msg->velocity[2];

  // Quaternion: FRD to FLU transformation
  Eigen::Quaterniond q_frd(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

  // Quaternion FRD to FLU
  Eigen::Quaterniond q_flu;
  q_flu.w() = q_frd.w();
  q_flu.x() = q_frd.x();
  q_flu.y() = -q_frd.y();
  q_flu.z() = -q_frd.z();

  q_flu.normalize();

  current_state_[6] = q_flu.w();
  current_state_[7] = q_flu.x();
  current_state_[8] = q_flu.y();
  current_state_[9] = q_flu.z();

  // If using torque controller, also set angular velocities
  if (controller_type_ == "TORQUE" && current_state_.size() >= 13) {
    // Angular velocity: FRD to FLU
    current_state_[10] = msg->angular_velocity[0];  // Roll rate
    current_state_[11] = -msg->angular_velocity[1]; // Pitch rate
    current_state_[12] = -msg->angular_velocity[2]; // Yaw rate
  }

  last_odometry_timestamp_ = this->get_clock()->now();
  pose_received_ = true;
}

void MpcController::gyroCallback(px4_msgs::msg::SensorGyro::UniquePtr msg) {

  std::lock_guard<std::mutex> lock(state_mutex_);
  // Create input vector from gyro message
  Eigen::Vector3d gyro_in(msg->x, msg->y, msg->z);

  // Apply the loaded transformation matrix
  Eigen::Vector3d gyro_out_mpc = gyro_transform_matrix_ * gyro_in;

  double new_wx = gyro_out_mpc.x();
  double new_wy = gyro_out_mpc.y();
  double new_wz = gyro_out_mpc.z();

  if (new_wx != 0.0) {
    last_good_angular_velocity_.x() = new_wx;
  }
  if (new_wy != 0.0) {
    last_good_angular_velocity_.y() = new_wy;
  }
  if (new_wz != 0.0) {
    last_good_angular_velocity_.z() = new_wz;
  }
  current_angular_velocity_ = last_good_angular_velocity_;
}

void MpcController::escStatusCallback(px4_msgs::msg::EscStatus::UniquePtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < 4; ++i) {
    double new_rpm = static_cast<double>(msg->esc[i].esc_rpm);
    if (new_rpm > 0.0) {
      last_good_rotor_speeds_[i] = new_rpm * RPM_TO_RAD_S * esc_scale_;
    }
    current_rotor_speeds_[i] = last_good_rotor_speeds_[i];
  }
}

void MpcController::publishUnifiedCommand(const float thrust_cmd) {

  uint64_t timestamp = this->get_clock()->now().nanoseconds() / 1000;

  if (controller_type_ == "RATE") {
    // Publish VehicleRatesSetpoint with 0.05 normalized thrust and zero rates
    auto msg = std::make_unique<px4_msgs::msg::VehicleRatesSetpoint>();
    msg->timestamp = timestamp;
    msg->roll = 0.0f;
    msg->pitch = 0.0f;
    msg->yaw = 0.0f;
    msg->thrust_body[0] = 0.0f;
    msg->thrust_body[1] = 0.0f;
    // PX4 expects negative for upward thrust, [0, 1] normalized
    msg->thrust_body[2] = -thrust_cmd;
    rates_pub_->publish(std::move(msg));

  } else if (controller_type_ == "TORQUE") {
    if (!indi_enabled_ || (indi_enabled_ && use_direct_torque_)) {
      // Publish 0.05 normalized thrust and zero torque
      // This covers:
      // 1. Torque controller without INDI
      // 2. Torque controller with INDI set to 'use_direct_torque'

      // 1. Thrust message
      auto thrust_msg =
          std::make_unique<px4_msgs::msg::VehicleThrustSetpoint>();
      thrust_msg->timestamp = timestamp;
      thrust_msg->xyz[0] = 0.0f;
      thrust_msg->xyz[1] = 0.0f;
      // PX4 expects negative for upward thrust, [0, 1] normalized
      thrust_msg->xyz[2] = -thrust_cmd;
      thrust_pub_->publish(std::move(thrust_msg));

      // 2. Torque message
      auto torque_msg =
          std::make_unique<px4_msgs::msg::VehicleTorqueSetpoint>();
      torque_msg->timestamp = timestamp;
      torque_msg->xyz[0] = 0.0f;
      torque_msg->xyz[1] = 0.0f;
      torque_msg->xyz[2] = 0.0f;
      torque_pub_->publish(std::move(torque_msg));

    } else {
      // This is the case for:
      // 1. Torque controller with INDI
      // 2. 'use_direct_torque' is false (i.e., using motor commands)

      // Publish ActuatorMotors with 0.05 normalized to first 4 motors
      auto msg = std::make_unique<px4_msgs::msg::ActuatorMotors>();
      msg->timestamp = timestamp;

      // Set first 4 motors to idle value
      for (size_t i = 0; i < 4; ++i) {
        msg->control[i] = thrust_cmd;
      }

      // Set unused motors to NaN (PX4 convention)
      for (size_t i = 4; i < 12; ++i) {
        msg->control[i] = NAN;
      }
      motors_pub_->publish(std::move(msg));
    }
  }
}

void MpcController::publishRateCommand(double thrust, double wx, double wy,
                                       double wz) {
  auto msg = std::make_unique<px4_msgs::msg::VehicleRatesSetpoint>();

  uint64_t timestamp =
      this->get_clock()->now().nanoseconds() / 1000; // microseconds
  msg->timestamp = timestamp;

  // Transform FRD to FLU
  msg->roll = wx;
  msg->pitch = -wy;
  msg->yaw = -wz;

  // Normalize thrust to [-1, 1] range for PX4
  double normalized_thrust = thrust / 4;
  normalized_thrust =
      sqrt(normalized_thrust / thrust_coeff_) / max_rotor_speed_rad_s_;
  normalized_thrust =
      std::clamp(normalized_thrust* command_scale_.load(), -1.0, 1.0);

  // PX4 expects thrust in body z-axis (downward in NED)
  msg->thrust_body[0] = 0.0;
  msg->thrust_body[1] = 0.0;
  msg->thrust_body[2] = -normalized_thrust; // Negative for upward thrust in NED

  rates_pub_->publish(std::move(msg));
}

void MpcController::publishTorqueCommand(double thrust, double tau_x,
                                         double tau_y, double tau_z) {
  // Publish thrust setpoint
  auto thrust_msg = std::make_unique<px4_msgs::msg::VehicleThrustSetpoint>();
  uint64_t timestamp =
      this->get_clock()->now().nanoseconds() / 1000; // microseconds
  thrust_msg->timestamp = timestamp;

  // Thrust is only in body z-axis, other components are zero
  double normalized_thrust = thrust / 4;
  normalized_thrust =
      sqrt(normalized_thrust / thrust_coeff_) / max_rotor_speed_rad_s_;
  normalized_thrust =
      std::clamp(normalized_thrust * command_scale_.load(), -1.0, 1.0);

  thrust_msg->xyz[0] = 0.0;
  thrust_msg->xyz[1] = 0.0;
  thrust_msg->xyz[2] = -normalized_thrust; // Negative because FRD z is down,
                                           // thrust is up

  thrust_pub_->publish(std::move(thrust_msg));

  // Publish torque setpoint
  auto torque_msg = std::make_unique<px4_msgs::msg::VehicleTorqueSetpoint>();
  torque_msg->timestamp = timestamp;

  // Transform FRD to FLU
  torque_msg->xyz[0] = tau_x;
  torque_msg->xyz[1] = -tau_y;
  torque_msg->xyz[2] = -tau_z;

  torque_pub_->publish(std::move(torque_msg));
}

void MpcController::publishMotorCommand(
    const std::vector<double> &motor_commands) {
  auto msg = std::make_unique<px4_msgs::msg::ActuatorMotors>();

  uint64_t timestamp = this->get_clock()->now().nanoseconds() / 1000;
  msg->timestamp = timestamp;

  // Convert rotor speeds (rad/s) to normalized values [0, 1]
  for (size_t i = 0; i < motor_commands.size() && i < 12; ++i) {
    // Normalize based on max rotor speed
    float normalized =
        static_cast<float>(motor_commands[i] / max_rotor_speed_rad_s_);
    normalized = std::clamp(normalized * command_scale_.load(), 0.0, 1.0);
    msg->control[i] = normalized;
  }

  // Set unused motors to NaN (PX4 convention)
  for (size_t i = motor_commands.size(); i < 12; ++i) {
    msg->control[i] = NAN;
  }

  motors_pub_->publish(std::move(msg));
}

void MpcController::indiControlLoop() {
  {
    std::lock_guard<std::mutex> lock(controller_enabled_mutex_);
    if (!controller_enabled_) {
      return;
    }
  }

  if (!pose_received_) {
    return;
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  double mpc_thrust_cmd;
  Eigen::Vector3d mpc_torques_cmd;
  {
    std::lock_guard<std::mutex> lock(mpc_output_mutex_);
    mpc_thrust_cmd = latest_mpc_thrust_;
    mpc_torques_cmd = latest_mpc_torques_;
  }

  if (!use_direct_torque_) {
    std::vector<double> rotor_speed_cmds =
        runIndiController(mpc_thrust_cmd, mpc_torques_cmd);
    publishMotorCommand(rotor_speed_cmds);
  } else {
    std::vector<double> thrust_torque_cmds =
        runIndiController(mpc_thrust_cmd, mpc_torques_cmd);
    publishTorqueCommand(thrust_torque_cmds[0], thrust_torque_cmds[1],
                         thrust_torque_cmds[2], thrust_torque_cmds[3]);
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  double comp_time_ms =
      std::chrono::duration<double, std::milli>(end_time - start_time).count();

  {
    std::lock_guard<std::mutex> lock(indi_stats_mutex_);
    indi_computation_times_ms_.push_back(comp_time_ms);

    // Limit buffer size (keep last 10000 samples)
    if (indi_computation_times_ms_.size() > 10000) {
      indi_computation_times_ms_.erase(indi_computation_times_ms_.begin());
    }
  }
}

void MpcController::initializeSolver() {
  // Create only the solver that's needed
  if (controller_type_ == "RATE") {
    solver_ = std::make_unique<MPCRateSolver>(this->get_logger());
  } else if (controller_type_ == "TORQUE") {
    solver_ = std::make_unique<MPCTorqueSolver>(this->get_logger());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unknown controller type: %s",
                 controller_type_.c_str());
    rclcpp::shutdown();
    return;
  }

  // Initialize the solver
  if (!solver_->initialize()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize %s solver",
                 controller_type_.c_str());
    rclcpp::shutdown();
    return;
  }

  n_ = solver_->getNHorizon();
  time_step_ = solver_->getTimeStep();

  // Set parameters based on controller type
  if (controller_type_ == "RATE") {
    std::vector<double> params = {mass_, kx_, ky_, kz_, kh_};
    solver_->setParameters(params);
    solver_->setCostMatrices(Q_rate_diag_, R_rate_diag_);

    // Apply stage-varying weights if decay factor is not zero
    if (horizon_decay_factor_ > 0.0) {
      solver_->setStageVaryingCostMatrices(Q_rate_diag_, R_rate_diag_,
                                           horizon_decay_factor_,
                                           horizon_decay_type_);
    }
  } else { // TORQUE
    std::vector<double> params = {mass_, Ixx_, Iyy_, Izz_, kx_, ky_, kz_, kh_};
    solver_->setParameters(params);
    solver_->setCostMatrices(Q_torque_diag_, R_torque_diag_);

    if (horizon_decay_factor_ > 0.0) {
      solver_->setStageVaryingCostMatrices(Q_torque_diag_, R_torque_diag_,
                                           horizon_decay_factor_,
                                           horizon_decay_type_);
    }
  }

  // Set control bounds
  updateControlBounds();

  // Initialize state vectors
  int nx = solver_->getNx();
  int nu = solver_->getNu();

  current_state_.resize(nx, 0.0);
  current_state_[6] = 1.0; // qw = 1
  predicted_state_cache_.resize(nx, 0.0);
  predicted_state_cache_[6] = 1.0;
  last_control_input_.resize(nu, 0.0);
  last_control_input_[0] = mass_ * 9.81; // Hover thrust

  RCLCPP_INFO(this->get_logger(), "%s solver initialized successfully",
              controller_type_.c_str());
}

void MpcController::updateCostMatrices() {
  if (controller_type_ == "RATE") {
    solver_->setCostMatrices(Q_rate_diag_, R_rate_diag_);
    if (horizon_decay_factor_ > 0.0) {
      solver_->setStageVaryingCostMatrices(Q_rate_diag_, R_rate_diag_,
                                           horizon_decay_factor_,
                                           horizon_decay_type_);
    }
  } else { // TORQUE
    solver_->setCostMatrices(Q_torque_diag_, R_torque_diag_);
    if (horizon_decay_factor_ > 0.0) {
      solver_->setStageVaryingCostMatrices(Q_torque_diag_, R_torque_diag_,
                                           horizon_decay_factor_,
                                           horizon_decay_type_);
    }
  }
}

void MpcController::updateControlBounds() {
  if (controller_type_ == "RATE") {
    std::vector<double> lbu = {thrust_min_, -rate_max_xy_, -rate_max_xy_,
                               -rate_max_z_};
    std::vector<double> ubu = {thrust_max_, rate_max_xy_, rate_max_xy_,
                               rate_max_z_};
    static_cast<MPCRateSolver *>(solver_.get())->setControlBounds(lbu, ubu);
  } else { // TORQUE
    std::vector<double> lbu = {thrust_min_, -torque_max_xy_, -torque_max_xy_,
                               -torque_max_z_};
    std::vector<double> ubu = {thrust_max_, torque_max_xy_, torque_max_xy_,
                               torque_max_z_};
    static_cast<MPCTorqueSolver *>(solver_.get())->setControlBounds(lbu, ubu);
  }
}

void MpcController::trajectoryCallback(
    mpc_controller_ros2_msgs::msg::Trajectory::UniquePtr msg) {

  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  if (msg->states.empty())
    return;

  ref_trajectory_.clear();

  // Get current time
  rclcpp::Time current_time = this->get_clock()->now();
  double current_time_sec = current_time.seconds();

  // Extract trajectory start time from t_0
  double t_0 = msg->t_0;
  double dt = msg->dt;

  // Check if trajectory starts in the future
  if (t_0 > current_time_sec) {
    if (verbose_) {
      RCLCPP_WARN(this->get_logger(),
                  "Trajectory starts %.3f seconds in the future. Extrapolating "
                  "backwards.",
                  t_0 - current_time_sec);
    }

    // Calculate how many steps back we need
    double time_diff = t_0 - current_time_sec;
    int num_steps_back = static_cast<int>(std::ceil(time_diff / dt));

    // Get the first state for extrapolation
    const auto &first_state = msg->states.front();

    // Extrapolate backwards using velocity
    std::vector<mpc_controller_ros2_msgs::msg::TrajectoryState> extra_states;
    for (int i = num_steps_back; i > 0; --i) {
      auto state = mpc_controller_ros2_msgs::msg::TrajectoryState();

      // Extrapolate position backwards: pos = first_pos - velocity * (i * dt)
      state.position.x =
          first_state.position.x - first_state.velocity.x * (i * dt);
      state.position.y =
          first_state.position.y - first_state.velocity.y * (i * dt);
      state.position.z =
          first_state.position.z - first_state.velocity.z * (i * dt);

      // Keep velocity constant (constant velocity extrapolation)
      state.velocity.x = first_state.velocity.x;
      state.velocity.y = first_state.velocity.y;
      state.velocity.z = first_state.velocity.z;

      // Keep same orientation
      state.orientation.w = first_state.orientation.w;
      state.orientation.x = first_state.orientation.x;
      state.orientation.y = first_state.orientation.y;
      state.orientation.z = first_state.orientation.z;

      // Keep angular velocity constant
      state.angular_velocity.x = first_state.angular_velocity.x;
      state.angular_velocity.y = first_state.angular_velocity.y;
      state.angular_velocity.z = first_state.angular_velocity.z;

      extra_states.push_back(state);
    }

    // Update t_0 to the new start time
    t_0 = current_time_sec - (num_steps_back * dt);

    RCLCPP_INFO(
        this->get_logger(),
        "Added %d extrapolated states before trajectory start. New t_0: %.3f",
        num_steps_back, t_0);

    // Add extrapolated states to the beginning
    for (const auto &state : extra_states) {
      msg->states.insert(msg->states.begin(), state);
    }
  }

  // Get the solver's state dimension
  int nx = solver_->getNx();

  // Add all the states from the message
  for (const auto &state : msg->states) {
    std::vector<double> point(nx, 0.0);

    // Position from geometry_msgs/Point
    point[0] = state.position.x;
    point[1] = state.position.y;
    point[2] = state.position.z;

    // Velocity from geometry_msgs/Vector3
    point[3] = state.velocity.x;
    point[4] = state.velocity.y;
    point[5] = state.velocity.z;

    // Quaternion from geometry_msgs/Quaternion
    point[6] = state.orientation.w;
    point[7] = state.orientation.x;
    point[8] = state.orientation.y;
    point[9] = state.orientation.z;

    // If using torque controller, also get angular velocities
    if (nx >= 13) {
      point[10] = state.angular_velocity.x;
      point[11] = state.angular_velocity.y;
      point[12] = state.angular_velocity.z;
    }

    ref_trajectory_.push_back(point);
  }

  trajectory_dt_ = dt;
  trajectory_received_ = true;

  // Use the (possibly modified) t_0
  int32_t sec = static_cast<int32_t>(t_0);
  uint32_t nanosec = static_cast<uint32_t>((t_0 - sec) * 1e9);
  trajectory_start_time_ =
      rclcpp::Time(sec, nanosec, this->get_clock()->get_clock_type());

  RCLCPP_DEBUG(this->get_logger(),
               "Trajectory received: %zu states, dt=%.3f, t_0=%.3f",
               ref_trajectory_.size(), trajectory_dt_, t_0);
}

void MpcController::publishMPCTrajectory() {
  auto trajectory_msg = std::make_unique<nav_msgs::msg::Path>();
  trajectory_msg->header.stamp = this->get_clock()->now();
  trajectory_msg->header.frame_id = "world";

  for (int i = 0; i <= n_; ++i) {
    std::vector<double> x(solver_->getNx());
    solver_->getState(i, x);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = trajectory_msg->header.stamp;
    pose.header.frame_id = "world";
    pose.pose.position.x = x[0];
    pose.pose.position.y = x[1];
    pose.pose.position.z = x[2];
    pose.pose.orientation.w = x[6];
    pose.pose.orientation.x = x[7];
    pose.pose.orientation.y = x[8];
    pose.pose.orientation.z = x[9];
    trajectory_msg->poses.push_back(pose);
  }

  trajectory_pub_->publish(std::move(trajectory_msg));
}

void MpcController::mpcControlLoop() {
  {
    std::lock_guard<std::mutex> lock(controller_enabled_mutex_);
    if (!controller_enabled_) {
      return;
    }
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  static int mpc_call_count = 0;

  // Get current state from ROS
  std::vector<double> x_current_local;
  rclcpp::Time state_timestamp;

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!pose_received_) {
      if (verbose_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Waiting for pose...");
      }
      return;
    }
    x_current_local = current_state_;
    state_timestamp = last_odometry_timestamp_;
  }

  // State prediction to compensate for delays
  rclcpp::Time current_time = this->get_clock()->now();
  double measurement_delay = (current_time - state_timestamp).seconds();
  double total_prediction_time = measurement_delay + mpc_computation_delay_;

  if (total_prediction_time > 0.001) { // 1ms threshold
    if (verbose_) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Predicting state forward by %.3f ms (measurement: %.3f ms, "
                   "computation: %.3f ms)",
                   total_prediction_time * 1000, measurement_delay * 1000,
                   mpc_computation_delay_ * 1000);
    }
    predictState(x_current_local, total_prediction_time);
  }

  double time_since_trajectory_start = 0;

  // Prepare reference trajectory storage for reuse
  std::vector<std::vector<double>> horizon_references;
  std::vector<double> terminal_reference;

  // Compute reference trajectory once
  {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);

    if (trajectory_received_ && !ref_trajectory_.empty()) {
      time_since_trajectory_start =
          (current_time - trajectory_start_time_).seconds();

      // Compute references for all horizon points
      for (int i = 0; i <= n_; ++i) {
        double horizon_time = i * time_step_ + total_prediction_time +
                              time_since_trajectory_start;
        std::vector<double> ref_point = interpolateReference(horizon_time);

        if (ref_point.empty()) { // If interpolation fails, fall back to
                                 // position hold
          ref_point.resize(10, 0.0);
          ref_point[0] = x_current_local[0]; // Hold current position
          ref_point[1] = x_current_local[1];
          ref_point[2] = x_current_local[2];
          ref_point[6] = 1.0; // qw
          ref_point[7] = 0.0; // qx
          ref_point[8] = 0.0; // qy
          ref_point[9] = 0.0; // qz
        }

        horizon_references.push_back(ref_point);
      }

      // Terminal reference is the last horizon point
      terminal_reference = horizon_references.back();

    } else {
      // if we are still on the ground (less then 15 cm) without any takeoff
      // trajectory yet, publish idle command
      if (x_current_local[2] <= 0.15) {
        thrust_cmd_ = 0.01;
        publishUnifiedCommand(thrust_cmd_);
        idle_ = true;
        command_scale_.store(1.0);
        return;
      }
      std::lock_guard<std::mutex> lock(x_init_mutex_);
      // No trajectory - hold initial position
      if (x_init_.empty()) {
        x_init_.resize(3, 0.0);
        x_init_[0] = x_current_local[0];
        x_init_[1] = x_current_local[1];
        x_init_[2] = x_current_local[2];
      }
      for (int i = 0; i <= n_; ++i) {
        std::vector<double> ref_point(10, 0.0);
        ref_point[0] = x_init_[0];
        ref_point[1] = x_init_[1];
        ref_point[2] = x_init_[2];
        ref_point[6] = 1.0; // qw
        ref_point[7] = 0.0; // qx
        ref_point[8] = 0.0; // qy
        ref_point[9] = 0.0; // qz

        horizon_references.push_back(ref_point);
      }

      terminal_reference = horizon_references.back();
    }
  }

  if (idle_ == true) {
    // take current thrust command and ramp it up
    thrust_cmd_ = thrust_cmd_ + 0.006 * control_freq_ / 100;
    publishUnifiedCommand(thrust_cmd_);
    if (thrust_cmd_ >= mass_ * 9.81 / thrust_max_) {
      idle_ = false;
    }
    return;
  }

  // update scale
  double dt = 1 / control_freq_;
  double z_error = horizon_references[0][2] - x_current_local[2];
  double current_scale = command_scale_.load();
  current_scale += dt * command_scale_gain_ * z_error;
  current_scale = std::clamp(current_scale, command_scale_min_, command_scale_max_);
  command_scale_.store(current_scale);

  for (int i = 0; i < n_; ++i) {
    std::vector<double> yref(solver_->getNy(), 0.0);

    // Copy available states from precomputed reference
    const auto &ref_point = horizon_references[i];
    int states_to_copy = std::min(solver_->getNx(), (int)ref_point.size());
    for (int j = 0; j < states_to_copy; ++j) {
      yref[j] = ref_point[j];
    }

    // Set control reference based on controller type
    if (controller_type_ == "RATE") {
      yref[10] = mass_ * 9.81; // hover thrust
    } else {                   // TORQUE
      yref[13] = mass_ * 9.81; // hover thrust
    }

    solver_->setReference(i, yref);
  }

  // Set terminal reference
  std::vector<double> yref_e(solver_->getNx(), 0.0);
  int states_to_copy =
      std::min(solver_->getNx(), (int)terminal_reference.size());
  for (int j = 0; j < states_to_copy; ++j) {
    yref_e[j] = terminal_reference[j];
  }
  solver_->setTerminalReference(yref_e);

  // Set initial state
  solver_->setInitialState(x_current_local);

  // Solve
  int status = solver_->solve();

  if (status != 0) {
    RCLCPP_ERROR(this->get_logger(), "MPC failed with status %d at call %d",
                 status, mpc_call_count);

    if (verbose_) {
      RCLCPP_INFO(
          this->get_logger(),
          "Current state: pos=(%.3f,%.3f,%.3f) quat=(%.3f,%.3f,%.3f,%.3f)",
          x_current_local[0], x_current_local[1], x_current_local[2],
          x_current_local[6], x_current_local[7], x_current_local[8],
          x_current_local[9]);
    }

    // Try to recover
    solver_->reset();

    std::vector<std::vector<double>> x_recovery_traj;
    std::vector<std::vector<double>> u_recovery_traj;

    for (int i = 0; i <= n_; ++i) {
      x_recovery_traj.push_back(x_current_local);
    }

    std::vector<double> u_hover(solver_->getNu(), 0.0);
    u_hover[0] = mass_ * 9.81;
    for (int i = 0; i < n_; ++i) {
      u_recovery_traj.push_back(u_hover);
    }

    solver_->warmStart(x_recovery_traj, u_recovery_traj);
    solver_->setInitialState(x_current_local);

    status = solver_->solve();
    if (status != 0) {
      RCLCPP_ERROR(this->get_logger(), "Recovery failed");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Recovered");
  }

  // Get solution
  std::vector<double> u_current;
  solver_->getControl(0, u_current);

  // Log the data if enabled
  if (enable_logging_) {
    // Use the first reference point for current reference
    const auto &current_ref = horizon_references[0];

    // Log current state vs reference
    double timestamp_sec = this->get_clock()->now().seconds();
    trajectory_logger_->logState(timestamp_sec, x_current_local, current_ref,
                                 u_current);

    // Log MPC horizon predictions (every 10th call to reduce file size)
    static int log_horizon_counter = 0;
    if (log_horizon_counter++ % 1 == 0) {
      std::vector<std::vector<double>> horizon_states;
      std::vector<std::vector<double>> horizon_controls;

      // Collect predicted states and controls from MPC solution
      for (int i = 0; i <= n_; ++i) {
        std::vector<double> x(solver_->getNx());
        solver_->getState(i, x);
        horizon_states.push_back(x);
      }

      for (int i = 0; i < n_; ++i) {
        std::vector<double> u(solver_->getNu());
        solver_->getControl(i, u);
        horizon_controls.push_back(u);
      }

      // Use precomputed references for logging
      trajectory_logger_->logMPCHorizon(timestamp_sec, horizon_states,
                                        horizon_controls, horizon_references);
    }
  }

  // Store for next iteration
  last_control_input_ = u_current;

  // Track computation time
  auto end_time = std::chrono::high_resolution_clock::now();
  double comp_time_ms =
      std::chrono::duration<double, std::milli>(end_time - start_time).count();
  computation_times_ms_.push_back(comp_time_ms);

  // Publish command based on controller type
  if (controller_type_ == "RATE") {
    publishRateCommand(u_current[0], u_current[1], u_current[2], u_current[3]);
    if (verbose_) {
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 100,
          "State: pos[%.2f,%.2f,%.2f] quat[%.2f,%.2f,%.2f,%.2f] "
          "vel[%.2f,%.2f,%.2f] | "
          "Cmd: T=%.2f w=[%.3f,%.3f,%.3f]",
          current_state_[0], current_state_[1], current_state_[2],
          current_state_[6], current_state_[7], current_state_[8],
          current_state_[9], current_state_[3], current_state_[4],
          current_state_[5], u_current[0], u_current[1], u_current[2],
          u_current[3]);
    }
  } else { // TORQUE
    if (!indi_enabled_) {
      // Send thrust and torques directly to PX4
      publishTorqueCommand(u_current[0], u_current[1], u_current[2],
                           u_current[3]);
      if (verbose_) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 100,
            "State: pos[%.2f,%.2f,%.2f] quat[%.2f,%.2f,%.2f,%.2f] "
            "vel[%.2f,%.2f,%.2f] ω[%.2f,%.2f,%.2f] | "
            "Cmd: T=%.2f τ[%.3f,%.3f,%.3f]",
            current_state_[0], current_state_[1], current_state_[2],
            current_state_[6], current_state_[7], current_state_[8],
            current_state_[9], current_state_[3], current_state_[4],
            current_state_[5], current_state_[10], current_state_[11],
            current_state_[12], u_current[0], u_current[1], u_current[2],
            u_current[3]);
      }
    } else {
      // Pass to INDI for conversion to motor commands
      std::lock_guard<std::mutex> lock(mpc_output_mutex_);
      latest_mpc_thrust_ = u_current[0];
      latest_mpc_torques_ << u_current[1], u_current[2], u_current[3];
      // INDI loop will pick these up and convert to motor commands
    }
  }
  // Publish MPC trajectory for visualization
  if (mpc_call_count % 10 == 0) {
    publishMPCTrajectory();
  }

  mpc_call_count++;
}

std::vector<double>
MpcController::runIndiController(double mpc_thrust,
                                 const Eigen::Vector3d &mpc_torques) {

  rclcpp::Time now = this->get_clock()->now();
  double dt = (now - last_indi_run_time_).seconds();
  if (dt < TINY_DT) {
    RCLCPP_WARN_ONCE(this->get_logger(),
                     "INDI dt is too small, holding last command.");
    return last_valid_rotor_speed_cmd_;
  }
  last_indi_run_time_ = now;

  Eigen::Vector3d angular_velocity_meas;
  std::vector<double> rotor_speeds_meas(4);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    angular_velocity_meas = current_angular_velocity_;
    rotor_speeds_meas = current_rotor_speeds_;
  }

  // Filter measurements
  if (indi_filter_type_ == "butterworth") {
    double wc = 2.0 * M_PI * indi_cutoff_freq_;
    double omega_tan = tan(wc * dt / 2.0);
    double alpha = 1.0 + sqrt(2.0) * omega_tan + omega_tan * omega_tan;
    double b0 = (omega_tan * omega_tan) / alpha;
    double b1 = 2.0 * b0;
    double b2 = b0;
    double a1 = (2.0 * omega_tan * omega_tan - 2.0) / alpha;
    double a2 = (1.0 - sqrt(2.0) * omega_tan + omega_tan * omega_tan) / alpha;

    for (int i = 0; i < 3; ++i) {
      double x_n = angular_velocity_meas(i);
      double y_n = b0 * x_n + b1 * omega_butter_states_[i].x_prev +
                   b2 * omega_butter_states_[i].x_prev2 -
                   a1 * omega_butter_states_[i].y_prev -
                   a2 * omega_butter_states_[i].y_prev2;
      filtered_omega_(i) = y_n;
      omega_butter_states_[i].x_prev2 = omega_butter_states_[i].x_prev;
      omega_butter_states_[i].x_prev = x_n;
      omega_butter_states_[i].y_prev2 = omega_butter_states_[i].y_prev;
      omega_butter_states_[i].y_prev = y_n;
    }
    for (int i = 0; i < 4; ++i) {
      double x_n = rotor_speeds_meas[i];
      double y_n = b0 * x_n + b1 * rotor_butter_states_[i].x_prev +
                   b2 * rotor_butter_states_[i].x_prev2 -
                   a1 * rotor_butter_states_[i].y_prev -
                   a2 * rotor_butter_states_[i].y_prev2;
      filtered_rotor_speeds_[i] = y_n;
      rotor_butter_states_[i].x_prev2 = rotor_butter_states_[i].x_prev;
      rotor_butter_states_[i].x_prev = x_n;
      rotor_butter_states_[i].y_prev2 = rotor_butter_states_[i].y_prev;
      rotor_butter_states_[i].y_prev = y_n;
    }
  } else {
    double alpha_fo = 2 * M_PI * dt * indi_cutoff_freq_ /
                      (2 * M_PI * dt * indi_cutoff_freq_ + 1);
    filtered_omega_ =
        alpha_fo * angular_velocity_meas + (1 - alpha_fo) * filtered_omega_;
    for (size_t i = 0; i < 4; ++i) {
      filtered_rotor_speeds_[i] = alpha_fo * rotor_speeds_meas[i] +
                                  (1 - alpha_fo) * filtered_rotor_speeds_[i];
    }
  }

  // Estimate accelerations
  Eigen::Vector3d omega_dot_f = (filtered_omega_ - prev_filtered_omega_) / dt;
  prev_filtered_omega_ = filtered_omega_;

  std::vector<double> rotor_accel(4);
  for (size_t i = 0; i < 4; ++i) {
    rotor_accel[i] =
        (filtered_rotor_speeds_[i] - prev_filtered_rotor_speeds_[i]) / dt;
  }
  prev_filtered_rotor_speeds_ = filtered_rotor_speeds_;

  // Build G matrices
  Eigen::Matrix<double, 4, 4> G1;
  Eigen::Matrix<double, 3, 4> G2_bar;
  double s = sin(arm_angle_rad_);
  double c = cos(arm_angle_rad_);
  double cq_ct = torque_coeff_ / thrust_coeff_;

  G1 << 1, 1, 1, 1, -arm_length_ * s, arm_length_ * s, arm_length_ * s,
      -arm_length_ * s, -arm_length_ * c, arm_length_ * c, -arm_length_ * c,
      arm_length_ * c, -cq_ct, -cq_ct, cq_ct, cq_ct;

  G2_bar.setZero();
  G2_bar.row(2) << -rotor_inertia_, -rotor_inertia_, rotor_inertia_,
      rotor_inertia_;

  // Calculate measured torque
  Eigen::Vector4d filtered_omega_sq;
  for (int i = 0; i < 4; ++i)
    filtered_omega_sq(i) =
        filtered_rotor_speeds_[i] * filtered_rotor_speeds_[i];

  Eigen::Matrix<double, 3, 4> G1_bar = G1.bottomRows<3>();
  Eigen::Vector4d rotor_accel_vec =
      Eigen::Map<Eigen::Vector4d>(rotor_accel.data());

  // Calculate the two components of tau_f separately for debugging
  Eigen::Vector3d tau_thrust_term = G1_bar * thrust_coeff_ * filtered_omega_sq;
  Eigen::Vector3d tau_inertia_term = G2_bar * rotor_accel_vec;
  Eigen::Vector3d tau_f = tau_thrust_term + tau_inertia_term;

  // Calculate desired angular acceleration
  Eigen::Matrix3d I_v = Eigen::Vector3d(Ixx_, Iyy_, Izz_).asDiagonal();
  Eigen::Vector3d gyro_effect =
      angular_velocity_meas.cross(I_v * angular_velocity_meas);
  Eigen::Vector3d desired_angular_accel =
      I_v.inverse() * (mpc_torques - gyro_effect);

  // Calculate final desired torque
  /* Eigen::Vector3d final_desired_torque = mpc_torques; */
  Eigen::Vector3d final_desired_torque =
      tau_f + I_v * (desired_angular_accel - omega_dot_f);

  // Check if full torque causes saturation
  Eigen::Vector4d command_vector_test;
  command_vector_test(0) = mpc_thrust;
  command_vector_test.tail<3>() = final_desired_torque;

  // Binary search to scale torques and avoid motor saturation
  double max_thrust_available = thrust_max_ / 4;
  Eigen::Matrix<double, 4, 4> G1_inv = G1.inverse();
  Eigen::Vector4d required_thrusts_test = G1_inv * command_vector_test;

  bool has_saturation = false;
  for (int i = 0; i < 4; ++i) {
    if (required_thrusts_test(i) < 0 ||
        required_thrusts_test(i) > max_thrust_available) {
      has_saturation = true;
      break;
    }
  }

  Eigen::Vector3d scaled_torque = final_desired_torque;

  if (has_saturation) {
    // Binary search for maximum feasible scaling factor
    double scale_min = 0.0;
    double scale_max = 1.0;
    double scale = 0.5;
    const double tolerance = 0.001;
    const int max_iterations = 20;

    for (int iter = 0; iter < max_iterations; ++iter) {
      scaled_torque = scale * final_desired_torque;
      command_vector_test.tail<3>() = scaled_torque;
      required_thrusts_test = G1_inv * command_vector_test;

      // Check if this scaling is feasible
      bool feasible = true;
      for (int i = 0; i < 4; ++i) {
        if (required_thrusts_test(i) < 0 ||
            required_thrusts_test(i) > max_thrust_available) {
          feasible = false;
          break;
        }
      }

      if (feasible) {
        scale_min = scale;
      } else {
        scale_max = scale;
      }

      if (scale_max - scale_min < tolerance) {
        break;
      }

      scale = (scale_min + scale_max) / 2.0;
    }

    // Use the feasible scaling factor
    scaled_torque = scale_min * final_desired_torque;

    // Log when scaling occurs
    if (verbose_) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Torque scaled to %.1f%% to avoid motor saturation. "
          "Desired: [%.3f, %.3f, %.3f] -> Scaled: [%.3f, %.3f, %.3f]",
          scale_min * 100.0, final_desired_torque(0), final_desired_torque(1),
          final_desired_torque(2), scaled_torque(0), scaled_torque(1),
          scaled_torque(2));
    }
  }

  // Solve for rotor speed commands using scaled torque
  Eigen::Vector4d command_vector;
  command_vector(0) = mpc_thrust;
  command_vector.tail<3>() = scaled_torque;

  // if we are publishing direct torque commands, then publish the desired
  // thrust and torque directly here and no longer continue with the loop
  if (use_direct_torque_) {
    return std::vector<double>{command_vector(0), command_vector(1),
                               command_vector(2), command_vector(3)};
  }

  Eigen::Vector4d required_thrusts = G1_inv * command_vector;

  std::vector<double> rotor_speed_cmds(4);
  for (int i = 0; i < 4; ++i) {
    double cmd = 0.0;
    if (required_thrusts(i) > 0) {
      cmd = sqrt(required_thrusts(i) / thrust_coeff_);
    }
    rotor_speed_cmds[i] = std::clamp(cmd, 0.0, max_rotor_speed_rad_s_);
  }

  if (verbose_) {
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 10,
        "\n\n\n================== INDI Debug ======================\n"
        "State: pos=[%.3f, %.3f, %.3f] vel=[%.3f, %.3f, %.3f]\n"
        "       quat=[%.3f, %.3f, %.3f, %.3f] ω=[%.3f, %.3f, %.3f]\n"
        "MPC Commands: T=%.2f τ_des=[%.3f, %.3f, %.3f]\n"
        /* "\n--- Measurements & Filtering ---\n" */
        /* "Angular Vel: ω_meas=[%.3f, %.3f, %.3f] ω_filt=[%.3f, %.3f, %.3f]\n"
         */
        /* "Angular Acc: ω̇_filt=[%.3f, %.3f, %.3f] ω̇_des=[%.3f, %.3f, %.3f]\n"
         */
        /* "Rotor Speeds (rad/s): meas=[%.1f, %.1f, %.1f, %.1f] filt=[%.1f,
           %.1f,
           " */
        /* "%.1f, %.1f]\n" */
        /* "Rotor Speeds²: [%.1f, %.1f, %.1f, %.1f]\n" */
        /* "Rotor Accel: [%.1f, %.1f, %.1f, %.1f]\n" */
        /* "\n--- G Matrices ---\n" */
        /* "G1_bar (3x4):\n" */
        /* "  [%.3f, %.3f, %.3f, %.3f]\n" */
        /* "  [%.3f, %.3f, %.3f, %.3f]\n" */
        /* "  [%.3f, %.3f, %.3f, %.3f]\n" */
        /* "G2_bar (3x4):\n" */
        /* "  [%.3f, %.3f, %.3f, %.3f]\n" */
        /* "  [%.3f, %.3f, %.3f, %.3f]\n" */
        /* "  [%.3f, %.3f, %.3f, %.3f]\n" */
        /* "\n--- Torque Estimation Breakdown ---\n" */
        /* "Thrust Coefficient: %.6e\n" */
        /* "τ_thrust = G1_bar * (c_T * ω²) = [%.3f, %.3f, %.3f]\n" */
        /* "  Row 0: %.6e*(%.3f*%.1f + %.3f*%.1f + %.3f*%.1f + %.3f*%.1f) =
           %.3f\n" */
        /* "  Row 1: %.6e*(%.3f*%.1f + %.3f*%.1f + %.3f*%.1f + %.3f*%.1f) =
           %.3f\n" */
        /* "  Row 2: %.6e*(%.3f*%.1f + %.3f*%.1f + %.3f*%.1f + %.3f*%.1f) =
           %.3f\n" */
        /* "τ_inertia = G2_bar * ω̇ = [%.3f, %.3f, %.3f]\n" */
        /* "  Row 0: %.3f*%.1f + %.3f*%.1f + %.3f*%.1f + %.3f*%.1f = %.3f\n" */
        /* "  Row 1: %.3f*%.1f + %.3f*%.1f + %.3f*%.1f + %.3f*%.1f = %.3f\n" */
        /* "  Row 2: %.3f*%.1f + %.3f*%.1f + %.3f*%.1f + %.3f*%.1f = %.3f\n" */
        "τ_estimated = τ_thrust + τ_inertia = [%.3f, %.3f, %.3f]\n"
        "\n--- Final Torque Calculation ---\n"
        "Gyro Effect: ω × (I*ω) = [%.3f, %.3f, %.3f]\n"
        "I_v*ω̇_des: [%.4f, %.4f, %.4f] * [%.3f, %.3f, %.3f] = [%.3f, %.3f, "
        "%.3f]\n"
        "I_v*ω̇_filt: [%.4f, %.4f, %.4f] * [%.3f, %.3f, %.3f] = [%.3f, %.3f, "
        "%.3f]\n"
        "τ_final = τ_estimated + I_v*(ω̇_des - ω̇_filt) = [%.3f, %.3f, %.3f]\n"
        "\n--- Command Vector & Solution ---\n"
        "Command Vector: [%.2f, %.3f, %.3f, %.3f] (T, τx, τy, τz)\n"
        "Required Thrusts: [%.2f, %.2f, %.2f, %.2f]\n"
        "Rotor Cmds (rad/s): [%.1f, %.1f, %.1f, %.1f]",
        // State - position
        current_state_[0], current_state_[1], current_state_[2],
        // State - velocity
        current_state_[3], current_state_[4], current_state_[5],
        // State - quaternion
        current_state_[6], current_state_[7], current_state_[8],
        current_state_[9],
        // State - angular velocity
        current_state_[10], current_state_[11], current_state_[12],
        // MPC Commands
        mpc_thrust, mpc_torques(0), mpc_torques(1), mpc_torques(2),
        // Measurements & Filtering
        /* angular_velocity_meas(0), angular_velocity_meas(1), */
        /* angular_velocity_meas(2), filtered_omega_(0), filtered_omega_(1), */
        /* filtered_omega_(2), omega_dot_f(0), omega_dot_f(1), omega_dot_f(2),
         */
        /* desired_angular_accel(0), desired_angular_accel(1), */
        /* desired_angular_accel(2), rotor_speeds_meas[0], rotor_speeds_meas[1],
         */
        /* rotor_speeds_meas[2], rotor_speeds_meas[3],
         * filtered_rotor_speeds_[0],
         */
        /* filtered_rotor_speeds_[1], filtered_rotor_speeds_[2], */
        /* filtered_rotor_speeds_[3], filtered_omega_sq(0),
         * filtered_omega_sq(1),
         */
        /* filtered_omega_sq(2), filtered_omega_sq(3), rotor_accel[0], */
        /* rotor_accel[1], rotor_accel[2], rotor_accel[3], */
        /* // G matrices */
        /* G1_bar(0, 0), G1_bar(0, 1), G1_bar(0, 2), G1_bar(0, 3), G1_bar(1, 0),
         */
        /* G1_bar(1, 1), G1_bar(1, 2), G1_bar(1, 3), G1_bar(2, 0), G1_bar(2, 1),
         */
        /* G1_bar(2, 2), G1_bar(2, 3), G2_bar(0, 0), G2_bar(0, 1), G2_bar(0, 2),
         */
        /* G2_bar(0, 3), G2_bar(1, 0), G2_bar(1, 1), G2_bar(1, 2), G2_bar(1, 3),
         */
        /* G2_bar(2, 0), G2_bar(2, 1), G2_bar(2, 2), G2_bar(2, 3), */
        /* // Torque estimation - thrust term with thrust_coeff_ */
        /* thrust_coeff_, tau_thrust_term(0), tau_thrust_term(1),
           tau_thrust_term(2), */
        /* // Row 0 breakdown */
        /* thrust_coeff_, G1_bar(0, 0), filtered_omega_sq(0), G1_bar(0, 1), */
        /* filtered_omega_sq(1), G1_bar(0, 2), filtered_omega_sq(2), G1_bar(0,
         * 3),
         */
        /* filtered_omega_sq(3), tau_thrust_term(0), */
        /* // Row 1 breakdown */
        /* thrust_coeff_, G1_bar(1, 0), filtered_omega_sq(0), G1_bar(1, 1), */
        /* filtered_omega_sq(1), G1_bar(1, 2), filtered_omega_sq(2), G1_bar(1,
         * 3),
         */
        /* filtered_omega_sq(3), tau_thrust_term(1), */
        /* // Row 2 breakdown */
        /* thrust_coeff_, G1_bar(2, 0), filtered_omega_sq(0), G1_bar(2, 1), */
        /* filtered_omega_sq(1), G1_bar(2, 2), filtered_omega_sq(2), G1_bar(2,
         * 3),
         */
        /* filtered_omega_sq(3), tau_thrust_term(2), */
        /* // Torque estimation - inertia term */
        /* tau_inertia_term(0), tau_inertia_term(1), tau_inertia_term(2), */
        /* // Row 0 breakdown */
        /* G2_bar(0, 0), rotor_accel_vec(0), G2_bar(0, 1), rotor_accel_vec(1),
         */
        /* G2_bar(0, 2), rotor_accel_vec(2), G2_bar(0, 3), rotor_accel_vec(3),
         */
        /* tau_inertia_term(0), */
        /* // Row 1 breakdown */
        /* G2_bar(1, 0), rotor_accel_vec(0), G2_bar(1, 1), rotor_accel_vec(1),
         */
        /* G2_bar(1, 2), rotor_accel_vec(2), G2_bar(1, 3), rotor_accel_vec(3),
         */
        /* tau_inertia_term(1), */
        /* // Row 2 breakdown */
        /* G2_bar(2, 0), rotor_accel_vec(0), G2_bar(2, 1), rotor_accel_vec(1),
         */
        /* G2_bar(2, 2), rotor_accel_vec(2), G2_bar(2, 3), rotor_accel_vec(3),
         */
        /* tau_inertia_term(2), */
        // Total estimated torque
        tau_f(0), tau_f(1), tau_f(2),
        // Final torque calculation
        gyro_effect(0), gyro_effect(1), gyro_effect(2), Ixx_, Iyy_, Izz_,
        desired_angular_accel(0), desired_angular_accel(1),
        desired_angular_accel(2), Ixx_ * desired_angular_accel(0),
        Iyy_ * desired_angular_accel(1), Izz_ * desired_angular_accel(2), Ixx_,
        Iyy_, Izz_, omega_dot_f(0), omega_dot_f(1), omega_dot_f(2),
        Ixx_ * omega_dot_f(0), Iyy_ * omega_dot_f(1), Izz_ * omega_dot_f(2),
        final_desired_torque(0), final_desired_torque(1),
        final_desired_torque(2),
        // Command vector & solution
        command_vector(0), command_vector(1), command_vector(2),
        command_vector(3), required_thrusts(0), required_thrusts(1),
        required_thrusts(2), required_thrusts(3), rotor_speed_cmds[0],
        rotor_speed_cmds[1], rotor_speed_cmds[2], rotor_speed_cmds[3]);
  }

  last_valid_rotor_speed_cmd_ = rotor_speed_cmds;
  return rotor_speed_cmds;
}

void MpcController::predictState(std::vector<double> &state, double dt) {
  if (state.size() < 10)
    return;

  Eigen::Vector3d p(state[0], state[1], state[2]);
  Eigen::Vector3d v(state[3], state[4], state[5]);
  Eigen::Quaterniond q(state[6], state[7], state[8], state[9]);

  double T = last_control_input_[0];
  p += v * dt;

  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Vector3d thrust_world = R * Eigen::Vector3d(0, 0, T);

  Eigen::Vector3d v_body = R.transpose() * v;
  double drag_force_body_x = -kx_ * v_body.x();
  double drag_force_body_y = -ky_ * v_body.y();
  double drag_force_body_z =
      -kz_ * v_body.z() +
      kh_ * (v_body.x() * v_body.x() + v_body.y() * v_body.y());
  Eigen::Vector3d drag_force_body(drag_force_body_x, drag_force_body_y,
                                  drag_force_body_z);
  Eigen::Vector3d drag_force_world = R * drag_force_body;

  Eigen::Vector3d g(0, 0, -9.81);
  Eigen::Vector3d a = g + (1.0 / mass_) * (thrust_world + drag_force_world);
  v += a * dt;

  if (controller_type_ == "RATE") {
    Eigen::Quaterniond omega_q(0, last_control_input_[1],
                               last_control_input_[2], last_control_input_[3]);
    q.coeffs() += (q * omega_q).coeffs() * 0.5 * dt;
  } else if (state.size() >= 13) {
    Eigen::Vector3d omega(state[10], state[11], state[12]);
    Eigen::Vector3d tau(last_control_input_[1], last_control_input_[2],
                        last_control_input_[3]);
    Eigen::Vector3d I_diag(Ixx_, Iyy_, Izz_);

    Eigen::Vector3d omega_dot =
        I_diag.asDiagonal().inverse() *
        (tau - omega.cross(I_diag.asDiagonal() * omega));
    omega += omega_dot * dt;
    state[10] = omega.x();
    state[11] = omega.y();
    state[12] = omega.z();

    Eigen::Quaterniond omega_q(0, omega.x(), omega.y(), omega.z());
    q.coeffs() += (q * omega_q).coeffs() * 0.5 * dt;
  }
  q.normalize();

  state[0] = p.x();
  state[1] = p.y();
  state[2] = p.z();
  state[3] = v.x();
  state[4] = v.y();
  state[5] = v.z();
  state[6] = q.w();
  state[7] = q.x();
  state[8] = q.y();
  state[9] = q.z();
}

std::vector<double> MpcController::interpolateReference(double t) {
  if (ref_trajectory_.empty()) {
    // Return empty if no trajectory
    return std::vector<double>();
  }

  // Handle edge cases
  if (t <= 0) {
    return ref_trajectory_.front();
  }

  double max_t = (ref_trajectory_.size() - 1) * trajectory_dt_;
  if (t >= max_t) {
    return ref_trajectory_.back();
  }

  // Find the two points to interpolate between
  double float_idx = t / trajectory_dt_;
  size_t idx_lower = static_cast<size_t>(std::floor(float_idx));
  size_t idx_upper = idx_lower + 1;

  if (idx_upper >= ref_trajectory_.size()) {
    return ref_trajectory_.back();
  }

  // Interpolation factor
  double alpha = float_idx - idx_lower;

  const auto &p1 = ref_trajectory_[idx_lower];
  const auto &p2 = ref_trajectory_[idx_upper];

  std::vector<double> interpolated(p1.size());

  // Linear interpolation for position and velocity
  for (size_t i = 0; i < 6; ++i) {
    interpolated[i] = (1.0 - alpha) * p1[i] + alpha * p2[i];
  }

  // SLERP for quaternion (indices 6-9)
  Eigen::Quaterniond q1(p1[6], p1[7], p1[8], p1[9]);
  Eigen::Quaterniond q2(p2[6], p2[7], p2[8], p2[9]);
  Eigen::Quaterniond q_interp = q1.slerp(alpha, q2);

  interpolated[6] = q_interp.w();
  interpolated[7] = q_interp.x();
  interpolated[8] = q_interp.y();
  interpolated[9] = q_interp.z();

  // For torque controller, interpolate angular velocities if present
  if (p1.size() >= 13 && interpolated.size() >= 13) {
    for (size_t i = 10; i < 13; ++i) {
      interpolated[i] = (1.0 - alpha) * p1[i] + alpha * p2[i];
    }
  }

  return interpolated;
}

} // namespace mpc_controller_ros2
