#ifndef MPC_CONTROLLER_ROS2_MPC_CONTROLLER_HPP_
#define MPC_CONTROLLER_ROS2_MPC_CONTROLLER_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// MPC solver interfaces
#include "mpc_controller_ros2/mpc_rate_solver.hpp"
#include "mpc_controller_ros2/mpc_solver_base.hpp"
#include "mpc_controller_ros2/mpc_torque_solver.hpp"
#include "mpc_controller_ros2/trajectory_logger.hpp"

// ROS2 Services
#include "std_srvs/srv/trigger.hpp"

// ROS and message types
#include "mpc_controller_ros2_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

// PX4 messages
#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/esc_status.hpp"
#include "px4_msgs/msg/sensor_gyro.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"
#include "px4_msgs/msg/vehicle_thrust_setpoint.hpp"
#include "px4_msgs/msg/vehicle_torque_setpoint.hpp"

namespace mpc_controller_ros2 {

// State for a 2nd order Butterworth filter
struct Butterworth2ndOrderState {
  double x_prev{0.0}, x_prev2{0.0}; // Previous inputs
  double y_prev{0.0}, y_prev2{0.0}; // Previous outputs
};

class MpcController : public rclcpp::Node {
public:
  explicit MpcController(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~MpcController();

  // Public method for printing statistics (callable from signal handler)
  void printTimingStatistics(const std::string &name,
                             const std::vector<double> &times_ms,
                             double expected_freq_hz);

private:
  // Node methods
  void declareAndLoadParams();
  void initializeSolver();
  void mpcControlLoop();
  void indiControlLoop();

  // Callbacks
  void odometryCallback(px4_msgs::msg::VehicleOdometry::UniquePtr msg);
  void
  trajectoryCallback(mpc_controller_ros2_msgs::msg::Trajectory::UniquePtr msg);
  void gyroCallback(px4_msgs::msg::SensorGyro::UniquePtr msg);
  void escStatusCallback(px4_msgs::msg::EscStatus::UniquePtr msg);
  void enableControllerCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // Service callbacks
  void enableControllerService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void disableControllerService(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Helper methods
  void publishRateCommand(double thrust, double wx, double wy, double wz);
  void publishTorqueCommand(double thrust, double tau_x, double tau_y,
                            double tau_z);
  void publishMotorCommand(const std::vector<double> &motor_commands);
  void predictState(std::vector<double> &predicted_state, double dt);
  std::vector<double> runIndiController(double mpc_thrust,
                                        const Eigen::Vector3d &mpc_torques);
  void publishMPCTrajectory();
  void updateCostMatrices();
  void updateControlBounds();

  // New: Interpolate reference trajectory
  std::vector<double> interpolateReference(double t);

  // ROS objects - updated for PX4 messages
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<mpc_controller_ros2_msgs::msg::Trajectory>::SharedPtr
      traj_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorGyro>::SharedPtr gyro_sub_;
  rclcpp::Subscription<px4_msgs::msg::EscStatus>::SharedPtr esc_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;

  // Publishers for PX4 commands
  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr rates_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr
      torque_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr
      thrust_pub_;
  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr motors_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;

  // Service servers
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_service_;

  // timers
  rclcpp::TimerBase::SharedPtr mpc_timer_;
  rclcpp::TimerBase::SharedPtr indi_timer_;

  // MPC Solver - only one will be created based on controller_type_
  std::unique_ptr<MPCSolverBase> solver_;

  // Parameters
  std::string controller_type_, indi_filter_type_;
  bool use_direct_torque_; // If true, send torque/thrust directly (no INDI)
  bool controller_enabled_;
  bool indi_enabled_;
  bool verbose_;
  int n_;
  double control_freq_, indi_freq_;
  double time_step_;
  double mpc_computation_delay_;
  double horizon_decay_factor_;
  std::string horizon_decay_type_;
  std::string traj_topic_;
  double mass_, Ixx_, Iyy_, Izz_;
  double kx_, ky_, kz_, kh_;
  double arm_length_, arm_angle_rad_, rotor_inertia_, thrust_coeff_,
      torque_coeff_;
  double indi_cutoff_freq_;
  double thrust_min_, thrust_max_, max_rotor_rpm_, max_rotor_speed_rad_s_;
  double esc_scale_;
  double rate_max_xy_, rate_max_z_, torque_max_xy_, torque_max_z_;
  std::vector<double> Q_rate_diag_, R_rate_diag_;
  std::vector<double> Q_torque_diag_, R_torque_diag_;

  // Thread-safe state (shared between ROS callbacks, MPC, and INDI loops)
  std::mutex state_mutex_;
  std::mutex trajectory_mutex_;
  std::mutex x_init_mutex_;
  std::mutex controller_enabled_mutex_;
  bool pose_received_ = false;
  bool trajectory_received_ = false;
  std::vector<double> current_state_;
  Eigen::Vector3d current_angular_velocity_{0., 0., 0.};
  std::vector<double> current_rotor_speeds_{0., 0., 0., 0.};
  rclcpp::Time last_odometry_timestamp_;

  // Thread-safe data exchange from MPC to INDI (only used in TORQUE mode)
  std::mutex mpc_output_mutex_;
  double latest_mpc_thrust_ = 0.0;
  Eigen::Vector3d latest_mpc_torques_{0.0, 0.0, 0.0};

  // last measusrement variables to replace sensor drop outs that give 0.0
  Eigen::Vector3d last_good_angular_velocity_{0.0, 0.0, 0.0};
  std::vector<double> last_good_rotor_speeds_{0.0, 0.0, 0.0, 0.0};

  // Controller state
  std::vector<double> predicted_state_cache_;
  std::vector<double> last_control_input_;
  std::vector<double> x_init_;
  std::vector<std::vector<double>> ref_trajectory_;
  rclcpp::Time trajectory_start_time_;
  double trajectory_dt_ = 0.1;
  unsigned int solver_failure_count_ = 0;

  // INDI Controller State (only used in TORQUE mode with INDI)
  Eigen::Vector3d filtered_omega_{0., 0., 0.};
  Eigen::Vector3d prev_filtered_omega_{0., 0., 0.};
  std::vector<double> filtered_rotor_speeds_{0., 0., 0., 0.};
  std::vector<double> prev_filtered_rotor_speeds_{0., 0., 0., 0.};
  std::vector<double> last_valid_rotor_speed_cmd_{0., 0., 0., 0.};
  rclcpp::Time last_indi_run_time_;

  // States for Butterworth filters
  std::vector<Butterworth2ndOrderState> omega_butter_states_;
  std::vector<Butterworth2ndOrderState> rotor_butter_states_;

  // Performance tracking
  std::vector<double> computation_times_ms_;
  std::vector<double> indi_computation_times_ms_;
  std::mutex indi_stats_mutex_;

  // Logging
  std::unique_ptr<TrajectoryLogger> trajectory_logger_;
  bool enable_logging_ = false;
  std::string log_directory_;
};

} // namespace mpc_controller_ros2

#endif // MPC_CONTROLLER_ROS2_MPC_CONTROLLER_HPP_
