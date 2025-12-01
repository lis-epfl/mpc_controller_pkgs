#ifndef MPC_CONTROLLER_ROS2_TRAJECTORY_LOGGER_HPP_
#define MPC_CONTROLLER_ROS2_TRAJECTORY_LOGGER_HPP_

#include <chrono>
#include <fstream>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace mpc_controller_ros2 {

class TrajectoryLogger {
public:
  TrajectoryLogger(const std::string &log_directory = "/tmp/mpc_logs");
  ~TrajectoryLogger();

  void startNewLog();
  void stopLogging();

  void logState(double timestamp, const std::vector<double> &actual_state,
                const std::vector<double> &reference_state,
                const std::vector<double> &control_input);

  void logMPCHorizon(double timestamp,
                     const std::vector<std::vector<double>> &predicted_states,
                     const std::vector<std::vector<double>> &predicted_controls,
                     const std::vector<std::vector<double>> &reference_states);

  void logComputationTime(double timestamp, double computation_time_ms);

  std::string getCurrentLogPath() const;

private:
  void createLogDirectory();
  void updateLatestLink();

  std::string log_directory_;
  std::string current_log_path_;
  std::string latest_path_;

  std::ofstream state_file_;
  std::ofstream horizon_file_;
  std::ofstream computation_time_file_;

  std::mutex mutex_;
  bool is_logging_;

  std::string generateTimestamp();
};

} // namespace mpc_controller_ros2

#endif // MPC_CONTROLLER_ROS2_TRAJECTORY_LOGGER_HPP_
