#include "mpc_controller_ros2/trajectory_logger.hpp"
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace mpc_controller_ros2 {

TrajectoryLogger::TrajectoryLogger(const std::string &log_directory)
    : log_directory_(log_directory), is_logging_(false) {
  createLogDirectory();
  latest_path_ = log_directory_ + "/latest";
}

TrajectoryLogger::~TrajectoryLogger() { stopLogging(); }

void TrajectoryLogger::createLogDirectory() {
  std::filesystem::create_directories(log_directory_);
}

std::string TrajectoryLogger::generateTimestamp() {
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
  return ss.str();
}

std::string TrajectoryLogger::getCurrentLogPath() const {
  return current_log_path_;
}

void TrajectoryLogger::updateLatestLink() {
  try {
    // Remove existing "latest" symlink if it exists
    if (std::filesystem::exists(latest_path_) || std::filesystem::is_symlink(latest_path_)) {
      std::filesystem::remove(latest_path_);
    }

    // Create symbolic link
    std::filesystem::create_directory_symlink(current_log_path_, latest_path_);
  } catch (const std::exception& e) {
    // Symlink failed - log but don't crash
    // Could add RCLCPP_WARN here if you have access to a logger
  }
}

void TrajectoryLogger::startNewLog() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (is_logging_) {
    stopLogging();
  }

  std::string timestamp = generateTimestamp();
  current_log_path_ = log_directory_ + "/" + timestamp;
  std::filesystem::create_directories(current_log_path_);

  // Open state log file with header - UPDATED to include angular velocities
  state_file_.open(current_log_path_ + "/states.csv");
  state_file_ << "timestamp,";
  state_file_ << "x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz,";  // Added wx,wy,wz
  state_file_ << "ref_x,ref_y,ref_z,ref_vx,ref_vy,ref_vz,ref_qw,ref_qx,ref_qy,ref_qz,ref_wx,ref_wy,ref_wz,";  // Added ref_wx,ref_wy,ref_wz
  state_file_ << "error_x,error_y,error_z,error_norm,";
  state_file_ << "thrust,u1,u2,u3\n";

  // Open horizon log file - UPDATED to include angular velocities
  horizon_file_.open(current_log_path_ + "/mpc_horizon.csv");
  horizon_file_ << "timestamp,horizon_idx,";
  horizon_file_ << "x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz,";  // Added wx,wy,wz
  horizon_file_ << "thrust,u1,u2,u3,";
  horizon_file_ << "ref_x,ref_y,ref_z,ref_vx,ref_vy,ref_vz,ref_qw,ref_qx,ref_qy,ref_qz,ref_wx,ref_wy,ref_wz\n";  // Added ref_wx,ref_wy,ref_wz

  // Open computation time log file
  computation_time_file_.open(current_log_path_ + "/computation_times.csv");
  computation_time_file_ << "timestamp,computation_time_ms\n";

  is_logging_ = true;

  // Create/update the "latest" symlink
  updateLatestLink();
}

void TrajectoryLogger::stopLogging() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (state_file_.is_open()) {
    state_file_.close();
  }
  if (horizon_file_.is_open()) {
    horizon_file_.close();
  }
  if (computation_time_file_.is_open()) {
    computation_time_file_.close();
  }

  is_logging_ = false;
}

void TrajectoryLogger::logState(double timestamp,
                                const std::vector<double> &actual_state,
                                const std::vector<double> &reference_state,
                                const std::vector<double> &control_input) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_logging_ || !state_file_.is_open()) {
    return;
  }

  // Write timestamp
  state_file_ << std::fixed << std::setprecision(6) << timestamp << ",";

  // Write actual state (first 10 elements: position, velocity, quaternion)
  for (size_t i = 0; i < std::min(size_t(10), actual_state.size()); ++i) {
    state_file_ << actual_state[i] << ",";
  }

  // Write angular velocities if available (for torque model)
  if (actual_state.size() >= 13) {
    state_file_ << actual_state[10] << "," << actual_state[11] << "," << actual_state[12] << ",";
  } else {
    // Write zeros for angular velocities if not available (rate model)
    state_file_ << "0,0,0,";
  }

  // Write reference state (first 10 elements)
  for (size_t i = 0; i < std::min(size_t(10), reference_state.size()); ++i) {
    state_file_ << reference_state[i] << ",";
  }

  // Write reference angular velocities if available
  if (reference_state.size() >= 13) {
    state_file_ << reference_state[10] << "," << reference_state[11] << "," << reference_state[12] << ",";
  } else {
    // Write zeros for reference angular velocities if not available
    state_file_ << "0,0,0,";
  }

  // Calculate and write errors
  double error_x = actual_state[0] - reference_state[0];
  double error_y = actual_state[1] - reference_state[1];
  double error_z = actual_state[2] - reference_state[2];
  double error_norm = std::sqrt(error_x * error_x + error_y * error_y + error_z * error_z);

  state_file_ << error_x << "," << error_y << "," << error_z << "," << error_norm << ",";

  // Write control input
  for (size_t i = 0; i < control_input.size(); ++i) {
    state_file_ << control_input[i];
    if (i < control_input.size() - 1) {
      state_file_ << ",";
    }
  }
  state_file_ << "\n";
  state_file_.flush();
}

void TrajectoryLogger::logMPCHorizon(
    double timestamp, const std::vector<std::vector<double>> &predicted_states,
    const std::vector<std::vector<double>> &predicted_controls,
    const std::vector<std::vector<double>> &reference_states) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_logging_ || !horizon_file_.is_open()) {
    return;
  }

  for (size_t i = 0; i < predicted_states.size(); ++i) {
    horizon_file_ << std::fixed << std::setprecision(6) << timestamp << ",";
    horizon_file_ << i << ",";

    // Write predicted state (first 10 elements)
    for (size_t j = 0; j < std::min(size_t(10), predicted_states[i].size()); ++j) {
      horizon_file_ << predicted_states[i][j] << ",";
    }

    // Write predicted angular velocities if available
    if (predicted_states[i].size() >= 13) {
      horizon_file_ << predicted_states[i][10] << "," << predicted_states[i][11] << "," << predicted_states[i][12] << ",";
    } else {
      // Write zeros for angular velocities if not available
      horizon_file_ << "0,0,0,";
    }

    // Write predicted control (if available)
    if (i < predicted_controls.size()) {
      for (size_t j = 0; j < predicted_controls[i].size(); ++j) {
        horizon_file_ << predicted_controls[i][j] << ",";
      }
    } else {
      horizon_file_ << "0,0,0,0,";
    }

    // Write reference state (if available)
    if (i < reference_states.size()) {
      // First 10 states
      for (size_t j = 0; j < std::min(size_t(10), reference_states[i].size()); ++j) {
        horizon_file_ << reference_states[i][j] << ",";
      }

      // Angular velocities if available
      if (reference_states[i].size() >= 13) {
        horizon_file_ << reference_states[i][10] << "," << reference_states[i][11] << "," << reference_states[i][12];
      } else {
        horizon_file_ << "0,0,0";
      }
    } else {
      // No reference available, write zeros
      for (size_t j = 0; j < 13; ++j) {
        horizon_file_ << "0";
        if (j < 12) {
          horizon_file_ << ",";
        }
      }
    }

    horizon_file_ << "\n";
  }
  horizon_file_.flush();
}

void TrajectoryLogger::logComputationTime(double timestamp, double computation_time_ms) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_logging_ || !computation_time_file_.is_open()) {
    return;
  }

  computation_time_file_ << std::fixed << std::setprecision(6) << timestamp << ","
                         << computation_time_ms << "\n";
  computation_time_file_.flush();
}

} // namespace mpc_controller_ros2
