/**
 * @file mpc_controller_node.cpp
 * @brief Main entry point for the MPC controller ROS2 node
 */

#include "mpc_controller_ros2/mpc_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char **argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Create node options
  rclcpp::NodeOptions options;
  
  // Create the MPC controller node
  auto node = std::make_shared<mpc_controller_ros2::MpcController>(options);
  
  // Use a MultiThreadedExecutor to allow MPC and INDI loops to run in parallel
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  // Spin the executor
  executor.spin();
  
  // Cleanup
  rclcpp::shutdown();
  return 0;
}
