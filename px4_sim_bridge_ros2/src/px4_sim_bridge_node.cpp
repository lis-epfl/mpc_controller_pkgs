/**
 * @file px4_sim_bridge_node.cpp
 * @brief Main entry point for the PX4 simulation bridge node
 */

#include "px4_sim_bridge_ros2/px4_sim_bridge.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char * argv[]) {
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Create node options
  rclcpp::NodeOptions options;
  
  // Create the bridge node
  auto node = std::make_shared<px4_sim_bridge_ros2::Px4SimBridge>(options);
  
  // Spin the node
  rclcpp::spin(node);
  
  // Cleanup
  rclcpp::shutdown();
  return 0;
}
