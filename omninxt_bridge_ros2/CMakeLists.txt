cmake_minimum_required(VERSION 3.8)
project(omninxt_bridge_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(multi_agent_planner_msgs REQUIRED)
find_package(mpc_controller_ros2_msgs REQUIRED)
find_package(swarmnxt_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# Trajectory Converter Node
add_executable(trajectory_converter_node
  src/trajectory_converter_node.cpp
)

target_include_directories(trajectory_converter_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(trajectory_converter_node
  rclcpp
  multi_agent_planner_msgs
  mpc_controller_ros2_msgs
  geometry_msgs
)

# Command Converter Node
add_executable(command_converter_node
  src/command_converter_node.cpp
)

target_include_directories(command_converter_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(command_converter_node
  rclcpp
  px4_msgs
  swarmnxt_msgs
  mavros_msgs
  std_msgs
  geometry_msgs
)

# Install executables
install(TARGETS
  trajectory_converter_node
  command_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
