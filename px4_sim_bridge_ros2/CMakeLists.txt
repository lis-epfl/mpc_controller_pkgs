cmake_minimum_required(VERSION 3.8)
project(px4_sim_bridge_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Build the bridge node
add_executable(px4_sim_bridge_node
  src/px4_sim_bridge_node.cpp
  src/px4_sim_bridge.cpp
)

target_include_directories(px4_sim_bridge_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(px4_sim_bridge_node
  rclcpp
  px4_msgs
  std_srvs
)

# Install executables
install(
  TARGETS px4_sim_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
