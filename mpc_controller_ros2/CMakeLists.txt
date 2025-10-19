cmake_minimum_required(VERSION 3.8)
project(mpc_controller_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_compile_options(-g) # compile options for debugging
add_compile_options(-O3) # compile options for max optimization

# Check if solvers have been generated
execute_process(
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_acados_solvers.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts
  RESULT_VARIABLE return_code
)

if(NOT return_code EQUAL 0)
  message(FATAL_ERROR "Failed to generate acados solvers. See output above for details.")
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mpc_controller_ros2_msgs REQUIRED)
find_package(px4_msgs REQUIRED)  # Only PX4 messages needed now
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)

# Set ACADOS paths directly based on diagnostic output
set(ACADOS_SOURCE_DIR "/opt/acados")
set(acados_INCLUDE_DIRS
    "${ACADOS_SOURCE_DIR}/include"
    "${ACADOS_SOURCE_DIR}/include/blasfeo/include"
    "${ACADOS_SOURCE_DIR}/include/hpipm/include"
)

# Try to find acados libraries
set(acados_LIBRARY_DIR "${ACADOS_SOURCE_DIR}/lib")
if(NOT EXISTS "${acados_LIBRARY_DIR}/libacados.so" AND NOT EXISTS "${acados_LIBRARY_DIR}/libacados.a")
    set(acados_LIBRARY_DIR "${ACADOS_SOURCE_DIR}/build/lib")
endif()

find_library(acados_LIB acados PATHS ${acados_LIBRARY_DIR} NO_DEFAULT_PATH)
find_library(blasfeo_LIB blasfeo PATHS ${acados_LIBRARY_DIR} NO_DEFAULT_PATH)
find_library(hpipm_LIB hpipm PATHS ${acados_LIBRARY_DIR} NO_DEFAULT_PATH)
find_library(qpOASES_LIB qpOASES_e PATHS ${acados_LIBRARY_DIR} NO_DEFAULT_PATH)

if(NOT acados_LIB)
    message(FATAL_ERROR "Cannot find acados library.
Please build acados in ${ACADOS_SOURCE_DIR}")
endif()

set(acados_LIBRARIES
    ${acados_LIB}
    ${blasfeo_LIB}
    ${hpipm_LIB}
    ${qpOASES_LIB}
    m
)

# Set RPATH
set(CMAKE_INSTALL_RPATH "${acados_LIBRARY_DIR}")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# Debug output
message(STATUS "ACADOS configuration:")
message(STATUS "  Source dir: ${ACADOS_SOURCE_DIR}")
message(STATUS "  Include dirs: ${acados_INCLUDE_DIRS}")
message(STATUS "  Library dir: ${acados_LIBRARY_DIR}")
message(STATUS "  Libraries found: ${acados_LIBRARIES}")

# Build the generated Acados Solvers as static libraries
set(C_CODE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code)
set(ACADOS_SOLVER_RATE_DIR ${C_CODE_DIR}/quadrotor_rate)
set(ACADOS_SOLVER_TORQUE_DIR ${C_CODE_DIR}/quadrotor_torque)


message(STATUS "Found generated acados solvers:")
message(STATUS "  Rate solver: ${ACADOS_SOLVER_RATE_DIR}")
message(STATUS "  Torque solver: ${ACADOS_SOLVER_TORQUE_DIR}")

# Create static library for RATE solver
add_library(acados_solver_quadrotor_rate STATIC
    ${ACADOS_SOLVER_RATE_DIR}/acados_solver_quadrotor_rate.c
    ${ACADOS_SOLVER_RATE_DIR}/acados_sim_solver_quadrotor_rate.c
    ${ACADOS_SOLVER_RATE_DIR}/quadrotor_rate_model/quadrotor_rate_expl_ode_fun.c
    ${ACADOS_SOLVER_RATE_DIR}/quadrotor_rate_model/quadrotor_rate_expl_vde_forw.c
    ${ACADOS_SOLVER_RATE_DIR}/quadrotor_rate_model/quadrotor_rate_expl_vde_adj.c
    ${ACADOS_SOLVER_RATE_DIR}/quadrotor_rate_constraints/quadrotor_rate_constr_h_fun.c
    ${ACADOS_SOLVER_RATE_DIR}/quadrotor_rate_constraints/quadrotor_rate_constr_h_fun_jac_uxt_zt.c
)

target_include_directories(acados_solver_quadrotor_rate PUBLIC
    $<BUILD_INTERFACE:${ACADOS_SOLVER_RATE_DIR}>
    $<INSTALL_INTERFACE:include>
    ${acados_INCLUDE_DIRS}
)

target_link_libraries(acados_solver_quadrotor_rate
    ${acados_LIBRARIES}
)

set_property(TARGET acados_solver_quadrotor_rate PROPERTY POSITION_INDEPENDENT_CODE ON)

# Create static library for TORQUE solver
add_library(acados_solver_quadrotor_torque STATIC
    ${ACADOS_SOLVER_TORQUE_DIR}/acados_solver_quadrotor_torque.c
    ${ACADOS_SOLVER_TORQUE_DIR}/acados_sim_solver_quadrotor_torque.c
    ${ACADOS_SOLVER_TORQUE_DIR}/quadrotor_torque_model/quadrotor_torque_expl_ode_fun.c
    ${ACADOS_SOLVER_TORQUE_DIR}/quadrotor_torque_model/quadrotor_torque_expl_vde_forw.c
    ${ACADOS_SOLVER_TORQUE_DIR}/quadrotor_torque_model/quadrotor_torque_expl_vde_adj.c
    ${ACADOS_SOLVER_TORQUE_DIR}/quadrotor_torque_constraints/quadrotor_torque_constr_h_fun.c
    ${ACADOS_SOLVER_TORQUE_DIR}/quadrotor_torque_constraints/quadrotor_torque_constr_h_fun_jac_uxt_zt.c
)

target_include_directories(acados_solver_quadrotor_torque PUBLIC
    $<BUILD_INTERFACE:${ACADOS_SOLVER_TORQUE_DIR}>
    $<INSTALL_INTERFACE:include>
    ${acados_INCLUDE_DIRS}
)

target_link_libraries(acados_solver_quadrotor_torque
    ${acados_LIBRARIES}
)

set_property(TARGET acados_solver_quadrotor_torque PROPERTY POSITION_INDEPENDENT_CODE ON)

# Create the MPC solver library
add_library(mpc_solvers STATIC
  src/mpc_rate_solver.cpp
  src/mpc_torque_solver.cpp
  src/trajectory_logger.cpp
)

target_include_directories(mpc_solvers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ACADOS_SOLVER_RATE_DIR}
  ${ACADOS_SOLVER_TORQUE_DIR}
  ${acados_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Use ament_target_dependencies first
ament_target_dependencies(mpc_solvers
  rclcpp
)

# Then link other libraries
target_link_libraries(mpc_solvers
  acados_solver_quadrotor_rate
  acados_solver_quadrotor_torque
  ${acados_LIBRARIES}
  Eigen3::Eigen
)

# Build the main controller node
add_executable(mpc_controller_node
  src/mpc_controller_node.cpp
  src/mpc_controller.cpp
)

target_include_directories(mpc_controller_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# First add ament dependencies
ament_target_dependencies(mpc_controller_node
  rclcpp
  geometry_msgs
  sensor_msgs
  mpc_controller_ros2_msgs
  px4_msgs
  nav_msgs
)

# Then link against libraries
target_link_libraries(mpc_controller_node
  mpc_solvers
  Eigen3::Eigen
)

# Install rules
install(
  TARGETS mpc_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)

# Make scripts executable
install(PROGRAMS
  scripts/trajectory_publisher.py
  scripts/plot_trajectory_data.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  mpc_controller_ros2_msgs
  px4_msgs
  nav_msgs
  Eigen3
)

ament_package()
