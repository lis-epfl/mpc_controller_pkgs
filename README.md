# MPC Controller Packages

A ROS2-based Model Predictive Control (MPC) controller implementation with Incremental Nonlinear Dynamic Inversion (INDI) for quadrotor trajectory tracking.

## Table of Contents
- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [Docker Installation (Recommended)](#docker-installation-recommended)
  - [Native Installation](#native-installation)
- [Usage](#usage)
  - [Docker Usage with Simulation](#docker-usage-with-simulation)
  - [Native Usage](#native-usage)

## Overview

This package provides a complete MPC controller solution with INDI implementation for ROS2, featuring:
- Nonlinear Model Predictive Control (NMPC) using ACADOS solvers
- Incremental Nonlinear Dynamic Inversion (INDI) for enhanced control
- Comprehensive simulation environment
- Trajectory planning and tracking capabilities
- Performance visualization and analysis tools

## Prerequisites

### For Docker Installation
- Docker Engine (version 20.10 or higher)
- At least 8GB of available RAM
- 20GB of free disk space

### For Native Installation
- Ubuntu 22.04 LTS (Jammy Jellyfish)
- ROS2 Humble Hawksbill
- Python 3.10+
- CMake 3.16+
- GCC 9.0+ or Clang 10.0+

## Installation

### Docker Installation (Recommended)

The Docker installation provides a containerized environment with all dependencies pre-configured.

1. **Clone the repository:**
   ```bash
   git clone https://github.com/lis-epfl/mpc_controller_pkgs.git
   cd mpc_controller_pkgs
   ```

2. **Build the Docker images:**
   ```bash
   # Build the simulation environment
   ./docker/build_simulation.sh
   
   # Build the MPC controller
   ./docker/build_mpc.sh
   ```

3. **Verify the installation:**
   ```bash
   docker images | grep ros2_mpc
   ```

### Native Installation

For native installation, follow these steps based on the Docker configuration:

1. **Install ROS2 Humble:**
   ```bash
   # Add ROS2 repository
   sudo apt update && sudo apt install curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   
   # Install ROS2 Humble
   sudo apt update
   sudo apt install ros-humble-desktop python3-rosdep2
   ```

2. **Install ACADOS:**
   ```bash
   # Clone ACADOS repository
   git clone https://github.com/acados/acados.git ~/acados
   cd ~/acados
   git submodule update --recursive --init
   
   # Build and install ACADOS
   mkdir -p build && cd build
   cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=ON ..
   make install -j4
   
   # Set environment variables
   echo "export ACADOS_SOURCE_DIR=$HOME/acados" >> ~/.bashrc
   echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/acados/lib" >> ~/.bashrc
   echo "export ACADOS_INSTALL_DIR=$HOME/acados" >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Install Python dependencies:**
   ```bash
   # Create virtual environment (optional but recommended)
   python3 -m venv mpc_env
   source mpc_env/bin/activate
   
   # Install required packages
   pip install numpy scipy matplotlib
   pip install casadi>=3.5.5
   pip install -e ~/acados/interfaces/acados_template
   ```

4. **Install additional ROS2 dependencies:**
   ```bash
   sudo apt install ros-humble-gazebo-ros-pkgs \
                    ros-humble-ros2-control \
                    ros-humble-ros2-controllers \
                    ros-humble-xacro \
                    ros-humble-joint-state-publisher-gui \
                    ros-humble-rviz2
   ```

5. **Build the workspace:**
   ```bash
   cd ~/ros2_ws/src/
   git clone https://github.com/lis-epfl/mpc_controller_pkgs.git
   cd mpc_controller_pkgs
   python3 scripts/generate_acados_solvers.py
   cd ../ 
   colcon build --symlink-install --packages-select px4_msgs mpc_controller_ros2_msgs mpc_controller_ros2 px4_sim_bridge_ros2
   source install/setup.bash
   ```

## Usage

The MPC and INDI parameters can be configured in `config/mpc_config.yaml`.
For options that can be passed to the trajectory publisher and the plotter scripts, check the arguments inside them. The default trajectory publisher with no arguments will publish a helix.

### Docker Usage with Simulation

Follow these steps to run the MPC controller with simulation in Docker:

1. **Generate ACADOS solvers:**
  Necessary after every change to `generate_acados_solvers.py`.
   ```bash
   # First, ensure you're in the project directory
   cd mpc_controller_pkgs
   
   # Run the solver generation script
   python3 scripts/generate_acados_solvers.py
   ```

2. **Start the simulation environment:**
   ```bash
   ./docker/run_sim.sh
   ```

3. **Launch the MPC controller:**
   In a new terminal:
   ```bash
   ./docker/launch_sim_mpc.sh
   ```

4. **Access the container:**
   ```bash
   docker exec -it ros2_mpc_container bash
   ```

5. **Run the trajectory publisher:**
   ```bash
   source install/setup.bash
   cd scripts
   python3 trajectory_publisher.py 
   ```

6. **Visualize results:**
   After the trajectory is completed (also could be done during the flight):
   ```bash
   python3 plot_trajectory_data.py 
   ```

To launch the controller alone without the simulation bridge (for the real drone), you can launch `/dockerfiles/launch_mpc.sh`.

### Native Usage

For native installation:

1. **Generate ACADOS solvers:**
  Necessary after every change to `generate_acados_solvers.py`.
   ```bash
   cd mpc_controller_pkgs
   python3 scripts/generate_acados_solvers.py
   cd ../ 
   colcon build --symlink-install --packages-select px4_msgs mpc_controller_ros2_msgs mpc_controller_ros2
   ```

2. **Launch the simulation from the dockerfile:**
   ```bash
   ./dockerfiles/build_simulation.sh
   ./dockerfiles/run_simulation.sh
   ```

3. **Launch the MPC controller with the drone state controller:**
   In a new terminal, in your ros2 workspace:
   ```bash
   source install/setup.bash
   ros2 launch px4_sim_bridge_ros2 sim_with_mpc.launch.py
   ```

5. **Publish trajectories**
In a new terminal, in your ros2 workspace:
   ```bash
   source install/setup.bash
   cd src/mpc_controller_pkgs/scripts
   python3 trajectory_publisher.py
   ```

6. **Visualize trajectories**
   ```bash
   python3 plot_trajectory_data.py
   ```
To launch the controller alone without the simulation bridge (for the real drone), you can launch `mpc.launch.py` from the `mpc_controller_ros2` package.
