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
- Docker Compose (version 2.0 or higher)
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
   cd ~/mpc_controller_pkgs
   colcon build --symlink-install
   source install/setup.bash
   ```

## Usage

The MPC and INDI parameters can be configured in `config/mpc_config.yaml`.
For options that can be passed to the trajectory publisher and the plotter scripts, check the arguments inside them. The default trajectory publisher with no arguments will publish a helix.

### Docker Usage with Simulation

Follow these steps to run the MPC controller with simulation in Docker:

1. **Generate ACADOS solvers:**
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

5. **Source the workspace:**
   Inside the container:
   ```bash
   source install/setup.bash
   ```

6. **Run the trajectory publisher:**
   ```bash
   cd scripts
   python3 trajectory_publisher.py 
   ```

7. **Visualize results:**
   After the trajectory is completed:
   ```bash
   python3 plot_trajectory_data.py 
   ```

### Native Usage

For native installation:

1. **Source ROS2 and workspace:**
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/mpc_controller_pkgs/install/setup.bash
   ```

2. **Generate ACADOS solvers:**
   ```bash
   cd ~/mpc_controller_pkgs
   python3 scripts/generate_acados_solvers.py
   ```

3. **Launch the simulation:**
   ```bash
   ros2 launch mpc_controller simulation.launch.py
   ```

4. **Launch the MPC controller:**
   In a new terminal:
   ```bash
   ros2 launch mpc_controller mpc_controller.launch.py
   ```

5. **Publish trajectories**
   ```bash
   cd ~/mpc_controller_pkgs/scripts
   python3 trajectory_publisher.py
   ```

6. **Visualize trajectories**
   ```bash
   python3 plot_trajectory_data.py
   ```
