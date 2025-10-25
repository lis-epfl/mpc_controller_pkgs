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

### Native Installation

For native installation, follow these steps:

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

2. **Setup Python virtual environment:**
   ```bash
   sudo apt install python3-venv
   python3 -m venv ~/mpc_venv
   ```

3. **Install ACADOS:**
   ```bash
   # Clone ACADOS repository
   cd /opt
   sudo git clone https://github.com/acados/acados.git
   cd acados
   sudo git checkout v0.5.1
   sudo git submodule update --init --recursive

   # Create build directory and configure
   sudo mkdir build && cd build
   sudo cmake -DACADOS_WITH_QPOASES=ON \
              -DACADOS_WITH_OSQP=ON \
              -DCMAKE_BUILD_TYPE=Release \
              ..
   sudo make -j$(nproc)
   sudo make install
   ```

4. **Set environment variables:**
   Add the following to your `~/.bashrc`:
   ```bash
   export ACADOS_SOURCE_DIR=/opt/acados
   export LD_LIBRARY_PATH=/opt/acados/lib:$LD_LIBRARY_PATH
   export PYTHONPATH=/opt/acados/interfaces/acados_template:$PYTHONPATH
   ```
   Then source it:
   ```bash
   source ~/.bashrc
   ```

5. **Install Python dependencies:**
   ```bash
   # Activate virtual environment
   source ~/mpc_venv/bin/activate

   # Install required packages
   pip install numpy==1.24.4 \
               scipy==1.8.0 \
               matplotlib==3.5.1 \
               pyyaml==5.3.1 \
               pandas==2.0.3
   ```

6. **Build and install t_renderer (required for ACADOS):**
   ```bash
   # Install Rust (needed to build t_renderer)
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
   source ~/.cargo/env

   # Clone and build t_renderer
   cd /tmp
   git clone https://github.com/acados/tera_renderer.git
   cd tera_renderer
   git checkout v0.2.0
   cargo build --release

   # Copy the binary to acados
   sudo mkdir -p /opt/acados/bin
   sudo cp target/release/t_renderer /opt/acados/bin/
   sudo chmod +x /opt/acados/bin/t_renderer

   # Clean up
   cd /tmp
   rm -rf tera_renderer
   ```

7. **Install ACADOS Python interface:**
   ```bash
   # Activate virtual environment
   source ~/mpc_venv/bin/activate

   # Change ownership (replace 'lis:lis' with your username:group)
   sudo chown -R lis:lis /opt/acados

   # Install acados template
   cd /opt/acados/interfaces/acados_template
   pip install --no-build-isolation -e .
   ```

8. **Setup workspace and clone packages:**
   ```bash
   # Create workspace if it doesn't exist
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src

   # Clone the MPC controller packages
   git clone https://github.com/lis-epfl/mpc_controller_pkgs
   ```

9. **Generate ACADOS solvers:**
    ```bash
    cd ~/ros2_ws/src/mpc_controller_pkgs/mpc_controller_ros2/scripts
    python3 generate_acados_solvers.py
    ```

11. **Build the workspace:**
    ```bash
    cd ~/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select mpc_controller_ros2_msgs mpc_controller_ros2 px4_sim_bridge_ros2
    source install/setup.bash
    ```

## Usage

The MPC and INDI parameters can be configured in `config/mpc_config.yaml`. The horizon and the number of steps for the MPC can only be changed inside `generate_acados_solvers.py`.
For options that can be passed to the trajectory publisher and the plotter scripts, check the arguments inside them. The default trajectory publisher with no arguments will publish a helix.
**For the simulation to work, you need to download and launch** [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html).

### Docker Usage with Simulation

 Follow these steps to run the MPC controller with simulation in Docker:

1. **Start the simulation environment:**
   ```bash
   ./dockerfiles/run_simulation.sh
   ```

2. **Launch the MPC controller:**
   In a new terminal:
   ```bash
   cd dockerfiles
   ./launch_sim_mpc.sh
   ```

3. **Access the container:**
   ```bash
   docker exec -it ros2_mpc_container bash
   ```

4. **Run the trajectory publisher:**
   ```bash
   source install/setup.bash
   cd src/mpc_controller_pkgs/mpc_controller_ros2/scripts
   python3 trajectory_publisher.py
   ```

5. **Visualize results:**
   After the trajectory is completed (also could be done during the flight):
   ```bash
   python3 plot_trajectory_data.py
   ```

To launch the controller alone without the simulation bridge (for the real drone), you can launch `launch_mpc.sh`.

### Native Usage

For native installation:

1. **Build package:**
  Necessary after every change to `generate_acados_solvers.py`.
   ```bash
   cd ~/ros2_ws/src/mpc_controller_pkgs/mpc_controller_ros2/scripts
   python3 generate_acados_solvers.py
   cd ~/ros2_ws
   colcon build --symlink-install --packages-select px4_msgs mpc_controller_ros2_msgs mpc_controller_ros2
   ```

2. **Launch the simulation from the dockerfile:**
   ```bash
   cd ~/ros2_ws/src/mpc_controller_pkgs/
   ./dockerfiles/build_simulation.sh
   ./dockerfiles/run_simulation.sh
   ```

3. **Launch the MPC controller:**
   In a new terminal, in your ros2 workspace:
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   ros2 launch mpc_controller_ros2 mpc.launch.py
   ```
   Note: For simulation with PX4, you'll need to set up the simulation bridge separately.

5. **Publish trajectories**
In a new terminal, in your ros2 workspace:
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   cd src/mpc_controller_pkgs/mpc_controller_ros2/scripts
   python3 trajectory_publisher.py
   ```

6. **Visualize trajectories**
   ```bash
   python3 plot_trajectory_data.py
   ```
To launch the controller alone without the simulation bridge (for the real drone), you can launch `mpc.launch.py` from the `mpc_controller_ros2` package.
