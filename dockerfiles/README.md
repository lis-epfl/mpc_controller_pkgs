# Docker Setup for ROS2 MPC Controller

## Quick Start

### 1. First Time Setup
```bash
# Run this setup script
./setup_docker_clean.sh

# Build Docker images
./build_mpc.sh

# Start container
./run_mpc.sh

# Build workspace (required once)
./build_workspace.sh

# Launch MPC controller
./launch_mpc.sh
```

### 2. Daily Use
```bash
# Start container (if not running)
./run_mpc.sh

# Launch MPC controller (auto-builds if needed)
./launch_mpc.sh
```

## Scripts

| Script | Purpose | When to Use |
|--------|---------|-------------|
| `build_mpc.sh` | Build Docker image | Once, when setting up |
| `run_mpc.sh` | Start container | Once per work session |
| `build_workspace.sh` | Build ROS2 packages | Once after container creation |
| `launch_mpc.sh` | Launch MPC controller | Every time you want to run |
| `test_setup.sh` | Verify installation | To check if everything works |
| `diagnose.sh` | Debug issues | When something goes wrong |
| `stop_all.sh` | Stop containers | End of work session |
| `clean_all.sh` | Remove everything | Complete reset |

## Common Commands

### Enter container shell
```bash
docker exec -it ros2_mpc_container bash
```

### Check logs
```bash
docker logs ros2_mpc_container
```

### Restart container
```bash
docker restart ros2_mpc_container
```

## Build Behavior

- **First launch**: Builds workspace (~45 seconds)
- **Subsequent launches**: No rebuild (fast, <2 seconds)
- **After editing Python/config**: No rebuild needed
- **After editing C++**: Rebuilds only changed packages (~10 seconds)

## Troubleshooting

### Container not running
```bash
./run_mpc.sh
```

### Start fresh
```bash
./clean_all.sh
./build_mpc.sh
./run_mpc.sh
./build_workspace.sh
```

## Architecture

- **Base Image**: `osrf/ros:humble-desktop-full`
- **Mount**: Your packages at `/root/ws_ros2/src/mpc_controller_pkgs/`
- **Build**: Workspace at `/root/ws_ros2/install/`
- **Network**: Host mode for ROS2 communication
