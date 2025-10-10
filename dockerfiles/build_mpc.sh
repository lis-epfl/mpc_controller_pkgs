#!/bin/bash
set -e

echo "Building ROS2 MPC Controller Docker image..."
echo "============================================"

if [ ! -f "Dockerfile.ros2_mpc" ]; then
    echo "Error: Dockerfile.ros2_mpc not found. Please run from dockerfiles directory."
    exit 1
fi

docker build -f Dockerfile.ros2_mpc -t ros2_mpc_controller:latest .

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ ROS2 MPC Controller image built successfully!"
    echo "  Image name: ros2_mpc_controller:latest"
    echo ""
    echo "To run the container: ./run_mpc.sh"
else
    echo "✗ Build failed!"
    exit 1
fi
