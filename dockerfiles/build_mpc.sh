#!/bin/bash
set -e

echo "Building ROS2 MPC Controller Docker image..."
echo "============================================"

# Check if Dockerfile exists
if [ ! -f "Dockerfile.ros2_mpc" ]; then
    echo "Error: Dockerfile.ros2_mpc not found. Please run from dockerfiles directory."
    exit 1
fi

# Detect architecture
ARCH=$(uname -m)
echo "Detected architecture: $ARCH"

# Set base image based on architecture
if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    BASE_IMAGE="dustynv/ros:humble-desktop-l4t-r36.2.0"
    echo "Using ARM64 base image: $BASE_IMAGE"
elif [ "$ARCH" = "x86_64" ]; then
    BASE_IMAGE="osrf/ros:humble-desktop-full"
    echo "Using AMD64 base image: $BASE_IMAGE"
else
    echo "Error: Unsupported architecture: $ARCH"
    exit 1
fi

# Build the Docker image with the appropriate base image
echo "Building Docker image..."
docker build \
    --build-arg BASE_IMAGE=$BASE_IMAGE \
    -f Dockerfile.ros2_mpc \
    -t ros2_mpc_controller:latest .

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ ROS2 MPC Controller image built successfully!"
    echo "  Image name: ros2_mpc_controller:latest"
    echo "  Base image: $BASE_IMAGE"
    echo "  Architecture: $ARCH"
    echo ""
    echo "To run the container: ./run_mpc.sh"
else
    echo "✗ Build failed!"
    exit 1
fi
