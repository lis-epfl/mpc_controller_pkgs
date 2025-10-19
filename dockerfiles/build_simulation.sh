#!/bin/bash
set -e

echo "Building PX4 Simulation Docker image..."
echo "======================================="

# Check if required files exist
if [ ! -f "Dockerfile.simulation" ]; then
    echo "Error: Dockerfile.simulation not found. Please run from dockerfiles directory."
    exit 1
fi

if [ ! -f "dds_topics.yaml" ]; then
    echo "Error: dds_topics.yaml not found. Please copy your dds_topics.yaml to this directory."
    exit 1
fi

# Detect architecture
ARCH=$(uname -m)
echo "Detected architecture: $ARCH"

# Set base image based on architecture
if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    BASE_IMAGE="dustynv/ros:humble-desktop-l4t-r35.4.1"
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
    -f Dockerfile.simulation \
    -t px4_simulation:latest .

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ PX4 Simulation image built successfully!"
    echo "  Image name: px4_simulation:latest"
    echo "  Base image: $BASE_IMAGE"
    echo "  Architecture: $ARCH"
    echo ""
    echo "To run the container: ./run_simulation.sh"
else
    echo "✗ Build failed!"
    exit 1
fi
