#!/bin/bash
set -e

echo "Building PX4 Simulation Docker image..."
echo "======================================="

if [ ! -f "Dockerfile.simulation" ]; then
    echo "Error: Dockerfile.simulation not found. Please run from dockerfiles directory."
    exit 1
fi

if [ ! -f "dds_topics.yaml" ]; then
    echo "Error: dds_topics.yaml not found. Please copy your dds_topics.yaml to this directory."
    exit 1
fi

docker build -f Dockerfile.simulation -t px4_simulation:latest .

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ PX4 Simulation image built successfully!"
    echo "  Image name: px4_simulation:latest"
    echo ""
    echo "To run the container: ./run_simulation.sh"
else
    echo "✗ Build failed!"
    exit 1
fi
