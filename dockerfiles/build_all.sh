#!/bin/bash
set -e

echo "Building all Docker images..."
echo "=============================="
echo ""

# Build MPC controller image
./build_mpc.sh
echo ""

# Build simulation image (optional)
if [ -f "dds_topics.yaml" ]; then
    ./build_simulation.sh
else
    echo "Skipping simulation image (dds_topics.yaml not found)"
fi

echo ""
echo "=============================="
echo "✓ Build complete!"
