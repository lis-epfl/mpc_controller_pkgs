#!/bin/bash

echo "This will remove all containers and images. Are you sure? (y/N)"
read -r response

if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
    echo "Removing containers..."
    docker rm -f ros2_mpc_container 2>/dev/null || true
    docker rm -f px4_sim_container 2>/dev/null || true

    echo "Removing images..."
    docker rmi ros2_mpc_controller:latest 2>/dev/null || true
    docker rmi px4_simulation:latest 2>/dev/null || true

    echo "✓ Cleanup complete"
else
    echo "Cleanup cancelled"
fi
