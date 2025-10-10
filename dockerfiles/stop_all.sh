#!/bin/bash

echo "Stopping all containers..."

if [ "$(docker ps -q -f name=^ros2_mpc_container$)" ]; then
    echo "Stopping ros2_mpc_container..."
    docker stop ros2_mpc_container
fi

if [ "$(docker ps -q -f name=^px4_sim_container$)" ]; then
    echo "Stopping px4_sim_container..."
    docker stop px4_sim_container
fi

echo "✓ All containers stopped"
