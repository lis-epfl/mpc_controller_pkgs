#!/bin/bash

CONTAINER_NAME="ros2_mpc_container"

# Check if container is running
if [ ! "$(docker ps -q -f name=^${CONTAINER_NAME}$)" ]; then
    echo "Container ${CONTAINER_NAME} is not running. Starting it first..."
    ./run_mpc.sh
    sleep 3
fi

# Check if workspace has been built
echo "Checking workspace build status..."
PACKAGES_BUILT=$(docker exec ${CONTAINER_NAME} bash -c "
    source /opt/ros/humble/setup.bash 2>/dev/null && \
    [ -f /root/ws_ros2/install/setup.bash ] && \
    source /root/ws_ros2/install/setup.bash 2>/dev/null && \
    ros2 pkg list 2>/dev/null | grep -q px4_sim_bridge_ros2 && echo 'yes' || echo 'no'" 2>/dev/null)

if [ "$PACKAGES_BUILT" != "yes" ]; then
    echo "Packages not built yet. Building workspace (this takes ~45 seconds on first run)..."
    docker exec ${CONTAINER_NAME} bash -c "
        source /opt/ros/humble/setup.bash && \
        cd /root/ws_ros2 && \
        colcon build"
    echo "✓ Build complete!"
fi

# Launch the simulation with MPC
echo "Launching simulation with MPC..."
docker exec -it ${CONTAINER_NAME} bash -c "
    source /opt/ros/humble/setup.bash && \
    source /root/ws_ros2/install/setup.bash && \
    ros2 launch px4_sim_bridge_ros2 sim_with_mpc.launch.py"
