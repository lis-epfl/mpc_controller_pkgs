#!/bin/bash

# launch_mpc.sh
# Fixed version - no redundant copying, smart rebuilding

CONTAINER_NAME="ros2_mpc_container"

# Check if container is running, start if needed
if [ ! "$(docker ps -q -f name=^${CONTAINER_NAME}$)" ]; then
    echo "Container ${CONTAINER_NAME} is not running. Starting it first..."
    ./run_mpc.sh
    sleep 3

    # Verify it started
    if [ ! "$(docker ps -q -f name=^${CONTAINER_NAME}$)" ]; then
        echo "Error: Failed to start container ${CONTAINER_NAME}"
        echo "Check: docker logs ${CONTAINER_NAME}"
        exit 1
    fi
fi

# Check and build workspace
echo "Checking workspace and building if needed..."
docker exec ${CONTAINER_NAME} bash -c '
    source /opt/ros/humble/setup.bash
    cd /root/ws_ros2

    # Check if this is first build or incremental
    if [ ! -d "install" ]; then
        echo "First time build - this will take ~45 seconds..."
        colcon build
        echo "✓ Initial build complete!"
    else
        # Only rebuild packages that changed
        echo "Checking for changes..."
        OUTPUT=$(colcon build --packages-skip-build-finished 2>&1)

        # Check if anything was actually built
        if echo "$OUTPUT" | grep -q "Starting >>>"; then
            echo "$OUTPUT" | grep "Starting >>>" | sed "s/Starting >>> /  Building: /"
            echo "✓ Build complete!"
        elif echo "$OUTPUT" | grep -q "All selected packages are already built"; then
            echo "✓ No changes detected, skipping build"
        else
            # Show output if something unexpected happened
            echo "$OUTPUT"
        fi
    fi
'

# Verify packages are available before launching
echo "Verifying packages..."
PACKAGES_OK=$(docker exec ${CONTAINER_NAME} bash -c "
    source /opt/ros/humble/setup.bash 2>/dev/null
    source /root/ws_ros2/install/setup.bash 2>/dev/null
    ros2 pkg list 2>/dev/null | grep -q mpc_controller_ros2 && echo 'yes' || echo 'no'
" 2>/dev/null)

if [ "$PACKAGES_OK" != "yes" ]; then
    echo "Error: mpc_controller_ros2 package not found!"
    echo "Trying to diagnose..."
    docker exec ${CONTAINER_NAME} bash -c "
        echo 'Packages in workspace:'
        ls -la /root/ws_ros2/src/mpc_controller_pkgs/ 2>/dev/null || echo 'Mount not found!'
        echo ''
        echo 'Built packages:'
        source /opt/ros/humble/setup.bash 2>/dev/null
        source /root/ws_ros2/install/setup.bash 2>/dev/null
        ros2 pkg list 2>/dev/null | grep -E '(mpc_controller|px4|omninxt)' || echo 'None found'
    "
    exit 1
fi

# Launch the MPC controller
echo "Launching MPC controller..."
docker exec -it ${CONTAINER_NAME} bash -c "
    source /opt/ros/humble/setup.bash && \
    source /root/ws_ros2/install/setup.bash && \
    ros2 launch mpc_controller_ros2 mpc.launch.py"
