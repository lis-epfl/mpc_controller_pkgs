#!/bin/bash
# launch_sim_with_mpc.sh
# Simulation launcher with MPC controller

CONTAINER_NAME="ros2_mpc_container"

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}MPC Simulation Launcher${NC}"
echo "Setting up and launching simulation with MPC controller"
echo ""

# Check if container is running
if [ ! "$(docker ps -q -f name=^${CONTAINER_NAME}$)" ]; then
    echo -e "${YELLOW}Starting container...${NC}"
    ./run_mpc.sh
    sleep 3
fi

# Check if acados solver generation is needed
echo -e "${BLUE}Checking if acados solver regeneration is needed...${NC}"
docker exec ${CONTAINER_NAME} bash -c '
    source /opt/ros/humble/setup.bash
    cd /root/ws_ros2

    # Define paths
    SOLVER_SCRIPT="src/mpc_controller_pkgs/mpc_controller_ros2/scripts/generate_acados_solvers.py"
    CHECKSUM_FILE="/root/ws_ros2/.acados_generation_checksum"

    # List of files that should trigger regeneration when changed
    WATCH_PATTERN="src/mpc_controller_pkgs/mpc_controller_ros2/scripts/*.py"

    # Calculate current checksum of all relevant Python files
    CURRENT_CHECKSUM=""
    if ls $WATCH_PATTERN 1> /dev/null 2>&1; then
        CURRENT_CHECKSUM=$(find src/mpc_controller_pkgs/mpc_controller_ros2/scripts -name "*.py" -type f -exec md5sum {} \; | sort | md5sum | cut -d" " -f1)
    fi

    # Check if we need to regenerate
    REGENERATE=false

    if [ ! -f "$CHECKSUM_FILE" ]; then
        echo "  No previous generation checksum found - will generate"
        REGENERATE=true
    elif [ -z "$CURRENT_CHECKSUM" ]; then
        echo "  No Python files found - skipping generation"
    else
        # Get the stored checksum
        STORED_CHECKSUM=$(cat "$CHECKSUM_FILE" 2>/dev/null)

        if [ "$CURRENT_CHECKSUM" != "$STORED_CHECKSUM" ]; then
            echo "  Changes detected (checksum mismatch) - will regenerate"
            echo "    Previous: ${STORED_CHECKSUM:0:8}..."
            echo "    Current:  ${CURRENT_CHECKSUM:0:8}..."
            REGENERATE=true
        else
            echo "  No changes detected (checksum match) - skipping generation"
        fi
    fi

    # Generate acados solvers if needed
    if [ "$REGENERATE" = true ]; then
        echo "Generating acados solvers..."
        if python3 $SOLVER_SCRIPT; then
            # Update checksum file
            echo "$CURRENT_CHECKSUM" > "$CHECKSUM_FILE"
            echo "✓ Acados solvers generated successfully"
        else
            echo "✗ Failed to generate acados solvers"
            exit 1
        fi
    else
        echo "✓ Using existing acados solvers"
    fi
'

# Build all packages
echo -e "${GREEN}Building packages...${NC}"
docker exec ${CONTAINER_NAME} bash -c '
    source /opt/ros/humble/setup.bash
    cd /root/ws_ros2

    # Build message packages first (they are dependencies)
    echo "Building message packages..."
    colcon build --symlink-install --packages-select px4_msgs mpc_controller_ros2_msgs

    # Then build the controller and simulation bridge packages
    echo "Building MPC controller and simulation bridge..."
    colcon build --symlink-install --packages-select mpc_controller_ros2 px4_sim_bridge_ros2

    echo "✓ Build complete!"
'

# Launch the simulation with MPC
echo -e "${GREEN}Launching simulation with MPC...${NC}"
docker exec -it ${CONTAINER_NAME} bash -c "
    source /opt/ros/humble/setup.bash && \
    source /root/ws_ros2/install/setup.bash && \
    ros2 launch px4_sim_bridge_ros2 sim_with_mpc.launch.py"
