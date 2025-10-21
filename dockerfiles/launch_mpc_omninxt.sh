#!/bin/bash
# launch_mpc.sh
# Development version with automatic Tera renderer setup

CONTAINER_NAME="ros2_mpc_container"

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}MPC Controller Development Launcher${NC}"
echo "This script ensures all changes are picked up"
echo ""

# --- ADDED: Handle ROS_DOMAIN_ID from command-line argument ---
if [ -z "$1" ]; then
    echo -e "${YELLOW}Usage: $0 <ROS_DOMAIN_ID>${NC}"
    echo -e "${YELLOW}No ROS_DOMAIN_ID provided. Using default: 0${NC}"
    ROS_DOMAIN_ID=0
else
    ROS_DOMAIN_ID=$1
fi
echo -e "${BLUE}Using ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}${NC}\n"
# --- END ADDED SECTION ---

# Check if container is running
if [ ! "$(docker ps -q -f name=^${CONTAINER_NAME}$)" ]; then
    echo -e "${YELLOW}Starting container...${NC}"
    ./run_mpc.sh
    sleep 3
fi

# Check if acados solver generation is needed
echo -e "${BLUE}Checking if acados solver regeneration is needed...${NC}"
docker exec ${CONTAINER_NAME} bash -c '
    if [ -f "/opt/ros/humble/install/setup.bash" ]; then
        source /opt/ros/humble/install/setup.bash
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
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
    if [ -f "/opt/ros/humble/install/setup.bash" ]; then
        source /opt/ros/humble/install/setup.bash
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
    cd /root/ws_ros2

    # Build message packages first (they are dependencies)
    echo "Building message packages..."
    colcon build --symlink-install --packages-select px4_msgs mpc_controller_ros2_msgs

    # Then build the main package
    echo "Building MPC controller package..."
    colcon build --symlink-install --packages-select mpc_controller_ros2

    echo "✓ Build complete!"
'

# Launch
echo -e "${GREEN}Launching MPC controller...${NC}"
docker exec -it ${CONTAINER_NAME} bash -c "
    if [ -f \"/opt/ros/humble/install/setup.bash\" ]; then
        source /opt/ros/humble/install/setup.bash
    elif [ -f \"/opt/ros/humble/setup.bash\" ]; then
        source /opt/ros/humble/setup.bash
    fi
    source /root/ws_ros2/install/setup.bash && \\
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && \\
    ros2 launch mpc_controller_ros2 mpc_omninxt.launch.py"
