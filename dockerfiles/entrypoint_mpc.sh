#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source the workspace if it exists
if [ -f "/root/ws_ros2/install/setup.bash" ]; then
    source /root/ws_ros2/install/setup.bash
fi

# Execute command
exec "$@"
