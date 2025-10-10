#!/bin/bash
set -e

# Source ROS 2 and local workspace
source /opt/ros/humble/setup.bash
if [ -f /root/ws_ros2/install/local_setup.bash ]; then
  source /root/ws_ros2/install/local_setup.bash
fi

# Execute command
exec "$@"
