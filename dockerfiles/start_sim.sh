#!/bin/bash
set -e

# Start the Micro XRCE-DDS Agent in the background
echo "Starting Micro XRCE-DDS Agent..."
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!
echo "Agent started with PID: $AGENT_PID"

# Navigate to the PX4 directory
cd /root/PX4-Autopilot

# Build and launch the PX4 SITL with Gazebo
echo "Building and starting PX4 SITL with Gazebo..."
make px4_sitl gz_x500

# Wait for the agent to close if SITL exits
wait $AGENT_PID
