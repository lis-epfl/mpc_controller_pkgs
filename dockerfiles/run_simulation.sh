#!/bin/bash

CONTAINER_NAME="px4_sim_container"
IMAGE_NAME="px4_simulation:latest"

# Allow local connections to the X server
echo "Setting xhost permissions for Docker GUI..."
xhost +local:

# Check if container already exists
if [ "$(docker ps -aq -f name=^${CONTAINER_NAME}$)" ]; then
    echo "Container ${CONTAINER_NAME} already exists."

    if [ "$(docker ps -q -f name=^${CONTAINER_NAME}$)" ]; then
        echo "Container is already running."
    else
        echo "Starting existing container..."
        docker start ${CONTAINER_NAME}
        sleep 2
    fi
else
    echo "Creating and starting new PX4 simulation container..."
    docker run -dit \
        --name ${CONTAINER_NAME} \
        --network host \
        --privileged \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e ROS_DOMAIN_ID=0 \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        ${IMAGE_NAME}

    echo "✓ Container ${CONTAINER_NAME} created and running"
fi
