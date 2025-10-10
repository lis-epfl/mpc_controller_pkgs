#!/bin/bash

CONTAINER_NAME="ros2_mpc_container"
IMAGE_NAME="ros2_mpc_controller:latest"

# Check if container already exists
if [ "$(docker ps -aq -f name=^${CONTAINER_NAME}$)" ]; then
    echo "Container ${CONTAINER_NAME} already exists."

    # Check if it's running
    if [ "$(docker ps -q -f name=^${CONTAINER_NAME}$)" ]; then
        echo "Container is already running."
    else
        echo "Starting existing container..."
        docker start ${CONTAINER_NAME}
        sleep 2
    fi
else
    echo "Creating and starting new MPC controller container..."
    docker run -dit \
        --name ${CONTAINER_NAME} \
        --network host \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e ROS_DOMAIN_ID=0 \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v $(pwd)/../:/root/ws_ros2/src/mpc_controller_pkgs \
        ${IMAGE_NAME}

    echo "✓ Container ${CONTAINER_NAME} created and running"
fi

echo "To enter the container: docker exec -it ${CONTAINER_NAME} bash"
