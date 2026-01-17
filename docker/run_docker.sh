#!/bin/bash
# run_docker.sh
# Usage: ./run_docker.sh <docker_image_name> [optional_command]
# Example: ./run_docker.sh turtlebot_behavior:realsense "ros2 launch realsense2_camera rs_launch.py"

IMAGE_NAME="$1"
COMMAND="${2:-bash}"  # default to interactive bash if no command provided

# --- Volumes for X11 GUI ---
DOCKER_VOLUMES="
--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
--volume=${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority:rw \
"

# --- Environment Variables ---
DOCKER_ENV_VARS="
--env=DISPLAY=$DISPLAY \
--env=QT_X11_NO_MITSHM=1 \
--env=ROS_DOMAIN_ID=30 \
--env=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
--env=TURTLEBOT3_MODEL=waffle_pi \
--env=CYCLONEDDS_URI=file:///etc/cyclonedds.xml
"

# --- RealSense / USB devices ---
DOCKER_DEVICES="
--device=/dev/video0 \
--device=/dev/video1 \
--device=/dev/bus/usb
"

# --- Mount host CycloneDDS config if it exists ---
if [ -f "$HOME/cyclonedds.xml" ]; then
    DOCKER_VOLUMES="$DOCKER_VOLUMES --volume=$HOME/cyclonedds.xml:/etc/cyclonedds.xml:ro"
fi

# --- Run the container ---
docker run -it --rm \
  --net=host \
  --ipc=host \
  --privileged \
  $DOCKER_VOLUMES \
  $DOCKER_ENV_VARS \
  $DOCKER_DEVICES \
  $IMAGE_NAME \
  bash -c "$COMMAND"
