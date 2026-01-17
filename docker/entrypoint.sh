#!/bin/bash
set -e

# --- Environment Variables ---
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}
echo "ROS_DOMAIN_ID set to $ROS_DOMAIN_ID"
echo "TURTLEBOT3_MODEL set to $TURTLEBOT3_MODEL"

# Source ROS 2 base
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS 2 ${ROS_DISTRO}"

# Source TurtleBot base workspace
[ -f /turtlebot_ws/install/setup.bash ] && source /turtlebot_ws/install/setup.bash && echo "Sourced TurtleBot base workspace"

# Source TurtleBot3 workspace
[ -f /turtlebot3_ws/install/setup.bash ] && source /turtlebot3_ws/install/setup.bash && echo "Sourced TurtleBot3 workspace"

# Source overlay workspace
[ -f /overlay_ws/install/setup.bash ] && source /overlay_ws/install/setup.bash && echo "Sourced autonomy overlay workspace"

exec "$@"
