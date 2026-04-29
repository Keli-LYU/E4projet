#!/usr/bin/env bash

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
ROS_BRIDGE_DIR="${ROS_BRIDGE_DIR:-${HOME}/ros-bridge}"
NAV_WS_DIR="${PROJECT_DIR}/projection/expleo_nav_stack_yolov7_improve_jazzy"

source "${ROS_SETUP}"
source "${ROS_BRIDGE_DIR}/install/local_setup.bash"
source "${NAV_WS_DIR}/install/local_setup.bash"

ros2 launch expleo_nav_stack carla_bridge_bringup.launch.py
