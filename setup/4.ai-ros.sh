#!/usr/bin/env bash

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
NAV_WS_DIR="${PROJECT_DIR}/projection/expleo_nav_stack_yolov7_improve_jazzy"

source "${ROS_SETUP}"
source "${NAV_WS_DIR}/install/local_setup.bash"
cd "${PROJECT_DIR}"
python3 ./ai-ros.py
