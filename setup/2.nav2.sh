#!/usr/bin/env bash

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
ROS_BRIDGE_DIR="${ROS_BRIDGE_DIR:-${HOME}/ros-bridge}"
NAV_WS_DIR="${PROJECT_DIR}/projection/expleo_nav_stack_yolov7_improve_jazzy"

source "${ROS_SETUP}"
source "${ROS_BRIDGE_DIR}/install/local_setup.bash"
source "${NAV_WS_DIR}/install/local_setup.bash"

ros2 launch expleo_nav_stack expleo_navstack_bringup.launch.py \
  map:="${NAV_WS_DIR}/src/maps/Town10_xodr.yaml" \
  params_file:="${NAV_WS_DIR}/src/expleo_nav_stack/params/nav2_config_smac_hybrid__regulated_pure_pursuit__carla_yolov7.yaml" \
  use_sim_time:=True
