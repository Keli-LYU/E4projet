#!/usr/bin/env bash

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

source "${ROS_SETUP}"
cd "${PROJECT_DIR}/AI_Part"
python3 semantic_node.py
