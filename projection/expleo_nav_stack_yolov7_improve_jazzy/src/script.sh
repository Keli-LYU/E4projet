#!/bin/bash

CMD1="ros2 launch expleo_nav_stack carla_bridge_bringup.launch.py"
CMD2="ros2 launch expleo_nav_stack expleo_navstack_bringup.launch.py map:=./src/expleo_nav_stack/maps/Town10_xodr.yaml params_file:=./src/expleo_nav_stack/expleo_nav_stack/params/nav2_config_smac_hybrid__regulated_pure_pursuit__carla_yolov7.yaml"
CMD3="ros2 run object_detection object_detection_node"
CMD4="/opt/Carla/CARLA_0.9.13/CarlaUE4.sh"

cd /home/hdl_sunshine/IlyessOuazzaniChahdi/expleo_nav_stack_ws

source /opt/ros/galactic/setup.bash
source install/setup.bash


# Run each command in a new terminal window
gnome-terminal -- bash -c "$CMD4; exec bash"
sleep 10
gnome-terminal -- bash -c "$CMD1; exec bash"
gnome-terminal -- bash -c "$CMD2; exec bash"
gnome-terminal -- bash -c "$CMD3; exec bash"