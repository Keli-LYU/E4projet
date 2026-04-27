source /opt/ros/jazzy/setup.bash
source /home/lyukeli/ros-bridge/install/local_setup.bash
source /home/lyukeli/E4projet/projection/expleo_nav_stack_yolov7_improve_jazzy/install/local_setup.bash

ros2 launch expleo_nav_stack carla_bridge_bringup.launch.py
