source /opt/ros/jazzy/setup.bash
source /home/lyukeli/ros-bridge/install/local_setup.bash
source /home/lyukeli/E4projet/projection/expleo_nav_stack_yolov7_improve_jazzy/install/local_setup.bash

cd /home/lyukeli/E4projet
python3 ./carla-follow-ego.py
