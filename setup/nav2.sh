source /opt/ros/jazzy/setup.bash
source /home/lyukeli/ros-bridge/install/local_setup.bash
source /home/lyukeli/E4projet/projection/expleo_nav_stack_yolov7_improve_jazzy/install/local_setup.bash

ros2 launch expleo_nav_stack expleo_navstack_bringup.launch.py \
  map:=/home/lyukeli/E4projet/projection/expleo_nav_stack_yolov7_improve_jazzy/src/maps/Town10_xodr.yaml \
  params_file:=/home/lyukeli/E4projet/projection/expleo_nav_stack_yolov7_improve_jazzy/src/expleo_nav_stack/params/nav2_config_smac_hybrid__regulated_pure_pursuit__carla_yolov7.yaml \
  use_sim_time:=True
