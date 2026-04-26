# Expleo Navigation Stack

This meta package contains additional packages for the ROS2 Navigation Stack
that are used within Expleo.

## autonomous-forklift-for-harbor
This package contains the YOLOv7 obstacle detection node

## expleo_nav_stack
This package contains the main code for the new plugins.

### Usage with Carla

The package provides some launchfiles to be used with Carla.
This assumes that the `carla_ros_bridge` (available on GitHub) and the
`carla_twist_to_regulated_control` (available on Expleo's Gitlab [here](https://gitlab.extend.internal.expleogroup.com/expleo_ad/control/twist_to_regulated_control))
are installed and available in the workspace.

First, launch the Carla bridge node:

```sh
$ ros2 launch expleo_nav_stack carla_bridge_bringup.launch.py
```

You can then launch the navstack. Don't forget to specify the map to use!

```sh
$ ros2 launch expleo_nav_stack expleo_navstack_bringup.launch.py map:=./src/maps/Town10_xodr.yaml params_file:=./src/expleo_nav_stack/params/nav2_config_smac_hybrid__regulated_pure_pursuit__carla_yolov7.yaml
```

Launch the detection node in a third terminal:
```sh
$ ros2 run object_detection object_detection_node
```

## expleo_nav_msgs
This package contains all interafce files (msgs, srv and action).

## nav_yolov7_plugin
This package contains the obstacle detection plugin based on the YOLOv7 obstacle detection node

## perception2d_interfaces
This package contains msg files used in packages that compute compute 2D perception.

## teb_local_planner
The teb_local_planner package implements a plugin to the base_local_planner of the 2D navigation stack. 
