# Expleo Navigation Stack

This package is an extension of the [ROS2 navigation stack](https://github.com/ros-planning/navigation2)
adapted for use within Expleo's Autonomous Driving R&D project.
The aim of this package is to integrate an intermediate planning level between
the global planner and the controller, which are already available in the
Navigation2 software stack. This intermediate planning level will enable
the use of dedicated local planning algorithms, widely used in autonomous
driving methods.

## Overview

## Behavior Tree action

The global behavior of the system is managed by the Behavior Tree. The BT is
provided by an XML file which describes at a high level the actions to take,
their order and how they exchange data. See [nav2_behavior_tree](https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree)
for more information on how BTs are used in the navigation stack.

In this package, a new BT action is implemented to use the local planning
action server. This is implemented as a plugin compiled as a shared library.
The structure of the plugin is based on the structure of the
`ComputePathToPoseAction` available in the `nav2_behavior_tree` package.

The package provides the `ComputeLocalPlan` BT action that can be used in the
description of the BT, to access the `ComputeLocalPlan` action server
implemented in the planner server.
