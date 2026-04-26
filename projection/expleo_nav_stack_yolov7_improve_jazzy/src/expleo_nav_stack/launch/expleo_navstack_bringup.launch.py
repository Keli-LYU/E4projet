# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import PushRosNamespace, Node
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    expleo_bringup_dir = get_package_share_directory("expleo_nav_stack")
    expleo_navstack_launch_dir = os.path.join(expleo_bringup_dir, "launch")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    slam = LaunchConfiguration("slam")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="False", description="Whether run a SLAM"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", description="Full path to map yaml file to load"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(expleo_bringup_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "slam_launch.py")
                ),
                condition=IfCondition(slam),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                }.items(),
            ),
            #IncludeLaunchDescription(
            #    PythonLaunchDescriptionSource(
            #        os.path.join(launch_dir, "localization_launch.py")
            #    ),
            #    condition=IfCondition(PythonExpression(["not ", slam])),
            #    launch_arguments={
            #        "namespace": namespace,
            #        "map": map_yaml_file,
            #        "use_sim_time": use_sim_time,
            #        "autostart": autostart,
            #        "params_file": params_file,
            #        "use_lifecycle_mgr": "false",
            #    }.items(),
            #),

            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[params_file, {'yaml_filename': map_yaml_file}],
                condition=IfCondition(PythonExpression(["not ", slam])) 
            ),

            # 2. Start a Lifecycle Manager specifically for the Map Server
            # (The navigation launch file has its own manager, but we need one for the map too)
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': ['map_server']}],
                condition=IfCondition(PythonExpression(["not ", slam]))
            ),

            # 3. Fake the Localization Transform (Map -> Odom)
            # Since we aren't running AMCL, we must statically link map to odom.
            # This assumes your robot spawns at the map origin (or Odom matches Map).
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                condition=IfCondition(PythonExpression(["not ", slam]))
            ),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        expleo_navstack_launch_dir,
                        "expleo_navstack_navigation.launch.py",
                    )
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_lifecycle_mgr": "false",
                    "map_subscribe_transient_local": "true",
                }.items(),
            ),
            #IncludeLaunchDescription(
                #PythonLaunchDescriptionSource(
                   # ['/opt/Carla/CARLA_0.9.13/PythonAPI/examples/generate_traffic.py']
               # ),
          #  ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
