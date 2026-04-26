import os

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import LaunchConfiguration as LC


def generate_launch_description():
    ld = []

    launch_arguments = [
        (
            "objects_definition_file",
            os.path.join(
                get_package_share_directory("expleo_nav_stack"),
                "params",
                "tesla_with_sensors.json",
            ),
        ),
        ("spawn_point_ego_vehicle", {"x": -74.4, "y": 35.6, "z": 0.0,"roll": 0.0, "pitch": 0.0, "yaw": -180}),
        ("spawn_sensors_only", "False"),
    ]

    for param, default in launch_arguments:
        ld.append(
            launch.actions.DeclareLaunchArgument(name=param, default_value=default)
        )

    ld.append(
        launch_ros.actions.Node(
            package="carla_spawn_objects",
            executable="carla_spawn_objects",
            name="carla_spawn_objects",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"objects_definition_file": LC("objects_definition_file")},
                {"spawn_point_ego_vehicle": LC("spawn_point_ego_vehicle")},
                {"spawn_sensors_only": LC("spawn_sensors_only")},
            ],
        )
    )
    return launch.LaunchDescription(ld)


if __name__ == "__main__":
    generate_launch_description()
