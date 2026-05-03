import os
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import launch_ros
import launch
from launch.substitutions import LaunchConfiguration as LC


def _load_wsl_bridge():
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "wsl-bridge" / "wsl-bridge.py"
        if candidate.exists():
            spec = importlib.util.spec_from_file_location("projection_wsl_bridge", candidate)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
    return None


def _default_carla_host():
    helper = _load_wsl_bridge()
    if helper is None:
        return os.environ.get("CARLA_HOST", "localhost")
    helper.apply_pythonpath_sanitization()
    return helper.detect_carla_host()


def generate_launch_description():
    ld = []

    launch_arguments = [
        ("host", _default_carla_host()),
        ("port", "2000"),
        ("timeout", "20"),
        ("role_name", "ego_vehicle"),
        ("town", "Town10HD"),
        ("passive", "False"),
        ("synchronous_mode_wait_for_vehicle_control_command", "False"),
        ("fixed_delta_seconds", "0.05"), # 20 FPS（仿真步长）
        ("spawn_point_ego_vehicle",{"x": -84.4, "y": 35.6, "z": 0.0,"roll": 0.0, "pitch": 0.0, "yaw": -180}),
        ("vehicle_filter", "vehicle.*"),
        (
            "objects_definition_file",
            os.path.join(
                get_package_share_directory("expleo_nav_stack"),
                "params",
                "tesla_with_sensors.json",
            ),
        ),
    ]

    for param, default in launch_arguments:
        ld.append(
            launch.actions.DeclareLaunchArgument(name=param, default_value=default)
        )

    # launch the ROS bridge node
    ld.append(
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("carla_ros_bridge"),
                    "carla_ros_bridge.launch.py",
                )
            ),
            launch_arguments={
                "host": LC("host"),
                "port": LC("port"),
                "town": LC("town"),
                "timeout": LC("timeout"),
                "passive": LC("passive"),
                "synchronous_mode_wait_for_vehicle_control_command": LC(
                    "synchronous_mode_wait_for_vehicle_control_command"
                ),
                "fixed_delta_seconds": LC("fixed_delta_seconds"),
            }.items(),
        ),
    )

    # spawn the ego-vehicle with its sensors
    ld.append(
        launch.actions.TimerAction(
            # add a 5-seconds delay to make sure the bridge is launched before the EV is spawned
            period=10.0,
            actions=[
                launch.actions.IncludeLaunchDescription(
                    launch.launch_description_sources.PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory("expleo_nav_stack"),
                            "launch",
                            "spawn_EV.launch.py",
                        )
                    ),
                    launch_arguments={
                        "host": launch.substitutions.LaunchConfiguration("host"),
                        "port": launch.substitutions.LaunchConfiguration("port"),
                        "timeout": launch.substitutions.LaunchConfiguration("timeout"),
                        "vehicle_filter": launch.substitutions.LaunchConfiguration(
                            "vehicle_filter"
                        ),
                        "role_name": launch.substitutions.LaunchConfiguration(
                            "role_name"
                        ),
                        "spawn_point_ego_vehicle": launch.substitutions.LaunchConfiguration(
                            "spawn_point_ego_vehicle"
                        ),
                        "objects_definition_file": launch.substitutions.LaunchConfiguration(
                            "objects_definition_file"
                        ),
                    }.items(),
                )
            ],
        )
    )

    # convert twist from nav2 to control
    ld.append(
        launch_ros.actions.Node(
            executable="twist_to_regulated_control",
            package="carla_twist_to_regulated_control",
            parameters=[
                {"twist_input_topic": "/cmd_vel"},
                {"odometry_input_topic": "/carla/ego_vehicle/odometry"},
                {"carla_control_topic": "/carla/ego_vehicle/vehicle_control_cmd"},
                {"SpeedController.Kp": 0.15},
                {"SpeedController.Ki": 0.2},
                {"SpeedController.Kd": 0.01},
                {"SpeedController.sat_min": 0.0},
                {"SpeedController.sat_max": 1.0},
                {"SpeedController.int_sat_min": 0.0},
                {"SpeedController.int_sat_max": 2.0},
            ],
        )
    )

    # publish static frames
    ld.append(
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        )
    )

    ld.append(
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "ego_vehicle", "base_link"],
        )
    )

    return launch.LaunchDescription(ld)


if __name__ == "__main__":
    generate_launch_description()
