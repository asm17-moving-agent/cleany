"""Permanent hardware graph; the parent runtime owns and shuts down this launch."""

import math
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def nodes(context):
    profile = yaml.safe_load(
        Path(LaunchConfiguration("resolved_profile").perform(context)).read_text()
    )
    base = (
        Path(get_package_share_directory("cleany_base_driver"))
        / "launch/base_driver.launch.py"
    )
    result = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(base)),
            launch_arguments={
                "config": profile["base_config"],
                "mock": str(profile["mock"]).lower(),
            }.items(),
        )
    ]
    if not profile["mock"]:
        agent = profile["agent"]
        lidar = profile["lidar"]
        result.extend(
            [
                Node(
                    package="micro_ros_agent",
                    executable="micro_ros_agent",
                    name="mapping_usb_agent",
                    arguments=[
                        "serial",
                        "--dev",
                        agent["serial_port"],
                        "-b",
                        str(agent["baudrate"]),
                    ],
                    output="screen",
                    respawn=True,
                    respawn_delay=1.0,
                ),
                Node(
                    package="sllidar_ros2",
                    executable="sllidar_node",
                    name="mapping_lidar",
                    parameters=[
                        {
                            "serial_port": lidar["serial_port"],
                            "serial_baudrate": lidar["baudrate"],
                            "frame_id": lidar["frame_id"],
                            "scan_mode": lidar["scan_mode"],
                            "inverted": lidar["inverted"],
                            "angle_compensate": lidar["angle_compensate"],
                        }
                    ],
                    remappings=[("scan", "scan_raw")],
                    output="screen",
                    respawn=True,
                    respawn_delay=1.0,
                ),
            ]
        )
    mount = profile["mount"]
    xyz = mount["translation_m"]
    result.extend(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="mapping_lidar_mount",
                arguments=[
                    "--x",
                    str(xyz[0]),
                    "--y",
                    str(xyz[1]),
                    "--z",
                    str(xyz[2]),
                    "--yaw",
                    str(math.radians(mount["yaw_deg"])),
                    "--frame-id",
                    profile["base_frame"],
                    "--child-frame-id",
                    profile["lidar"]["frame_id"],
                ],
            ),
            Node(
                package="cleany_navigation",
                executable="mapping_scan_filter",
                parameters=[
                    {
                        "min_range_m": profile["scan_filter"]["min_range_m"],
                        "front_center_rad": profile["scan_filter"]["front_center_rad"],
                    }
                ],
                output="screen",
            ),
            Node(
                package="cleany_teleop",
                executable="phone_remote",
                arguments=[
                    "--base-config",
                    profile["base_config"],
                    "--host",
                    profile["host"],
                    "--port",
                    str(profile["phone_port"]),
                    "--max-speed",
                    str(profile["teleop"]["max_wheel_speed"]),
                    "--event-log",
                    str(Path(profile["token_file"]).parent / "teleop-events.jsonl"),
                    "--token-file",
                    profile["token_file"],
                ],
                output="screen",
                respawn=True,
                respawn_delay=1.0,
            ),
        ]
    )
    return result


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument("resolved_profile"), OpaqueFunction(function=nodes)]
    )
