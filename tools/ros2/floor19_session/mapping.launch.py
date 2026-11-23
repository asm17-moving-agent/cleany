"""Map a new floor from an existing manual drive session, with private map and TF topics."""

import math
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    run = Path(__file__).resolve().parent
    profile = yaml.safe_load((run / "config/profile.yaml").read_text())
    lidar, mount, scan = (profile[key] for key in ("lidar", "mount", "scan_filter"))
    x, y, z = mount["translation_m"]
    yaw = math.radians(mount["yaw_deg"])
    namespace = "/floor19"
    tf = [("/tf", namespace + "/tf"), ("/tf_static", namespace + "/tf_static")]

    def node(package, executable, **kwargs):
        return Node(
            package=package,
            executable=executable,
            namespace=namespace,
            output="screen",
            **kwargs
        )

    children = [
        node(
            "sllidar_ros2",
            "sllidar_node",
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
            remappings=[("scan", "scan_raw"), ("/scan", namespace + "/scan_raw")],
        ),
        node(
            "cleany_navigation",
            "mapping_scan_filter",
            parameters=[
                {
                    "min_range_m": scan["min_range_m"],
                    "front_center_rad": -yaw,
                    "front_half_angle_rad": math.radians(scan["half_angle_deg"]),
                }
            ],
        ),
        node(
            "cleany_base_driver",
            "odom_relay_node",
            remappings=[("wheel/odom", "/wheel/odom"), *tf],
        ),
        node(
            "tf2_ros",
            "static_transform_publisher",
            arguments=[
                "--x",
                str(x),
                "--y",
                str(y),
                "--z",
                str(z),
                "--yaw",
                str(yaw),
                "--frame-id",
                profile["base_frame"],
                "--child-frame-id",
                lidar["frame_id"],
            ],
            remappings=tf,
        ),
        node(
            "cartographer_ros",
            "cartographer_node",
            arguments=[
                "-configuration_directory",
                str(run / "config"),
                "-configuration_basename",
                "cartographer_hardware_2d.lua",
                *(
                    [
                        "-load_state_filename",
                        str(run / "resume.pbstream"),
                        "-load_frozen_state",
                        "true",
                    ]
                    if (run / "resume.pbstream").is_file()
                    else []
                ),
                "-save_state_filename",
                str(run / "session.pbstream"),
            ],
            remappings=[
                *tf,
                ("/scan", namespace + "/scan"),
                ("/odom", namespace + "/odom"),
            ],
        ),
        node(
            "cartographer_ros",
            "cartographer_occupancy_grid_node",
            arguments=["-resolution", "0.05", "-publish_period_sec", "2.0"],
            remappings=[("/map", namespace + "/map")],
        ),
    ]
    guards = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=child,
                on_exit=[EmitEvent(event=Shutdown(reason="mapping child exited"))],
            )
        )
        for child in children
    ]
    return LaunchDescription([*guards, *children])
