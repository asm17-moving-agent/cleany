"""Inspect CAD collision geometry and a cloud expressed in its reference frame."""

import os
from pathlib import Path
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from cleany_moveit_config.freecad_collision import inspection_urdf
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _inspection_nodes(context):
    if (
        os.environ.get("ROS_LOCALHOST_ONLY") != "1"
        or int(os.environ.get("ROS_DOMAIN_ID", "0")) == 0
    ):
        raise ValueError(
            "FreeCAD inspection requires localhost and a separate ROS domain"
        )

    variant = LaunchConfiguration("model_variant").perform(context)
    package = Path(get_package_share_directory("cleany_description"))
    cache = LaunchConfiguration("collision_cache").perform(context)
    urdf = inspection_urdf(package, variant, Path(cache) if cache else None)
    has_enclosures = any(
        collision.get("name", "").startswith("cad_enclosure__")
        for collision in ET.fromstring(urdf).findall("link/collision")
    )
    semantic = ET.tostring(
        ET.Element("robot", name=ET.fromstring(urdf).attrib["name"]),
        encoding="unicode",
    )
    description = {"robot_description": urdf}
    common = {
        "namespace": "freecad_inspection",
        "remappings": [
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/joint_states", "joint_states"),
        ],
        "output": "screen",
    }
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[description],
            **common,
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            on_exit=Shutdown(reason="FreeCAD inspection process stopped"),
            parameters=[
                description,
                {
                    "robot_description_semantic": semantic,
                    "allow_trajectory_execution": False,
                    "collision_detector": "cleany/CADInspectionFCL"
                    if has_enclosures
                    else "FCL",
                    "capabilities": "cleany/RequireCollisionDetector"
                    if has_enclosures
                    else "",
                    "disable_capabilities": " ".join(
                        "move_group/" + capability
                        for capability in (
                            "MoveGroupMoveAction",
                            "MoveGroupPlanService",
                            "MoveGroupExecuteService",
                            "MoveGroupExecuteTrajectoryAction",
                            "MoveGroupCartesianPathService",
                            "MoveGroupKinematicsService",
                            "MoveGroupQueryPlannersService",
                        )
                    ),
                    "sensors": ["depth_cloud"],
                    "octomap_frame": "base_link",
                    "octomap_resolution": 0.01,
                    "depth_cloud.sensor_plugin": (
                        "occupancy_map_monitor/PointCloudOctomapUpdater"
                    ),
                    "depth_cloud.point_cloud_topic": "/freecad_inspection/scene_cloud",
                    "depth_cloud.filtered_cloud_topic": "scene_cloud_filtered",
                    "depth_cloud.max_range": 2.0,
                    "depth_cloud.point_subsample": 1,
                    "depth_cloud.padding_offset": 0.0,
                    "depth_cloud.padding_scale": 1.0,
                    "depth_cloud.max_update_rate": 2.0,
                },
            ],
            **common,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_variant",
                default_value="freecad_kinematic",
                choices=["freecad_final", "freecad_kinematic"],
            ),
            DeclareLaunchArgument("collision_cache", default_value=""),
            OpaqueFunction(function=_inspection_nodes),
        ]
    )
