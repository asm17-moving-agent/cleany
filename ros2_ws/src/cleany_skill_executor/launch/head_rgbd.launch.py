"""Publish the head RGB-D stream and calibrated frames without arm control."""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def setup(context):
    root = Path(LaunchConfiguration('repository_root').perform(context)).expanduser().resolve()
    profile_path = Path(LaunchConfiguration('profile_file').perform(context)).expanduser()
    profile = json.loads((root / profile_path).read_text())
    head = json.loads((root / Path(profile['head_config']).expanduser()).read_text())
    bridge_config = root / Path(profile['rgbd_bridge']).expanduser()
    perception = Path(get_package_share_directory('cleany_perception'))
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(perception / 'launch/realsense.launch.py')),
            launch_arguments={
                'serial_no': str(head['camera_serial']),
                'config_file': str(perception / 'config/realsense_pregrasp.yaml'),
            }.items(),
        ),
        Node(package='cleany_robot_interface', executable='rgbd_frame_bridge', output='screen',
             parameters=[{'config_file': str(bridge_config)}], respawn=True, respawn_delay=3.0),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('repository_root'),
        DeclareLaunchArgument('profile_file', default_value=PathJoinSubstitution([
            LaunchConfiguration('repository_root'), 'configs/robot/cad_grasp.json',
        ])),
        OpaqueFunction(function=setup),
    ])
