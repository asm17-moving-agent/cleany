"""Relay MCU-calibrated gyro and status."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('cleany_base_driver'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config' / 'imu_bringup.yaml')),
        Node(package='cleany_base_driver', executable='imu_driver_node', name='cleany_imu',
             parameters=[LaunchConfiguration('config')], output='screen'),
    ])
