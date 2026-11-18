"""Combine wheel odometry and fixed-bias gyro, with shadow output by default."""
from math import isfinite
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    mode = value('mode')
    if mode not in ('shadow', 'canonical'):
        raise ValueError('mode must be shadow or canonical')
    canonical = mode == 'canonical'
    share = Path(get_package_share_directory('cleany_base_driver'))
    nodes = []
    if value('publish_imu_tf').lower() == 'true':
        angles = [value('imu_' + axis) for axis in ('roll', 'pitch', 'yaw')]
        if any(not angle or not isfinite(float(angle)) for angle in angles):
            raise ValueError('publish_imu_tf requires explicit imu_roll, imu_pitch and imu_yaw')
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher', name='imu_mount_tf',
            arguments=['--frame-id', 'base_link', '--child-frame-id', 'imu_link',
                       '--roll', angles[0], '--pitch', angles[1], '--yaw', angles[2]],
            output='screen'))
    nodes.extend([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/base_driver.launch.py')),
            launch_arguments={
                'config': value('base_config'), 'mock': value('mock'),
                'relay_odom': 'false' if canonical else 'true',
                'odometry_config': value('ekf_config'),
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/imu_bringup.launch.py')),
            launch_arguments={'config': value('imu_config')}.items()),
        Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            parameters=[value('ekf_config'), {'publish_tf': canonical}],
            remappings=[('odometry/filtered', 'odom' if canonical else 'odometry/filtered')],
            output='screen'),
    ])
    return nodes


def generate_launch_description():
    config = Path(get_package_share_directory('cleany_base_driver')) / 'config'
    return LaunchDescription([
        DeclareLaunchArgument('base_config', default_value=str(config / 'base_hardware.yaml')),
        DeclareLaunchArgument('imu_config', default_value=str(config / 'imu_bringup.yaml')),
        DeclareLaunchArgument('ekf_config', default_value=str(config / 'base_ekf.yaml')),
        DeclareLaunchArgument('mock', default_value='false'),
        DeclareLaunchArgument('mode', default_value='shadow', choices=['shadow', 'canonical'],
                              description='Shadow keeps wheel odom/TF; canonical transfers both to EKF.'),
        DeclareLaunchArgument('publish_imu_tf', default_value='false',
                              description='Use only when no robot description owns the IMU mount.'),
        DeclareLaunchArgument('imu_roll', default_value=''),
        DeclareLaunchArgument('imu_pitch', default_value=''),
        DeclareLaunchArgument('imu_yaw', default_value=''),
        OpaqueFunction(function=_nodes),
    ])
