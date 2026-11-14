"""Compose read-only pose observation and optional Backend delivery."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    def value(name):
        return LaunchConfiguration(name)

    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='tf', choices=['tf', 'odom']),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('pose_topic', default_value='/telemetry/pose'),
        DeclareLaunchArgument('target_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_backend', default_value='false'),
        DeclareLaunchArgument('map_id', default_value=''),
        DeclareLaunchArgument('device_token_file', default_value=''),
        DeclareLaunchArgument('pose_url', default_value='ws://127.0.0.1:8080/api/robots/cleany-01/pose/ws'),
        Node(package='cleany_telemetry', executable='pose_observer', output='screen', parameters=[{
            'source': value('source'), 'odom_topic': value('odom_topic'),
            'pose_topic': value('pose_topic'), 'target_frame': value('target_frame'),
            'base_frame': value('base_frame'),
            'use_sim_time': ParameterValue(value('use_sim_time'), value_type=bool),
        }]),
        Node(package='cleany_control_bridge', executable='pose_backend_bridge', output='screen',
             condition=IfCondition(value('enable_backend')), parameters=[{
                 'pose_topic': value('pose_topic'), 'url': value('pose_url'),
                 'map_frame': value('target_frame'),
                 'map_id': ParameterValue(value('map_id'), value_type=str),
                 'device_token_file': ParameterValue(value('device_token_file'), value_type=str),
                 'use_sim_time': ParameterValue(value('use_sim_time'), value_type=bool),
             }]),
    ])
