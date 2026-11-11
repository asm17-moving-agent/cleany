"""Connect an existing perception/MoveIt/physical runtime; default is audit only."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_node(context):
    execute = LaunchConfiguration('execute').perform(context)
    if execute not in ('true', 'false'):
        raise ValueError('execute must be true or false')
    arguments = ['--config', LaunchConfiguration('config').perform(context),
                 '--output', LaunchConfiguration('output').perform(context),
                 '--query', LaunchConfiguration('query').perform(context)]
    if execute == 'true':
        arguments.append('--execute')
    return [Node(package='cleany_skill_executor', executable='collection_pipeline',
                 arguments=arguments, output='screen',
                 parameters=[{
                     'use_sim_time': False,
                     'require_sensor_scene': True,
                     'planning_scene_timeout_sec': 5.0,
                 }])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', description='Physical collection JSON configuration'),
        DeclareLaunchArgument('output', description='Execution/audit report JSON path'),
        DeclareLaunchArgument('query', default_value=''),
        DeclareLaunchArgument('execute', default_value='false'),
        OpaqueFunction(function=create_node),
    ])
