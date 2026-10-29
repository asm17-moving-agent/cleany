"""Actual joint positions and TF; no command interface or controllers."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    reader = Node(package='cleany_robot_interface', executable='robot_state_reader', output='screen',
                  parameters=[{key: value(key) for key in
                               ('arm_config_file', 'head_config_file', 'head_mapping_file')}])
    state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='real_robot_state_publisher', output='screen',
                 parameters=[{'robot_description': Path(value('robot_description_file')).read_text(),
                              'use_sim_time': False}])
    return [RegisterEventHandler(OnProcessExit(
                target_action=reader, on_exit=[EmitEvent(event=Shutdown(reason='Live state reader exited'))])),
            RegisterEventHandler(OnProcessExit(
                target_action=state, on_exit=[EmitEvent(event=Shutdown(reason='Robot TF publisher exited'))])),
            reader, state]


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(key) for key in ('robot_description_file', 'arm_config_file',
                                                'head_config_file', 'head_mapping_file')),
        OpaqueFunction(function=setup),
    ])
