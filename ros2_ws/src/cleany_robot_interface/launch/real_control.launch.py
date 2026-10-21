"""Feetech hardware and standard controllers; READ-only unless explicitly enabled."""
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from cleany_robot_interface.control_profile import (
    prepare_control_profile,
    resolve_moveit_trajectory_execution,
)


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    read_only = value('read_only') == 'true'
    allow_trajectory_execution = resolve_moveit_trajectory_execution(
        read_only, value('allow_trajectory_execution')
    )
    output = Path(value('output_root')).expanduser() / str(time.time_ns())
    profile = prepare_control_profile(Path(value('profile_file')).resolve(),
                                      Path(value('repository_root')).resolve(), output,
                                      Path(value('control_file')).resolve(), read_only)
    command_count = len(ET.parse(profile.model).findall('ros2_control/joint/command_interface'))
    if command_count != 12 and value('with_moveit') == 'true':
        raise ValueError('A partial commissioning profile requires with_moveit:=false')
    print(f'REAL_CONTROL_RUN_DIR={output}', flush=True)
    model = {'robot_description': profile.model.read_text(), 'use_sim_time': False}
    manager = Node(package='controller_manager', executable='ros2_control_node',
                   parameters=[model, str(profile.controllers)], output='screen')
    state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='real_robot_state_publisher', parameters=[model], output='screen')
    broadcaster = Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
                       output='screen')
    controllers = Node(package='controller_manager', executable='spawner',
                       arguments=[*profile.controller_names, '--controller-manager-timeout', '30',
                                  *(['--inactive'] if read_only else ['--activate-as-group'])], output='screen')

    def spawned(event, _context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason='Controller spawner failed'))]
        return []

    guard = Node(package='cleany_robot_interface', executable='control_state_guard', output='screen',
                 parameters=[{'controller_names': list(profile.controller_names)}])
    actions = [state, manager, guard, broadcaster, controllers]
    if value('with_moveit') == 'true':
        moveit = Path(get_package_share_directory('cleany_moveit_config')) / 'launch/physical_move_group.launch.py'
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(moveit)),
            launch_arguments={'robot_description_file': str(profile.model),
                              'controllers_file': str(profile.moveit_controllers),
                              'joint_limits_file': str(profile.joint_limits),
                              'use_rviz': value('use_rviz'),
                              'include_head_camera': 'true', 'use_sim_time': 'false',
                              'enable_depth_octomap': value('enable_depth_octomap'),
                              'allow_trajectory_execution': str(allow_trajectory_execution).lower()}.items()))
    guards = [RegisterEventHandler(OnProcessExit(target_action=node,
              on_exit=[EmitEvent(event=Shutdown(reason='Robot state/control process exited'))]))
              for node in (manager, state, guard)]
    guards += [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=spawned))
               for node in (broadcaster, controllers)]
    return guards + actions


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(key) for key in ('repository_root', 'profile_file', 'control_file')),
        DeclareLaunchArgument('read_only', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'allow_trajectory_execution',
            default_value='auto',
            choices=['auto', 'true', 'false'],
            description=(
                'MoveIt execution permission. Use false with read_only=false when a guarded '
                'external executor owns FollowJointTrajectory.'
            ),
        ),
        DeclareLaunchArgument('with_moveit', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_depth_octomap', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('output_root', default_value='~/.local/share/cleany/real_control'),
        OpaqueFunction(function=setup),
    ])
