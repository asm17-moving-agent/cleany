"""Feetech hardware and standard controllers; READ-only unless explicitly enabled."""
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
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
    if not read_only:
        from cleany_robot_interface.calibrated_cad_model import require_cad_motion_ready
        require_cad_motion_ready(Path(value('profile_file')).resolve(),
                                 Path(value('repository_root')).resolve())
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
    # Queue Fast DDS writes so outgoing state does not hold up feedback processing.
    feedback_env = {'RMW_FASTRTPS_PUBLICATION_MODE': 'ASYNCHRONOUS'}
    feedback_profile = value('feedback_fastdds_profile')
    if feedback_profile:
        feedback_env['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(
            Path(feedback_profile).expanduser().resolve(strict=True))
    staged = value('with_moveit') == 'true' and value('staged_moveit') == 'true'
    cold = value('preload_cold_controllers') == 'true'
    defer_activation = value('defer_cold_activation') == 'true'
    if defer_activation and (not cold or allow_trajectory_execution):
        raise ValueError('Deferred activation requires cold controllers and plan-only MoveIt')
    if cold and (read_only or not staged):
        raise ValueError('Cold controller activation requires staged physical control')
    manager_parameters = [model, str(profile.controllers)]
    if cold:
        manager_parameters.append({'hardware_components_initial_state.inactive': ['FeetechSystem']})
    feedback_cpus = os.environ.get('CLEANY_REAL_FEEDBACK_CPU_LIST')
    feedback_prefix = ['taskset -c ' + feedback_cpus] if feedback_cpus else None
    manager = Node(package='controller_manager', executable='ros2_control_node', prefix=feedback_prefix,
                   parameters=manager_parameters, output='screen',
                   additional_env=feedback_env)
    state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='real_robot_state_publisher', parameters=[model], output='screen')
    broadcaster = Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
                       output='screen')
    controllers = Node(package='controller_manager', executable='spawner',
                       arguments=[*profile.controller_names, '--controller-manager-timeout', '30',
                                  *(['--inactive'] if read_only or cold else ['--activate-as-group'])], output='screen')
    first_spawner, second_spawner = (controllers, broadcaster) if cold else (broadcaster, controllers)
    feedback_probe = None

    def spawned(event, _context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason='Controller spawner failed'))]
        if staged:
            return [second_spawner] if event.action is first_spawner else [feedback_probe]
        return []

    cad_calibration = ET.parse(profile.model).getroot().get('cleany_cad_calibration', '')
    guard = Node(package='cleany_robot_interface', executable='control_state_guard', output='screen',
                 prefix=feedback_prefix,
                 additional_env=feedback_env,
                 parameters=[{'controller_names': list(profile.controller_names),
                              'cad_calibration_json': cad_calibration}])
    actions = [state, manager, guard, first_spawner]
    if not staged:
        actions.append(second_spawner)
    if profile.state.camera_pose_config:
        actions.append(Node(package='cleany_robot_interface', executable='calibrated_camera_pose',
                            parameters=[{'config_file': str(profile.state.camera_pose_config)}],
                            output='screen'))
    probe_handlers = []
    if value('with_moveit') == 'true':
        moveit = Path(get_package_share_directory('cleany_moveit_config')) / 'launch/physical_move_group.launch.py'
        moveit_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(moveit)),
            launch_arguments={'robot_description_file': str(profile.model),
                              'controllers_file': str(profile.moveit_controllers),
                              'joint_limits_file': str(profile.joint_limits),
                              'use_rviz': value('use_rviz'),
                              'include_head_camera': 'true', 'use_sim_time': 'false',
                              'enable_depth_octomap': value('enable_depth_octomap'),
                              'allow_trajectory_execution': str(allow_trajectory_execution).lower()}.items())
        if value('staged_moveit') == 'true':
            feedback_probe = ExecuteProcess(
                cmd=[sys.executable, '-m', 'cleany_robot_interface.control_feedback_startup',
                     '--timeout', value('startup_timeout_sec'),
                     *(['--cold-controllers', *profile.controller_names] if cold and not defer_activation else [])],
                name='control_feedback_startup', output='screen')

            def feedback_ready(event, _context):
                if event.returncode == 0:
                    return [moveit_launch]
                return [EmitEvent(event=Shutdown(reason='Controller feedback startup check failed'))]

            probe_handlers.append(RegisterEventHandler(OnProcessExit(
                target_action=feedback_probe, on_exit=feedback_ready)))
        else:
            actions.append(moveit_launch)
    guards = [RegisterEventHandler(OnProcessExit(target_action=node,
              on_exit=[EmitEvent(event=Shutdown(reason='Robot state/control process exited'))]))
              for node in (manager, state, guard)]
    guards += [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=spawned))
               for node in (broadcaster, controllers)]
    return guards + probe_handlers + actions


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
        DeclareLaunchArgument('preload_cold_controllers', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('defer_cold_activation', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('feedback_fastdds_profile', default_value='',
                              description='Fast DDS XML for the controller manager and feedback guard'),
        DeclareLaunchArgument('staged_moveit', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('startup_timeout_sec', default_value='30.0'),
        DeclareLaunchArgument('use_rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_depth_octomap', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('output_root', default_value='~/.local/share/cleany/real_control'),
        OpaqueFunction(function=setup),
    ])
