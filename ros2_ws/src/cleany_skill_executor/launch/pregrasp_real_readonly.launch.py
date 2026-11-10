"""Connected robot observation and MoveIt planning with execution disabled."""
import json
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from cleany_robot_interface.state_profile import prepare_state_profile
from cleany_skill_executor.motion_runtime import motion_launch_arguments


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    root = Path(value('repository_root')).resolve()
    profile_path = Path(value('profile_file')).resolve()
    run = Path(value('output_root')).expanduser() / str(time.time_ns())
    profile = prepare_state_profile(profile_path, root, run)
    config = json.loads(profile_path.read_text())
    bridge_config = Path(config['rgbd_bridge']).expanduser()
    if not bridge_config.is_absolute():
        bridge_config = root / bridge_config

    def include(package, name, arguments):
        path = Path(get_package_share_directory(package)) / 'launch' / name
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                        launch_arguments=arguments.items())

    bridge = Node(package='cleany_robot_interface', executable='rgbd_frame_bridge', output='screen',
                  parameters=[{'config_file': str(bridge_config),
                               'receipt_file': str(run / 'rgbd_tf_receipt.json')}])
    print(f'READ_ONLY_RUN_DIR={run}', flush=True)
    return [
        include('cleany_robot_interface', 'read_only_state.launch.py', {
            'robot_description_file': str(profile.model), 'arm_config_file': str(profile.arm_config),
            'head_config_file': str(profile.head_config), 'head_mapping_file': str(profile.head_mapping)}),
        include('cleany_perception', 'realsense.launch.py', {
            'serial_no': profile.camera_serial,
            'config_file': str(
                Path(get_package_share_directory('cleany_perception'))
                / 'config' / 'realsense_pregrasp.yaml'
            ),
        }),
        RegisterEventHandler(OnProcessExit(target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='RGB-D TF bridge exited'))])),
        bridge,
        include('cleany_skill_executor', 'pregrasp_runtime.launch.py', {
            'repository_root': value('repository_root'),
            'motion_backend': value('motion_backend'), 'motion_runtime': value('motion_runtime'),
            'curobo_python': value('curobo_python'),
            'robot_description_file': str(profile.model), 'include_head_camera': 'true',
            'use_sim_time': 'false', 'use_rviz': value('use_rviz')}),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('repository_root'),
        *motion_launch_arguments(),
        DeclareLaunchArgument('motion_backend', default_value='moveit', choices=['moveit', 'curobo_ik_moveit']),
        DeclareLaunchArgument('profile_file'),
        DeclareLaunchArgument('output_root', default_value='~/.local/share/cleany/real_readonly'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        OpaqueFunction(function=setup),
    ])
