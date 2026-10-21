"""Attach the idle unified Action server to an existing Jetson control runtime."""
from pathlib import Path
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_server(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    root = Path(value('repository_root')).expanduser()
    config_path = Path(value('collection_config')).expanduser()
    if not config_path.is_file():
        raise ValueError('collection_config must name an explicit local hardware profile')
    monitor_path = Path(value('hardware_monitor_config')).expanduser()
    if not monitor_path.is_file():
        raise ValueError('hardware_monitor_config must name explicit local motion limits')
    config = str(config_path.resolve())
    shutdown_timeout = float(value('shutdown_timeout_sec'))
    if not math.isfinite(shutdown_timeout) or shutdown_timeout <= 0:
        raise ValueError('shutdown_timeout_sec must be finite and positive')
    return [Node(package='cleany_manipulation_bt', executable='manipulation_jetson_server',
        namespace=value('namespace'), output='screen', sigterm_timeout=str(shutdown_timeout), parameters=[
            str(Path(get_package_share_directory('cleany_manipulation_bt')) / 'config/manipulation_jetson.yaml'),
            str(Path(value('hardware_monitor_config')).expanduser().resolve()),
            {'collection_config': config, 'database_path': value('database_path'),
             'arm_return_pose_file': str(root / 'configs/robot/arm_return_home.json'),
             'gripper_contact_rule_file': str(root / 'configs/robot/gripper_contact_decision.yaml'),
             'monitor_port': int(value('monitor_port'))}],
        arguments=['--ros-args', '--log-level', value('log_level')])]


def generate_launch_description():
    state = Path(os.environ.get('XDG_STATE_HOME') or Path.home() / '.local/state')
    return LaunchDescription([
        DeclareLaunchArgument('repository_root', description='Cleany checkout containing physical configuration'),
        DeclareLaunchArgument('collection_config', description='Explicit local hardware profile; no shared experimental default'),
        DeclareLaunchArgument('hardware_monitor_config', description='Explicit local motion-monitor parameters'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('database_path', default_value=str(state / 'cleany/manipulation_jetson/executions.sqlite3')),
        DeclareLaunchArgument('monitor_port', default_value='1667'),
        DeclareLaunchArgument('shutdown_timeout_sec', default_value='25.0',
                              description='Grace period for stop assessment and Action result delivery'),
        DeclareLaunchArgument('log_level', default_value='info', choices=['debug', 'info', 'warn', 'error', 'fatal']),
        OpaqueFunction(function=create_server),
    ])
