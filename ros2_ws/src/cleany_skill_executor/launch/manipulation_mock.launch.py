"""Launch the manipulation Action in a mock namespace with a persistent journal."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from cleany_skill_executor.manipulation.store import default_database_path


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='mock'),
        DeclareLaunchArgument('database_path', default_value=default_database_path()),
        DeclareLaunchArgument('scenario', default_value='success'),
        DeclareLaunchArgument('mock_config', default_value=PathJoinSubstitution([
            FindPackageShare('cleany_skill_executor'), 'config', 'manipulation_mock.yaml'])),
        Node(package='cleany_skill_executor', executable='manipulation_server',
             namespace=LaunchConfiguration('namespace'), output='screen',
             parameters=[{'backend': 'mock', 'database_path': LaunchConfiguration('database_path'),
                          'scenario': LaunchConfiguration('scenario'),
                          'mock_config': LaunchConfiguration('mock_config')}]),
    ])
