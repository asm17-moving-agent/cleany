from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('cleany_base_odometry'))
    default_config = package_share / 'config' / 'wheel_odometry.yaml'

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=str(default_config),
        description='Wheel odometry parameter file.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false'
    )
    wheel_odometry = Node(
        package='cleany_base_odometry',
        executable='wheel_odometry_node',
        name='wheel_odometry',
        parameters=[
            LaunchConfiguration('config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )
    return LaunchDescription(
        [config_arg, use_sim_time_arg, wheel_odometry]
    )
