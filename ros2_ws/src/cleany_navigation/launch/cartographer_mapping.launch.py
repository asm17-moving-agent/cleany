from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('cleany_navigation'))
    config_dir = package_share / 'config' / 'slam'

    configuration_arg = DeclareLaunchArgument(
        'configuration_basename', default_value='cartographer_2d.lua'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation or rosbag time from /clock.',
    )
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        arguments=[
            '-configuration_directory', str(config_dir),
            '-configuration_basename',
            LaunchConfiguration('configuration_basename'),
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('scan', '/scan'), ('imu', '/imu/data')],
        output='screen',
    )
    occupancy = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        arguments=['-resolution', '0.05', '-publish_period_sec', '2.0'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )
    return LaunchDescription(
        [configuration_arg, use_sim_time_arg, cartographer, occupancy]
    )
