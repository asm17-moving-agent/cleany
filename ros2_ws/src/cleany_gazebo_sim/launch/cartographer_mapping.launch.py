from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
    config_dir = package_share / 'config'

    configuration_arg = DeclareLaunchArgument(
        'configuration_basename', default_value='cartographer_2d.lua'
    )
    odom_tf = Node(
        package='cleany_gazebo_sim',
        executable='gazebo_odom_tf_publisher',
        name='replay_odom_tf_publisher',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/odom',
            'publish_odometry': False,
        }],
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
        parameters=[{'use_sim_time': True}],
        remappings=[('scan', '/scan'), ('imu', '/imu/data')],
        output='screen',
    )
    occupancy = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        arguments=['-resolution', '0.05', '-publish_period_sec', '2.0'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    return LaunchDescription(
        [configuration_arg, odom_tf, cartographer, occupancy]
    )
