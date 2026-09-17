"""Replay slam_toolbox with the same loop settings used by live mapping."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    navigation_share = Path(
        get_package_share_directory('cleany_navigation')
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / 'launch' / 'slam_mapping.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
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
    return LaunchDescription([odom_tf, slam])
