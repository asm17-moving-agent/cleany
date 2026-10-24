from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    navigation_share = Path(
        get_package_share_directory('cleany_navigation')
    )
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
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                navigation_share
                / 'launch'
                / 'cartographer_mapping.launch.py'
            )
        ),
        launch_arguments={
            'configuration_basename': LaunchConfiguration(
                'configuration_basename'
            ),
            'use_sim_time': 'true',
        }.items(),
    )
    return LaunchDescription([configuration_arg, odom_tf, cartographer])
