from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory('cleany_gazebo_sim'))
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / 'launch' / 'slam_mapping.launch.py')
        ),
        launch_arguments={
            'do_loop_closing': 'true',
            'loop_search_maximum_distance': '2.0',
            'loop_search_space_dimension': '4.0',
            'loop_match_minimum_response_coarse': '0.50',
            'loop_match_minimum_response_fine': '0.60',
        }.items(),
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
