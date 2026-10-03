"""Compatibility launch using the common AMCL/Nav2 implementation."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('cleany_gazebo_sim'))
    navigation = Path(get_package_share_directory('cleany_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(share/'maps/study_cafe_26cm.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(share/'config/nav2_amcl.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(navigation/'launch/amcl_nav2.launch.py')),
            launch_arguments={name: LaunchConfiguration(name)
                              for name in ('map', 'params_file', 'use_sim_time', 'autostart')}.items()),
    ])
