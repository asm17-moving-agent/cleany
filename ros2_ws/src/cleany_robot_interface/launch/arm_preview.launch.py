"""Use the common CAD preview without opening the servo bus."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    source = Path(get_package_share_directory('cleany_description'))/'launch/description.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(source)),
            launch_arguments={'namespace': 'arm_preview',
                              'use_rviz': LaunchConfiguration('rviz')}.items())])
