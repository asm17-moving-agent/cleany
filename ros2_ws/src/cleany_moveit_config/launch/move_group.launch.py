"""Use the common calibrated CAD MoveIt launch for standalone planning."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    source = Path(get_package_share_directory('cleany_moveit_config'))/'launch/physical_move_group.launch.py'
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(source)))])
