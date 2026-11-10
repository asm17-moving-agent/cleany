"""기존 이름을 유지하며 공통 navigation.launch.py로 위임합니다."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch = Path(get_package_share_directory('cleany_navigation'))/'launch/navigation.launch.py'
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch)))])
