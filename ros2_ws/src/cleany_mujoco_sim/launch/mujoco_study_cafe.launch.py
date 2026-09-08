from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare('cleany_mujoco_sim')
    headless = LaunchConfiguration('headless')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'headless',
                default_value='false',
                description='Run without the MuJoCo viewer.',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [package_share, 'launch', 'mujoco_sim.launch.py']
                    )
                ),
                launch_arguments={
                    'scene_path': PathJoinSubstitution(
                        [package_share, 'scenes', 'study_cafe.xml.in']
                    ),
                    'headless': headless,
                }.items(),
            ),
        ]
    )
