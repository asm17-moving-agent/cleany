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
            DeclareLaunchArgument('viewer_rate_hz', default_value='20.0'),
            DeclareLaunchArgument('viewer_shadows', default_value='false'),
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
                    'viewer_rate_hz': LaunchConfiguration('viewer_rate_hz'),
                    'viewer_shadows': LaunchConfiguration('viewer_shadows'),
                }.items(),
            ),
        ]
    )
