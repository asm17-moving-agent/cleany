from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from cleany_skill_executor.motion_runtime import motion_launch_arguments


def _config(package: str, name: str) -> str:
    return str(Path(get_package_share_directory(package)) / 'config' / name)


def generate_launch_description() -> LaunchDescription:
    """Start the plan-only pipeline around an already running MoveIt/RGB-D stack."""
    evidence_output_directory = LaunchConfiguration('evidence_output_directory')
    return LaunchDescription(
        [
            DeclareLaunchArgument('repository_root', default_value=str(Path(__file__).resolve().parents[4])),
            *motion_launch_arguments(),
            DeclareLaunchArgument('evidence_output_directory', default_value=''),
            DeclareLaunchArgument('start_perception', default_value='true'),
            DeclareLaunchArgument('start_grasping', default_value='true'),
            DeclareLaunchArgument('motion_backend', default_value='moveit'),
            Node(
                package='cleany_perception',
                executable='inspection_node',
                parameters=[_config('cleany_perception', 'inspect_scene.yaml')],
                output='screen',
                condition=IfCondition(
                    LaunchConfiguration('start_perception')
                ),
            ),
            Node(
                package='cleany_grasping',
                executable='grasp_server',
                parameters=[_config('cleany_grasping', 'anygrasp.yaml')],
                output='screen',
                condition=IfCondition(
                    LaunchConfiguration('start_grasping')
                ),
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory('cleany_skill_executor')) / 'launch/grasp_selection.launch.py')),
                launch_arguments={
                    'repository_root': LaunchConfiguration('repository_root'),
                    'motion_backend': LaunchConfiguration('motion_backend'),
                    'motion_runtime': LaunchConfiguration('motion_runtime'),
                    'curobo_python': LaunchConfiguration('curobo_python')}.items()),
            Node(
                package='cleany_skill_executor',
                executable='validate_pick_server',
                parameters=[
                    _config('cleany_skill_executor', 'validate_pick.yaml'),
                    {'evidence_output_directory': evidence_output_directory},
                ],
                output='screen',
            ),
        ]
    )
