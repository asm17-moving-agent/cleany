from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_file(package: str, filename: str) -> str:
    return str(
        Path(get_package_share_directory(package)) / 'launch' / filename
    )


def generate_launch_description() -> LaunchDescription:
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    mujoco_share = Path(get_package_share_directory('cleany_mujoco_sim'))
    skill_share = Path(get_package_share_directory('cleany_skill_executor'))
    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('cleany_mujoco_sim', 'handeye_backend.launch.py')
        ),
        launch_arguments={
            'scene_path': str(
                mujoco_share / 'scenes' / 'grasp_execution_demo.xml.in'
            ),
            'headless': headless,
            'sim_speed_factor': '1.0',
            'enable_gripper_controllers': 'true',
        }.items(),
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('cleany_moveit_config', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_rviz': use_rviz,
            'allow_trajectory_execution': 'true',
        }.items(),
    )
    selector = Node(
        package='cleany_skill_executor',
        executable='grasp_selection_server',
        parameters=[
            str(skill_share / 'config' / 'grasp_selection.yaml'),
            {'use_sim_time': True},
        ],
        output='screen',
    )
    coordinator = Node(
        package='cleany_skill_executor',
        executable='nearest_pregrasp_coordinator',
        parameters=[
            str(skill_share / 'config' / 'nearest_pregrasp.yaml'),
            {'use_sim_time': True},
        ],
        output='screen',
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('headless', default_value='true'),
            DeclareLaunchArgument('use_rviz', default_value='false'),
            backend,
            move_group,
            selector,
            coordinator,
        ]
    )
