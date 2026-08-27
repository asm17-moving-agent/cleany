from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _launch_file(package: str, filename: str) -> str:
    return str(Path(get_package_share_directory(package)) / 'launch' / filename)


def generate_launch_description() -> LaunchDescription:
    mujoco_share = Path(get_package_share_directory('cleany_mujoco_sim'))
    moveit_share = Path(get_package_share_directory('cleany_moveit_config'))
    skill_share = Path(get_package_share_directory('cleany_skill_executor'))
    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('cleany_mujoco_sim', 'handeye_backend.launch.py')
        ),
        launch_arguments={
            'scene_path': str(
                mujoco_share / 'scenes' / 'can_grasp_execution_demo.xml.in'
            ),
            'controller_config': str(
                mujoco_share / 'config' / 'grasp_demo_ros2_controllers.yaml'
            ),
            'headless': 'true',
            'sim_speed_factor': '4.0',
            'camera_publish_rate': '0.0',
            'enable_camera_contract_adapter': 'false',
            'enable_gripper_controllers': 'true',
            'left_shoulder_yaw_initial': '-1.53',
            'left_shoulder_pitch_initial': '3.35',
            'left_elbow_pitch_initial': '3.12',
            'left_wrist_pitch_initial': '-1.63',
            'left_wrist_roll_initial': '1.58',
            'left_gripper_initial': '-0.35',
            'right_shoulder_yaw_initial': '1.58',
            'right_shoulder_pitch_initial': '3.35',
            'right_elbow_pitch_initial': '3.12',
            'right_wrist_pitch_initial': '-1.63',
            'right_wrist_roll_initial': '1.58',
            'right_gripper_initial': '-0.35',
        }.items(),
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('cleany_moveit_config', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_rviz': 'false',
            'allow_trajectory_execution': 'true',
        }.items(),
    )
    collision_scene = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file(
                'cleany_moveit_config',
                'handeye_collision_scene.launch.py',
            )
        ),
        launch_arguments={
            'scene_config': str(
                moveit_share / 'config' / 'pick_demo_collision_objects.yaml'
            )
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
    return LaunchDescription([backend, move_group, collision_scene, selector])
