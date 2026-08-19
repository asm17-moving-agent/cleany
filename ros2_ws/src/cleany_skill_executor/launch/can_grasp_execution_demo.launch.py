from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_file(package: str, filename: str) -> str:
    return str(Path(get_package_share_directory(package)) / 'launch' / filename)


def generate_launch_description() -> LaunchDescription:
    pregrasp_offset_m = 0.14
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    use_grasp_image_view = LaunchConfiguration('use_grasp_image_view')
    demo_start_delay_sec = LaunchConfiguration('demo_start_delay_sec')
    stage_hold_sec = LaunchConfiguration('stage_hold_sec')
    mujoco_share = Path(get_package_share_directory('cleany_mujoco_sim'))
    moveit_share = Path(get_package_share_directory('cleany_moveit_config'))
    skill_share = Path(get_package_share_directory('cleany_skill_executor'))
    grasping_share = Path(get_package_share_directory('cleany_grasping'))

    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('cleany_mujoco_sim', 'handeye_backend.launch.py')
        ),
        launch_arguments={
            'scene_path': str(
                mujoco_share / 'scenes' / 'can_grasp_execution_demo.xml.in'
            ),
            'headless': headless,
            'sim_speed_factor': '1.0',
            'camera_name': 'pick_demo_rgbd',
            'camera_frame_name': 'pick_demo_rgbd_optical_frame',
            'enable_camera_contract_adapter': 'false',
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
    grasp_server = Node(
        package='cleany_grasping',
        executable='grasp_server',
        parameters=[
            str(grasping_share / 'config' / 'anygrasp.yaml'),
            {'use_sim_time': True},
        ],
        output='screen',
    )
    selector = Node(
        package='cleany_skill_executor',
        executable='grasp_selection_server',
        parameters=[
            str(skill_share / 'config' / 'grasp_selection.yaml'),
            {
                'use_sim_time': True,
                'pregrasp_offset_m': pregrasp_offset_m,
            },
        ],
        output='screen',
    )
    demo = Node(
        package='cleany_skill_executor',
        executable='can_grasp_execution_demo',
        parameters=[{
            'use_sim_time': True,
            'demo_start_delay_sec': ParameterValue(
                demo_start_delay_sec, value_type=float
            ),
            'stage_hold_sec': ParameterValue(
                stage_hold_sec, value_type=float
            ),
        }],
        output='screen',
    )
    image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/grasp/can_grasp_image'],
        condition=IfCondition(use_grasp_image_view),
        output='log',
    )
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_grasp_image_view', default_value='true'),
        DeclareLaunchArgument('demo_start_delay_sec', default_value='5.0'),
        DeclareLaunchArgument('stage_hold_sec', default_value='3.0'),
        backend,
        move_group,
        collision_scene,
        grasp_server,
        selector,
        demo,
        image_view,
    ])
