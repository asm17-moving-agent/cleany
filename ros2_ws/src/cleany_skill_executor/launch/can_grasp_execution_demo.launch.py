from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    grasp_settle_sec = LaunchConfiguration('grasp_settle_sec')
    preclose_hold_sec = LaunchConfiguration('preclose_hold_sec')
    lift_hold_sec = LaunchConfiguration('lift_hold_sec')
    gripper_close_position_rad = LaunchConfiguration(
        'gripper_close_position_rad'
    )
    gripper_force_full_close = LaunchConfiguration('gripper_force_full_close')
    gripper_close_opening_reduction_m = LaunchConfiguration(
        'gripper_close_opening_reduction_m'
    )
    mujoco_share = Path(get_package_share_directory('cleany_mujoco_sim'))
    moveit_share = Path(get_package_share_directory('cleany_moveit_config'))
    skill_share = Path(get_package_share_directory('cleany_skill_executor'))
    grasping_share = Path(get_package_share_directory('cleany_grasping'))
    scene_path = LaunchConfiguration('scene_path')
    target_label = LaunchConfiguration('target_label')
    target_width_m = LaunchConfiguration('target_width_m')
    target_depth_m = LaunchConfiguration('target_depth_m')
    target_height_m = LaunchConfiguration('target_height_m')
    grasp_approach_offset_m = LaunchConfiguration('grasp_approach_offset_m')
    grasp_lateral_offset_m = LaunchConfiguration('grasp_lateral_offset_m')
    grasp_closing_tolerance_deg = LaunchConfiguration(
        'grasp_closing_tolerance_deg'
    )
    geometric_collision_clearance_m = LaunchConfiguration(
        'geometric_collision_clearance_m'
    )
    geometric_yaw_offsets_degrees = LaunchConfiguration(
        'geometric_yaw_offsets_degrees'
    )
    geometric_approach_tilt_degrees = LaunchConfiguration(
        'geometric_approach_tilt_degrees'
    )
    geometric_finger_thickness_m = LaunchConfiguration(
        'geometric_finger_thickness_m'
    )
    geometric_finger_length_m = LaunchConfiguration(
        'geometric_finger_length_m'
    )
    geometric_palm_depth_m = LaunchConfiguration('geometric_palm_depth_m')

    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('cleany_mujoco_sim', 'handeye_backend.launch.py')
        ),
        launch_arguments={
            'scene_path': scene_path,
            'controller_config': str(
                mujoco_share / 'config' / 'grasp_demo_ros2_controllers.yaml'
            ),
            'headless': headless,
            'sim_speed_factor': '1.0',
            'camera_name': 'pick_demo_rgbd',
            'camera_frame_name': 'pick_demo_rgbd_optical_frame',
            'enable_camera_contract_adapter': 'false',
            'enable_gripper_controllers': 'true',
            # Folded, collision-checked can-demo spawn pose. Keep a small
            # margin from every mechanical joint limit.
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
            'use_rviz': use_rviz,
            'allow_trajectory_execution': 'true',
            'allowed_execution_duration_scaling': '2.0',
            'allowed_goal_duration_margin': '1.0',
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
            {
                'use_sim_time': True,
                'geometric.approach_tilt_degrees': ParameterValue(
                    geometric_approach_tilt_degrees, value_type=float
                ),
                'geometric.approach_tilt_direction': [1.0, 0.0, 0.0],
                'geometric.collision_clearance_m': ParameterValue(
                    geometric_collision_clearance_m, value_type=float
                ),
                'geometric.yaw_offsets_degrees': ParameterValue(
                    geometric_yaw_offsets_degrees, value_type=list[float]
                ),
                'geometric.finger_thickness_m': ParameterValue(
                    geometric_finger_thickness_m, value_type=float
                ),
                'geometric.finger_length_m': ParameterValue(
                    geometric_finger_length_m, value_type=float
                ),
                'geometric.palm_depth_m': ParameterValue(
                    geometric_palm_depth_m, value_type=float
                ),
            },
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
                'pregrasp_closing_tolerance_deg': 15.0,
                'grasp_closing_tolerance_deg': ParameterValue(
                    grasp_closing_tolerance_deg, value_type=float
                ),
                'grasp_closing_sign_invariant': False,
                'grasp_approach_offset_m': ParameterValue(
                    grasp_approach_offset_m, value_type=float
                ),
                'grasp_lateral_offset_m': ParameterValue(
                    grasp_lateral_offset_m, value_type=float
                ),
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
            'grasp_settle_sec': ParameterValue(
                grasp_settle_sec, value_type=float
            ),
            'preclose_hold_sec': ParameterValue(
                preclose_hold_sec, value_type=float
            ),
            'lift_hold_sec': ParameterValue(lift_hold_sec, value_type=float),
            'gripper_close_position_rad': ParameterValue(
                gripper_close_position_rad, value_type=float
            ),
            'gripper_force_full_close': ParameterValue(
                gripper_force_full_close, value_type=bool
            ),
            'gripper_close_opening_reduction_m': ParameterValue(
                gripper_close_opening_reduction_m, value_type=float
            ),
            'planning_attempts': 3,
            'target_label': target_label,
            'can_diameter_m': ParameterValue(
                target_width_m, value_type=float
            ),
            'target_depth_m': ParameterValue(
                target_depth_m, value_type=float
            ),
            'can_height_m': ParameterValue(
                target_height_m, value_type=float
            ),
            'grasp_approach_execution_offset_m': ParameterValue(
                grasp_approach_offset_m, value_type=float
            ),
            'grasp_lateral_execution_offset_m': ParameterValue(
                grasp_lateral_offset_m, value_type=float
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
        DeclareLaunchArgument('grasp_settle_sec', default_value='1.0'),
        DeclareLaunchArgument('preclose_hold_sec', default_value='0.0'),
        DeclareLaunchArgument('lift_hold_sec', default_value='3.0'),
        DeclareLaunchArgument(
            'gripper_close_position_rad', default_value='0.30'
        ),
        DeclareLaunchArgument(
            'gripper_force_full_close', default_value='false'
        ),
        DeclareLaunchArgument(
            'gripper_close_opening_reduction_m', default_value='0.010'
        ),
        DeclareLaunchArgument(
            'scene_path',
            default_value=str(
                mujoco_share / 'scenes' / 'can_grasp_execution_demo.xml.in'
            ),
        ),
        DeclareLaunchArgument('target_label', default_value='can'),
        DeclareLaunchArgument('target_width_m', default_value='0.070'),
        DeclareLaunchArgument('target_depth_m', default_value='0.070'),
        DeclareLaunchArgument('target_height_m', default_value='0.100'),
        DeclareLaunchArgument(
            'grasp_approach_offset_m', default_value='0.010'
        ),
        DeclareLaunchArgument(
            'grasp_lateral_offset_m',
            # Candidates can close across either OBB horizontal extent.  Use
            # the conservative extent so the fixed jaw never starts inside a
            # rotated box.  Cleany's TCP is 8 mm inside that jaw contact face.
            default_value=PythonExpression(
                [
                    'max(',
                    target_width_m,
                    ', ',
                    target_depth_m,
                    ') / 2.0 - 0.008',
                ]
            ),
        ),
        DeclareLaunchArgument(
            'grasp_closing_tolerance_deg', default_value='5.0'
        ),
        DeclareLaunchArgument(
            'geometric_collision_clearance_m', default_value='0.012'
        ),
        DeclareLaunchArgument(
            'geometric_yaw_offsets_degrees',
            default_value='[-20.0, -10.0, 0.0, 10.0, 20.0]',
        ),
        DeclareLaunchArgument(
            'geometric_approach_tilt_degrees', default_value='16.0'
        ),
        DeclareLaunchArgument(
            'geometric_finger_thickness_m', default_value='0.010'
        ),
        DeclareLaunchArgument(
            'geometric_finger_length_m', default_value='0.045'
        ),
        DeclareLaunchArgument(
            'geometric_palm_depth_m', default_value='0.018'
        ),
        backend,
        move_group,
        collision_scene,
        grasp_server,
        selector,
        demo,
        image_view,
    ])
