from pathlib import Path
import math
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    OpaqueFunction,
    RegisterEventHandler,
    EmitEvent,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from cleany_perception.model_runtime import (
    load_model_profile, resolve_model_assets,
)


def _launch_file(package: str, filename: str) -> str:
    return str(
        Path(get_package_share_directory(package)) / 'launch' / filename
    )


def _wrist_camera_transforms(context):
    if LaunchConfiguration('sorting_use_wrist_camera').perform(context) != 'true':
        return []
    if (LaunchConfiguration('sorting_mode').perform(context) != 'true'
            or LaunchConfiguration('start_simulator').perform(context) != 'true'):
        raise RuntimeError('Nominal wrist mounts are for simulation sorting only')
    path = Path(LaunchConfiguration('sorting_wrist_cameras_config').perform(context))
    config = yaml.safe_load(path.read_text())
    if config.get('source') != 'simulation_nominal_cad':
        raise ValueError('Expected explicit simulation nominal wrist configuration')
    nodes = []
    for arm in ('left', 'right'):
        mount = config[arm]
        xyz, quaternion = mount['translation_m'], mount['quaternion_xyzw']
        if (len(xyz) != 3 or len(quaternion) != 4
                or not all(math.isfinite(v) for v in [*xyz, *quaternion])
                or abs(sum(v*v for v in quaternion)-1.0) > 1e-6
                or mount['parent_frame'] != f'{arm}_gripper_frame'
                or mount['child_frame'] != f'{arm}_wrist_rgb_optical_frame'):
            raise ValueError(f'Invalid {arm} wrist mounting transform')
        arguments = []
        for key, value in zip(('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'), [*xyz, *quaternion]):
            arguments.extend((f'--{key}', str(value)))
        arguments.extend(('--frame-id', mount['parent_frame'], '--child-frame-id', mount['child_frame']))
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
            name=f'{arm}_nominal_wrist_tf', arguments=arguments,
            parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
            output='log'))
    return nodes


def _preflight(context):
    def value(key):
        return LaunchConfiguration(key).perform(context)

    if value('start_simulator') == 'false' and value('plan_only') != 'true':
        raise RuntimeError('External robot mode is currently plan-only')
    if value('start_simulator') == 'false' and value('sensor_scene') != 'true':
        raise RuntimeError('External robot mode requires sensor_scene=true')
    if value('start_simulator') == 'true' and value('use_sim_time') != 'true':
        raise RuntimeError('MuJoCo requires use_sim_time=true')
    if value('sorting_mode') == 'true':
        if value('start_simulator') != 'true' or value('plan_only') != 'false':
            raise RuntimeError('Sorting requires simulation with execution')
        if not Path(value('sorting_bins_config')).is_file():
            raise RuntimeError('Sorting requires collection bin configuration')
    if value('sorting_use_wrist_camera') == 'true' and value('sorting_mode') != 'true':
        raise RuntimeError('Wrist switching currently requires the simulation sorting backend')
    if value('sorting_async_carry_monitor') == 'true' and (
            value('sorting_use_wrist_camera') != 'true'
            or value('wrist_continuous_tracking') != 'true'):
        raise RuntimeError('Asynchronous carry monitoring requires wrist camera and continuous tracking')
    if value('sensor_scene') == 'true':
        get_package_share_directory('moveit_ros_perception')
    if value('fastdds_profiles_file') and not Path(value('fastdds_profiles_file')).is_file():
        raise RuntimeError('Fast DDS profiles file does not exist')
    if value('start_perception') == 'false':
        # External perception owns its model files, credentials and CUDA runtime.
        # Geometry / simulation validation above still applies to the host.
        return []
    for argument, kind in (('perception_detector_type', 'yoloe'),
                           ('perception_segmenter_type', 'sam2')):
        if value(argument) == kind:
            keys = (
                ('yoloe_model_path', 'yoloe_text_encoder_directory')
                if kind == 'yoloe'
                else ('sam2_checkpoint', 'sam2_model_config')
            )
            resolve_model_assets(
                {key: value(key) for key in keys},
                value('model_directory'), kind,
            )
    if value('perception_detector_type') == 'gemini':
        if not value('gemini_model').strip():
            raise RuntimeError('Gemini model must not be empty')
        key_name = value('gemini_api_key_environment')
        if not context.environment.get(key_name, '').strip():
            raise RuntimeError(f'{key_name} must be set before launching the Gemini pipeline')
    return []


def generate_launch_description() -> LaunchDescription:
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    use_image_view = LaunchConfiguration('use_image_view')
    sensor_scene = LaunchConfiguration('sensor_scene')
    plan_only = LaunchConfiguration('plan_only')
    start_simulator = LaunchConfiguration('start_simulator')
    sorting_mode = LaunchConfiguration('sorting_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    clock_parameter = ParameterValue(use_sim_time, value_type=bool)
    grasp_approach_offset = ParameterValue(
        LaunchConfiguration('grasp_approach_offset_m'), value_type=float
    )
    perception_detector_type = LaunchConfiguration(
        'perception_detector_type'
    )
    perception_segmenter_type = LaunchConfiguration(
        'perception_segmenter_type'
    )
    perception_minimum_detection_confidence = LaunchConfiguration(
        'perception_minimum_detection_confidence'
    )
    yoloe_model_path = LaunchConfiguration('yoloe_model_path')
    yoloe_classes = LaunchConfiguration('yoloe_classes')
    yoloe_text_encoder_directory = LaunchConfiguration(
        'yoloe_text_encoder_directory'
    )
    perception_device = LaunchConfiguration('perception_device')
    sam2_model_config = LaunchConfiguration('sam2_model_config')
    sam2_checkpoint = LaunchConfiguration('sam2_checkpoint')
    mujoco_share = Path(get_package_share_directory('cleany_mujoco_sim'))
    moveit_share = Path(get_package_share_directory('cleany_moveit_config'))
    perception_share = Path(get_package_share_directory('cleany_perception'))
    grasping_share = Path(get_package_share_directory('cleany_grasping'))
    skill_share = Path(get_package_share_directory('cleany_skill_executor'))
    profile_path = perception_share / 'config' / 'yoloe_s_sam2_tiny.yaml'
    profile = load_model_profile(profile_path)
    # Keep YOLOE asset defaults only for an explicit detector override; Gemini
    # does not load them or silently fall back to a local detector.
    profile.update(load_model_profile(
        perception_share / 'config' / 'gemini_flash_lite_sam2_tiny.yaml'))

    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('cleany_mujoco_sim', 'handeye_backend.launch.py')
        ),
        condition=IfCondition(start_simulator),
        launch_arguments={
            'sorting_bins_config': LaunchConfiguration('sorting_bins_config'),
            'sorting_contact_diagnostics': LaunchConfiguration('sorting_contact_diagnostics'),
            'scheduled_cameras': LaunchConfiguration('sorting_use_wrist_camera'),
            'color_image_topic': LaunchConfiguration('color_image_topic'),
            'camera_info_topic': LaunchConfiguration('color_info_topic'),
            'depth_image_topic': LaunchConfiguration('depth_image_topic'),
            'scene_path': str(
                mujoco_share
                / 'scenes'
                / 'study_cafe_grasp_execution.xml.in'
            ),
            'controller_config': str(
                mujoco_share / 'config' / 'grasp_demo_ros2_controllers.yaml'
            ),
            'headless': headless,
            'sim_speed_factor': LaunchConfiguration('sim_speed_factor'),
            'camera_name': 'head_realsense_rgb',
            'camera_frame_name': 'head_camera_rgb_optical_frame',
            'enable_camera_contract_adapter': 'false',
            'enable_gripper_controllers': 'true',
            'head_tilt_initial': '1.0',
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
            'right_wrist_roll_initial': '-1.58',
            'right_gripper_initial': '-0.35',
        }.items(),
    )
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.140751687191',
            '--y', '-0.002000000139',
            '--z', '0.766322294556',
            '--qx', '-0.678504049029',
            '--qy', '0.678504051465',
            '--qz', '-0.199078512000',
            '--qw', '0.199078511286',
            '--frame-id', 'base_link',
            '--child-frame-id', 'head_camera_rgb_optical_frame',
        ],
        condition=IfCondition(start_simulator),
        parameters=[{'use_sim_time': clock_parameter}],
        output='screen',
    )
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _launch_file('cleany_moveit_config', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            'enable_gripper_execution': sorting_mode,
            'allow_trajectory_execution': PythonExpression([
                "'false' if '", plan_only, "' == 'true' else 'true'",
            ]),
            'enable_depth_octomap': sensor_scene,
            'depth_octomap_plugin': LaunchConfiguration('depth_octomap_plugin'),
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
        condition=UnlessCondition(sensor_scene),
        launch_arguments={
            'scene_config': str(
                moveit_share
                / 'config'
                / 'study_cafe_grasp_collision_objects.yaml'
            )
        }.items(),
    )
    depth_scene = Node(
        package='cleany_perception',
        executable='depth_scene_node',
        condition=IfCondition(sensor_scene),
        parameters=[
            str(perception_share / 'config' / 'depth_scene.yaml'),
            {
                'use_sim_time': clock_parameter,
                'depth_image_topic': LaunchConfiguration('depth_image_topic'),
                'depth_info_topic': LaunchConfiguration('depth_info_topic'),
            },
        ],
        output='screen',
    )
    perception = Node(
        package='cleany_perception',
        executable='inspection_node',
        condition=IfCondition(LaunchConfiguration('start_perception')),
        parameters=[
            str(perception_share / 'config' / 'inspect_scene.yaml'),
            profile,
            {
                'use_sim_time': clock_parameter,
                'model_directory': LaunchConfiguration('model_directory'),
                'preload_models': ParameterValue(
                    LaunchConfiguration('preload_models'), value_type=bool
                ),
                'detector_type': perception_detector_type,
                'gemini_model': LaunchConfiguration('gemini_model'),
                'gemini_api_key_environment': LaunchConfiguration('gemini_api_key_environment'),
                'enable_wrist_observation': ParameterValue(LaunchConfiguration('sorting_use_wrist_camera'), value_type=bool),
                'enable_reference_observation': ParameterValue(PythonExpression([
                    "'", LaunchConfiguration('sorting_use_wrist_camera'), "' == 'true' or '",
                    LaunchConfiguration('sorting_use_reference_observation'), "' == 'true'",
                ]), value_type=bool),
                'wrist_continuous_tracking': ParameterValue(LaunchConfiguration('wrist_continuous_tracking'), value_type=bool),
                'segmenter_type': perception_segmenter_type,
                'simulation_color_profile': 'study_cafe',
                'yoloe_model_path': yoloe_model_path,
                'yoloe_classes': ParameterValue(
                    yoloe_classes,
                    value_type=list[str],
                ),
                'yoloe_device': perception_device,
                'yoloe_image_size': ParameterValue(
                    LaunchConfiguration('yoloe_image_size'), value_type=int
                ),
                'yoloe_text_encoder_directory': (
                    yoloe_text_encoder_directory
                ),
                'sam2_model_config': sam2_model_config,
                'sam2_checkpoint': sam2_checkpoint,
                'sam2_device': perception_device,
                'minimum_detection_confidence': ParameterValue(
                    perception_minimum_detection_confidence,
                    value_type=float,
                ),
                'color_image_topic': LaunchConfiguration('color_image_topic'),
                'color_info_topic': LaunchConfiguration('color_info_topic'),
                'depth_image_topic': LaunchConfiguration('depth_image_topic'),
                'depth_info_topic': LaunchConfiguration('depth_info_topic'),
                'target_frame': 'base_link',
                'snapshot_timeout_seconds': 10.0,
            },
        ],
        output='screen',
    )
    grasp_server = Node(
        package='cleany_grasping',
        executable='grasp_server',
        parameters=[
            str(grasping_share / 'config' / 'anygrasp.yaml'),
            {
                'use_sim_time': clock_parameter,
                'geometric.approach_tilt_degrees': 16.0,
                'geometric.maximum_top_contact_depth_m': ParameterValue(
                    LaunchConfiguration('grasp_maximum_top_contact_depth_m'), value_type=float),
                'publish_collision_geometry': ParameterValue(sorting_mode, value_type=bool),
                'geometric.search_longitudinal_contacts': ParameterValue(sorting_mode, value_type=bool),
                'geometric.longitudinal_contact_height_offset_m': ParameterValue(PythonExpression([
                    "0.003 if '", sorting_mode, "' == 'true' else 0.0"]), value_type=float),
                'geometric.defer_support_plane_collision': ParameterValue(sorting_mode, value_type=bool),
                'geometric.prefer_upward_closing_axis': ParameterValue(
                    LaunchConfiguration('prefer_upward_closing_axis'), value_type=bool),
                # Nominal base_link shoulder origins from cleany_geometry.xacro.
                'geometric.approach_reference_positions': [0.09, 0.1552, 0.4115, 0.09, -0.1552, 0.4115],
                'geometric.search_approach_tilts': ParameterValue(
                    sorting_mode, value_type=bool
                ),
                'geometric.include_reverse_closing_axis': ParameterValue(
                    sorting_mode, value_type=bool
                ),
                'nms_rotation_threshold_degrees': ParameterValue(
                    PythonExpression([
                        "5.0 if '", sorting_mode, "' == 'true' else 20.0",
                    ]), value_type=float,
                ),
                'geometric.approach_tilt_direction': [1.0, 0.0, 0.0],
                'geometric.reject_robot_opposite_approach': True,
                'geometric.robot_reference_position': [0.0, 0.0, 0.0],
                'geometric.yaw_offsets_degrees': [
                    -80.0,
                    -60.0,
                    -40.0,
                    -20.0,
                    0.0,
                    20.0,
                    40.0,
                    60.0,
                    80.0,
                ],
                'geometric.maximum_candidates': ParameterValue(PythonExpression([
                    "96 if '", sorting_mode, "' == 'true' else 24"]), value_type=int),
                'geometric.collision_clearance_m': 0.008,
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
                'use_sim_time': clock_parameter,
                'pregrasp_offset_m': 0.14,
                'pregrasp_position_tolerance_m': 0.020,
                'grasp_position_tolerance_m': ParameterValue(PythonExpression([
                    "0.0015 if '", sorting_mode, "' == 'true' else 0.020"]), value_type=float),
                'pose_refinement_position_weight': ParameterValue(PythonExpression([
                    "10.0 if '", sorting_mode, "' == 'true' else 1.0"]), value_type=float),
                'pregrasp_preferred_approach_tolerance_deg': 8.0,
                'pregrasp_approach_tolerance_deg': 18.0,
                'pregrasp_closing_tolerance_deg': 15.0,
                'grasp_closing_tolerance_deg': 8.0,
                'pregrasp_aim_attempts': 48,
                'pose_refinement_iterations': ParameterValue(PythonExpression([
                    "30 if '", sorting_mode, "' == 'true' else 0"]), value_type=int),
                'grasp_pose_seed_attempts': ParameterValue(
                    PythonExpression([
                        "48 if '", sorting_mode, "' == 'true' else 0",
                    ]), value_type=int,
                ),
                'grasp_closing_sign_invariant': False,
                'align_grasp_wrist_roll': ParameterValue(sorting_mode, value_type=bool),
                'joint_limit_margin_rad': ParameterValue(
                    PythonExpression(["0.005 if '", sorting_mode, "' == 'true' else 0.0"]),
                    value_type=float),
                # Head visibility at pregrasp is only required by the head-only
                # branch; wrist handoff performs its own current-image gate.
                'require_pregrasp_visibility': ParameterValue(PythonExpression([
                    "'", sorting_mode, "' == 'true' and '",
                    LaunchConfiguration('sorting_use_wrist_camera'), "' != 'true'",
                ]), value_type=bool),
                'service_artifact_directory': LaunchConfiguration('sorting_artifact_directory'),
                'maximum_candidates': 24,
                'grasp_approach_offset_m': grasp_approach_offset,
                'grasp_lateral_offset_m': 0.030,
                'grasp_use_aperture_centering': ParameterValue(sorting_mode, value_type=bool),
                'grasp_execution_lateral_offset_m': 0.030,
                'grasp_fixed_jaw_clearance_m': ParameterValue(PythonExpression([
                    "0.003 if '", sorting_mode, "' == 'true' else 0.0"]), value_type=float),
                'require_open_grasp_clearance': ParameterValue(sorting_mode, value_type=bool),
                'require_gripper_closure_clearance': ParameterValue(sorting_mode, value_type=bool),
                'use_observed_collision_geometry': ParameterValue(sorting_mode, value_type=bool),
                'selection_gripper_close_position_rad': -0.30,
                'planning_scene_timeout_sec': ParameterValue(PythonExpression([
                    "5.0 if '", sorting_mode, "' == 'true' else 1.0"]), value_type=float),
                'state_validity_timeout_sec': ParameterValue(PythonExpression([
                    "5.0 if '", sorting_mode, "' == 'true' else 1.0"]), value_type=float),
                'fk_timeout_sec': ParameterValue(PythonExpression([
                    "5.0 if '", sorting_mode, "' == 'true' else 1.0"]), value_type=float),
                'ik_response_margin_sec': ParameterValue(PythonExpression([
                    "5.0 if '", sorting_mode, "' == 'true' else 1.0"]), value_type=float),
                'planning_response_margin_sec': ParameterValue(PythonExpression([
                    "5.0 if '", sorting_mode, "' == 'true' else 1.0"]), value_type=float),
                'support_patch_margin_m': ParameterValue(PythonExpression([
                    "0.02 if '", sorting_mode, "' == 'true' else 0.0"]), value_type=float),
                'selection_gripper_open_position_rad': ParameterValue(
                    LaunchConfiguration('gripper_open_position_rad'), value_type=float),
            },
        ],
        output='screen',
    )
    coordinator = Node(
        package='cleany_skill_executor',
        executable=PythonExpression([
            "'sorting_coordinator' if '", sorting_mode,
            "' == 'true' else 'nearest_pregrasp_coordinator'",
        ]),
        parameters=[
            str(skill_share / 'config' / 'nearest_pregrasp.yaml'),
            {
                'use_sim_time': clock_parameter,
                'sorting_bins_config': LaunchConfiguration('sorting_bins_config'),
                'sorting_artifact_directory': LaunchConfiguration('sorting_artifact_directory'),
                'sorting_test_only_label': LaunchConfiguration('sorting_test_only_label'),
                'sorting_use_wrist_camera': ParameterValue(LaunchConfiguration('sorting_use_wrist_camera'), value_type=bool),
                'sorting_use_reference_observation': ParameterValue(LaunchConfiguration('sorting_use_reference_observation'), value_type=bool),
                'sorting_head_reference_refresh_age_sec': ParameterValue(
                    LaunchConfiguration('sorting_head_reference_refresh_age_sec'), value_type=float),
                'sorting_async_carry_monitor': ParameterValue(LaunchConfiguration('sorting_async_carry_monitor'), value_type=bool),
                'sorting_approach_acceleration_scaling': ParameterValue(LaunchConfiguration('sorting_approach_acceleration_scaling'), value_type=float),
                'sorting_return_gripper_position_rad': ParameterValue(LaunchConfiguration('sorting_return_gripper_position_rad'), value_type=float),
                'velocity_scaling': ParameterValue(LaunchConfiguration('velocity_scaling'), value_type=float),
                'acceleration_scaling': ParameterValue(LaunchConfiguration('acceleration_scaling'), value_type=float),
                'cartesian_translation_speed_m_s': ParameterValue(LaunchConfiguration('cartesian_translation_speed_m_s'), value_type=float),
                'cartesian_translation_acceleration_m_s2': ParameterValue(LaunchConfiguration('cartesian_translation_acceleration_m_s2'), value_type=float),
                'cartesian_rotation_speed_rad_s': ParameterValue(LaunchConfiguration('cartesian_rotation_speed_rad_s'), value_type=float),
                'cartesian_joint_acceleration_rad_s2': ParameterValue(LaunchConfiguration('cartesian_joint_acceleration_rad_s2'), value_type=float),
                'approach_velocity_scaling': ParameterValue(LaunchConfiguration('approach_velocity_scaling'), value_type=float),
                'retreat_velocity_scaling': ParameterValue(LaunchConfiguration('retreat_velocity_scaling'), value_type=float),
                'lin_acceleration_scaling': ParameterValue(LaunchConfiguration('lin_acceleration_scaling'), value_type=float),
                'corridor_time_margin': ParameterValue(LaunchConfiguration('corridor_time_margin'), value_type=float),
                'sorting_payload_velocity_scaling': ParameterValue(LaunchConfiguration('sorting_payload_velocity_scaling'), value_type=float),
                'sorting_payload_acceleration_scaling': ParameterValue(LaunchConfiguration('sorting_payload_acceleration_scaling'), value_type=float),
                'query': profile['default_query'],
                'camera_info_topic': LaunchConfiguration('color_info_topic'),
                'plan_only': ParameterValue(plan_only, value_type=bool),
                'require_sensor_scene': ParameterValue(
                    sensor_scene, value_type=bool
                ),
                'sensor_scene_receipt_topic': '/perception/scene_cloud_receipt',
                'use_joint_corridor_grasp': ParameterValue(sorting_mode, value_type=bool),
                'use_seeded_cartesian_grasp': ParameterValue(sorting_mode, value_type=bool),
                'cartesian_local_refinement_iterations': 40,
                'execute_grasp_and_lift': ParameterValue(PythonExpression([
                    "'", plan_only, "' != 'true'",
                ]), value_type=bool),
                'minimum_candidate_opening_m': 0.0,
                'gripper_force_full_close': ParameterValue(sorting_mode, value_type=bool),
                'direct_vertical_lift': ParameterValue(
                    LaunchConfiguration('direct_vertical_lift'), value_type=bool),
                'attachment_scene_timeout_sec': ParameterValue(PythonExpression([
                    "10.0 if '", sorting_mode, "' == 'true' else 5.0"]), value_type=float),
                'grasp_use_aperture_centering': ParameterValue(sorting_mode, value_type=bool),
                'gripper_aperture_m_per_rad': 0.065,
                'gripper_close_opening_reduction_m': 0.018,
                'gripper_contact_retry_steps': ParameterValue(PythonExpression([
                    "5 if '", sorting_mode, "' == 'true' else 0"]), value_type=int),
                'gripper_wall_timeout_factor': ParameterValue(PythonExpression([
                    "6.0 if '", sorting_mode, "' == 'true' else 2.0"]), value_type=float),
                'cartesian_execution_wall_timeout_factor': ParameterValue(PythonExpression([
                    "10.0 if '", sorting_mode, "' == 'true' else 2.0"]), value_type=float),
                'gripper_motion_sec': ParameterValue(LaunchConfiguration('gripper_motion_sec'), value_type=float),
                'gripper_close_position_rad': -0.30,
                'gripper_open_position_rad': ParameterValue(
                    LaunchConfiguration('gripper_open_position_rad'), value_type=float),
                'gripper_contact_max_velocity_rad_s': 0.20,
                'grasp_contact_stop_max_distance_m': 0.020,
                'selector_pregrasp_offset_m': 0.14,
                'planning_scene_timeout_sec': ParameterValue(PythonExpression([
                    "5.0 if '", sorting_mode, "' == 'true' else 2.0"]), value_type=float),
                'use_observed_collision_geometry': ParameterValue(sorting_mode, value_type=bool),
                'support_patch_margin_m': ParameterValue(PythonExpression([
                    "0.02 if '", sorting_mode, "' == 'true' else 0.0"]), value_type=float),
                'selector_grasp_approach_offset_m': grasp_approach_offset,
                'selector_grasp_lateral_offset_m': 0.030,
                'grasp_fixed_jaw_clearance_m': ParameterValue(PythonExpression([
                    "0.003 if '", sorting_mode, "' == 'true' else 0.0"]), value_type=float),
                'require_gripper_contact': True,
                'demo_start_delay_sec': 3.0,
                'stage_hold_sec': 1.0,
                'grasp_settle_sec': 1.0,
                'lift_hold_sec': 3.0,
                'count_retreat_as_lift': ParameterValue(sorting_mode, value_type=bool),
                'lift_min_center_z_m': 0.38,
                'grasp_debug_republish_period_sec': 0.5,
                'keep_debug_image_alive_on_failure': True,
            },
        ],
        output='screen',
    )
    image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=[
            '--clear-config',
            '--on-top',
            '/perception/debug_image',
        ],
        condition=IfCondition(use_image_view),
        output='log',
    )
    delayed_image_view = TimerAction(period=3.0, actions=[image_view])
    return LaunchDescription(
        [
            SetEnvironmentVariable('MUJOCO_GL', 'egl'),
            DeclareLaunchArgument(
                'start_simulator', default_value='true',
                choices=['true', 'false'],
            ),
            DeclareLaunchArgument('start_perception', default_value='true', choices=['true', 'false']),
            DeclareLaunchArgument('sam2_tracking_enabled', default_value='true', choices=['true', 'false']),
            DeclareLaunchArgument(
                'use_sim_time', default_value=start_simulator,
                choices=['true', 'false'],
            ),
            DeclareLaunchArgument(
                'model_directory', default_value=EnvironmentVariable(
                    'CLEANY_MODEL_DIR',
                    default_value=str(Path.home() / 'models'),
                ),
            ),
            DeclareLaunchArgument(
                'preload_models', default_value='true',
                choices=['true', 'false'],
            ),
            DeclareLaunchArgument('sorting_bins_config', default_value=str(
                mujoco_share / 'config' / 'robot_top_bins.yaml')),
            DeclareLaunchArgument(
                'sorting_mode', default_value='false', choices=['true', 'false']
            ),
            DeclareLaunchArgument(
                'color_image_topic', default_value='/camera/color/image_raw'
            ),
            DeclareLaunchArgument(
                'color_info_topic', default_value='/camera/color/camera_info'
            ),
            DeclareLaunchArgument(
                'depth_image_topic',
                default_value='/camera/aligned_depth_to_color/image_raw',
            ),
            DeclareLaunchArgument(
                'depth_info_topic',
                default_value=LaunchConfiguration('color_info_topic'),
            ),
            DeclareLaunchArgument('headless', default_value='false'),
            DeclareLaunchArgument('sim_speed_factor', default_value='1.0'),
            DeclareLaunchArgument('sorting_use_wrist_camera', default_value=PythonExpression([
                "'true' if '", sorting_mode, "' == 'true' and '",
                LaunchConfiguration('sam2_tracking_enabled'), "' == 'true' else 'false'",
            ]), choices=['true','false']),
            DeclareLaunchArgument('sorting_use_reference_observation', default_value=LaunchConfiguration('sam2_tracking_enabled'), choices=['true','false']),
            DeclareLaunchArgument('sorting_head_reference_refresh_age_sec', default_value='30.0'),
            DeclareLaunchArgument('wrist_continuous_tracking', default_value=LaunchConfiguration('sam2_tracking_enabled'), choices=['true','false']),
            DeclareLaunchArgument('sorting_async_carry_monitor', default_value=LaunchConfiguration('sorting_use_wrist_camera'), choices=['true','false']),
            DeclareLaunchArgument('sorting_approach_acceleration_scaling', default_value='1.0'),
            DeclareLaunchArgument('sorting_return_gripper_position_rad', default_value='-0.30'),
            DeclareLaunchArgument('sorting_wrist_cameras_config',
                default_value=str(mujoco_share / 'config' / 'sorting_wrist_cameras.yaml')),
            DeclareLaunchArgument(
                'fastdds_profiles_file', default_value=EnvironmentVariable(
                    'FASTRTPS_DEFAULT_PROFILES_FILE',
                    default_value=str(perception_share / 'config' / 'fastdds_rgbd.xml'))),
            DeclareLaunchArgument('sorting_artifact_directory', default_value=''),
            DeclareLaunchArgument('sorting_test_only_label', default_value=''),
            DeclareLaunchArgument('grasp_approach_offset_m', default_value='0.016'),
            DeclareLaunchArgument('grasp_maximum_top_contact_depth_m', default_value='0.0'),
            DeclareLaunchArgument('gripper_open_position_rad', default_value='1.2'),
            DeclareLaunchArgument('prefer_upward_closing_axis', default_value='false'),
            DeclareLaunchArgument('direct_vertical_lift', default_value='false'),
            DeclareLaunchArgument('approach_velocity_scaling', default_value=PythonExpression([
                "'1.0' if '", sorting_mode, "' == 'true' else '0.2'",
            ])),
            DeclareLaunchArgument('retreat_velocity_scaling', default_value=PythonExpression([
                "'0.8' if '", sorting_mode, "' == 'true' else '0.4'",
            ])),
            DeclareLaunchArgument('corridor_time_margin', default_value=PythonExpression([
                "'1.05' if '", sorting_mode, "' == 'true' else '2.0'",
            ])),
            DeclareLaunchArgument('velocity_scaling', default_value=PythonExpression([
                "'0.30' if '", sorting_mode, "' == 'true' else '0.08'"])),
            DeclareLaunchArgument('acceleration_scaling', default_value=PythonExpression([
                "'0.50' if '", sorting_mode, "' == 'true' else '0.08'"])),
            DeclareLaunchArgument('sorting_payload_velocity_scaling', default_value=PythonExpression([
                "'0.24' if '", sorting_mode, "' == 'true' else '0.04'"])),
            DeclareLaunchArgument('sorting_payload_acceleration_scaling', default_value=PythonExpression([
                "'0.20' if '", sorting_mode, "' == 'true' else '0.02'"])),
            DeclareLaunchArgument('lin_acceleration_scaling', default_value=PythonExpression([
                "'0.8' if '", sorting_mode, "' == 'true' else '0.4'"])),
            DeclareLaunchArgument('cartesian_translation_speed_m_s', default_value=PythonExpression([
                "'0.30' if '", sorting_mode, "' == 'true' else '0.10'"])),
            DeclareLaunchArgument('cartesian_translation_acceleration_m_s2', default_value=PythonExpression([
                "'0.80' if '", sorting_mode, "' == 'true' else '0.20'"])),
            DeclareLaunchArgument('cartesian_rotation_speed_rad_s', default_value=PythonExpression([
                "'1.50' if '", sorting_mode, "' == 'true' else '0.50'"])),
            DeclareLaunchArgument('cartesian_joint_acceleration_rad_s2', default_value=PythonExpression([
                "'4.0' if '", sorting_mode, "' == 'true' else '1.0'"])),
            DeclareLaunchArgument('gripper_motion_sec', default_value=PythonExpression([
                "'2.0' if '", sorting_mode, "' == 'true' else '3.0'"])),
            DeclareLaunchArgument('sorting_contact_diagnostics', default_value='false'),
            DeclareLaunchArgument(
                'depth_octomap_plugin',
                default_value='occupancy_map_monitor/PointCloudOctomapUpdater',
            ),
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('use_image_view', default_value='true'),
            DeclareLaunchArgument(
                'sensor_scene', default_value='true', choices=['true', 'false']
            ),
            DeclareLaunchArgument(
                'plan_only', default_value='true', choices=['true', 'false']
            ),
            DeclareLaunchArgument(
                'perception_detector_type',
                default_value=profile['detector_type'],
            ),
            DeclareLaunchArgument('gemini_model', default_value=profile['gemini_model']),
            DeclareLaunchArgument('gemini_api_key_environment', default_value=profile['gemini_api_key_environment']),
            DeclareLaunchArgument(
                'perception_segmenter_type',
                default_value=profile['segmenter_type'],
            ),
            DeclareLaunchArgument(
                'perception_minimum_detection_confidence',
                default_value=str(profile['minimum_detection_confidence']),
            ),
            DeclareLaunchArgument(
                'yoloe_model_path', default_value=profile['yoloe_model_path']
            ),
            DeclareLaunchArgument(
                'yoloe_image_size',
                default_value=str(profile['yoloe_image_size']),
            ),
            DeclareLaunchArgument(
                'yoloe_classes',
                default_value=str(profile['yoloe_classes']),
            ),
            DeclareLaunchArgument(
                'yoloe_text_encoder_directory',
                default_value=profile['yoloe_text_encoder_directory'],
            ),
            DeclareLaunchArgument(
                'perception_device',
                default_value=profile['yoloe_device'],
            ),
            DeclareLaunchArgument(
                'sam2_model_config', default_value=profile['sam2_model_config']
            ),
            DeclareLaunchArgument(
                'sam2_checkpoint', default_value=profile['sam2_checkpoint']
            ),
            OpaqueFunction(function=_preflight),
            SetEnvironmentVariable(
                'FASTRTPS_DEFAULT_PROFILES_FILE', LaunchConfiguration('fastdds_profiles_file')),
            RegisterEventHandler(OnProcessExit(
                target_action=perception,
                on_exit=[EmitEvent(event=Shutdown(
                    reason='Perception node exited'
                ))],
            )),
            backend,
            camera_tf,
            OpaqueFunction(function=_wrist_camera_transforms),
            move_group,
            collision_scene,
            depth_scene,
            Node(
                package='cleany_perception', executable='scene_cloud_receipt_node',
                condition=IfCondition(sensor_scene),
                parameters=[{'use_sim_time': clock_parameter}], output='screen',
            ),
            perception,
            grasp_server,
            selector,
            Node(
                package='cleany_mujoco_sim', executable='placement_verifier',
                condition=IfCondition(sorting_mode),
                parameters=[{
                    'use_sim_time': clock_parameter,
                    'bins_config': LaunchConfiguration('sorting_bins_config'),
                }], output='screen',
            ),
            coordinator,
            RegisterEventHandler(OnProcessExit(
                target_action=coordinator,
                on_exit=[EmitEvent(event=Shutdown(reason='Coordinator finished'))],
            ), condition=IfCondition(sorting_mode)),
            delayed_image_view,
        ]
    )
