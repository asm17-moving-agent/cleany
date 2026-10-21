"""Live D435 and selectable grasp services for an existing real_control runtime."""
import json
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from cleany_robot_interface.state_profile import prepare_state_profile
from cleany_skill_executor.motion_runtime import motion_launch_arguments, motion_runtime_prefix, selector_exit_handler


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    root = Path(value('repository_root')).resolve()
    profile_path = Path(value('profile_file')).resolve()
    run = Path(value('output_root')).expanduser() / str(time.time_ns())
    profile = prepare_state_profile(profile_path, root, run)
    bridge_config = Path(json.loads(profile_path.read_text())['rgbd_bridge']).expanduser()
    if not bridge_config.is_absolute():
        bridge_config = root / bridge_config
    skill = Path(get_package_share_directory('cleany_skill_executor'))
    perception = Path(get_package_share_directory('cleany_perception'))
    backend = value('motion_backend')
    if backend not in ('moveit', 'curobo_ik_moveit'):
        raise ValueError('Hardware sensor scene requires a MoveIt planner')
    selector_parameters = [str(skill / 'config/physical_grasp_selection.yaml'),
                           str(skill / 'config/pregrasp.yaml')]
    if backend == 'curobo_ik_moveit':
        selector_parameters.append(str(skill / 'config/hybrid_motion.yaml'))
    selector_parameters.append(str(skill / 'config/collection_motion.yaml'))
    collection_file = value('collection_config_file')
    if collection_file:
        collection = json.loads(Path(collection_file).expanduser().resolve().read_text())
        policy = {name: collection.get(name, default) for name, default in (
            ('pregrasp_motion_enabled', True), ('direct_grasp_shortcut_enabled', False))}
        if any(type(setting) is not bool for setting in policy.values()):
            raise ValueError('Collection selection policy values must be boolean')
        selector_parameters.append(policy)
    predictor = value('grasp_predictor')
    if predictor not in ('geometric', 'anygrasp'):
        raise ValueError('Unknown grasp predictor')
    grasp_nodes = []
    if predictor == 'geometric':
        grasping = Path(get_package_share_directory('cleany_grasping'))
        grasp_server = Node(
            package='cleany_grasping', executable='grasp_server', output='screen',
            parameters=[str(grasping / 'config/anygrasp.yaml'),
                        str(skill / 'config/geometric_collection.yaml')],
        )
        grasp_nodes = [
            RegisterEventHandler(OnProcessExit(target_action=grasp_server,
                on_exit=[EmitEvent(event=Shutdown(reason='Geometric grasp server exited'))])),
            grasp_server,
        ]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(perception / 'launch/realsense.launch.py')),
        launch_arguments={'serial_no': profile.camera_serial,
                          'config_file': str(perception / 'config/realsense_pregrasp.yaml')}.items(),
    )
    bridge = Node(
        package='cleany_robot_interface', executable='rgbd_frame_bridge', output='screen',
        parameters=[{'config_file': str(bridge_config),
                     'receipt_file': str(run / 'rgbd_tf_receipt.json')}],
    )
    print(f'PREGRASP_HARDWARE_SUPPORT_RUN_DIR={run}', flush=True)
    validation_parameters = [
        str(skill / 'config/validate_pick.yaml'),
        str(skill / 'config/pregrasp.yaml'),
    ]
    workspace_config = value('workspace_config_file')
    if workspace_config:
        workspace_path = Path(workspace_config).expanduser().resolve()
        if not workspace_path.is_file():
            raise ValueError(f'Workspace config does not exist: {workspace_path}')
        validation_parameters.append(str(workspace_path))
    # Target screening never determines the obstacle cloud's field of view.
    scene_parameters = [str(perception / 'config/depth_scene.yaml')]
    scene_config = value('scene_config_file')
    if scene_config:
        scene_path = Path(scene_config).expanduser().resolve()
        if not scene_path.is_file():
            raise ValueError(f'Scene config does not exist: {scene_path}')
        scene_parameters.append(str(scene_path))
    validation_parameters.append(
        {'use_sim_time': False,
         'selection_timeout_sec': float(value('selection_timeout_sec')),
         'action_timeout_sec': float(value('pipeline_timeout_sec')),
         'required_arm': value('required_arm'),
         'evidence_output_directory': value('evidence_output_directory')}
    )

    selector = Node(
        package='cleany_skill_executor', executable='physical_grasp_selection_server', output='screen',
        prefix=motion_runtime_prefix(),
        parameters=[*selector_parameters,
            {'use_sim_time': False, 'motion_backend': backend,
             'maximum_candidates': int(value('maximum_candidates')),
             'action_timeout_sec': float(value('selection_timeout_sec')),
             'visibility_camera_frame': 'head_camera_rgb_optical_frame'}])
    return [
        *grasp_nodes,
        camera,
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='RGB-D TF bridge exited'))],
        )),
        bridge,
        Node(
            package='cleany_perception', executable='depth_scene_node', output='screen',
            parameters=[*scene_parameters, {'use_sim_time': False,
                         'pixel_stride': int(value('scene_pixel_stride')),
                         'depth_image_topic': '/camera/depth/image_raw',
                         'depth_info_topic': '/camera/depth/camera_info'}],
        ),
        Node(
            package='cleany_perception', executable='scene_cloud_receipt_node', output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        selector_exit_handler(selector), selector,
        Node(
            package='cleany_skill_executor', executable='validate_pick_server', output='screen',
            parameters=validation_parameters,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('repository_root'),
        *motion_launch_arguments(),
        DeclareLaunchArgument('profile_file'),
        DeclareLaunchArgument('collection_config_file', default_value=''),
        DeclareLaunchArgument('workspace_config_file', default_value=''),
        DeclareLaunchArgument('scene_config_file', default_value=''),
        DeclareLaunchArgument('output_root', default_value='~/.local/share/cleany/pregrasp_hardware'),
        DeclareLaunchArgument('scene_pixel_stride', default_value='4'),
        DeclareLaunchArgument('maximum_candidates', default_value='32'),
        DeclareLaunchArgument('motion_backend', default_value='moveit',
                              choices=['moveit', 'curobo_ik_moveit']),
        DeclareLaunchArgument('grasp_predictor', default_value='geometric',
                              choices=['geometric', 'anygrasp']),
        DeclareLaunchArgument('required_arm', default_value=''),
        DeclareLaunchArgument('selection_timeout_sec', default_value='480.0'),
        DeclareLaunchArgument('pipeline_timeout_sec', default_value='600.0'),
        DeclareLaunchArgument('evidence_output_directory', default_value=''),
        OpaqueFunction(function=setup),
    ])
