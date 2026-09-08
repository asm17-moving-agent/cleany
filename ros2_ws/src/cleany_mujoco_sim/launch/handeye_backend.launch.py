from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
import xacro

from cleany_mujoco_sim.scene_loader import resolve_control_scene_path
from cleany_mujoco_sim.scene_manifest import (
    default_manifest_path,
    load_handeye_scene_manifest,
    preflight_manifest,
)


_INITIAL_JOINT_ARGUMENTS = tuple(
    f'{side}_{joint}_initial'
    for side in ('left', 'right')
    for joint in (
        'shoulder_yaw',
        'shoulder_pitch',
        'elbow_pitch',
        'wrist_pitch',
        'wrist_roll',
        'gripper',
    )
)


def _launch_setup(context: LaunchContext) -> list[Node]:
    scene_source = Path(
        LaunchConfiguration('scene_path').perform(context)
    ).expanduser().resolve()
    initial_joint_positions = {
        f'{name.removesuffix("_initial")}_joint': float(
            LaunchConfiguration(name).perform(context)
        )
        for name in _INITIAL_JOINT_ARGUMENTS
    }
    initial_joint_positions['head_tilt_joint'] = float(
        LaunchConfiguration('head_tilt_initial').perform(context)
    )
    bins_path = LaunchConfiguration('sorting_bins_config').perform(context)
    if LaunchConfiguration('scheduled_cameras').perform(context)=='true' and not bins_path:
        raise ValueError('Scheduled cameras require the sorting hardware backend')
    observer_parameters = {}
    hardware_plugin = 'mujoco_ros2_control/MujocoSystemInterface'
    if bins_path:
        get_package_share_directory('cleany_mujoco_observer')
        hardware_plugin = 'cleany_mujoco_observer/ObservedMujocoSystem'
        observer_parameters = {
            'sorting_observer.publish_contacts': ParameterValue(
                LaunchConfiguration('sorting_contact_diagnostics'), value_type=bool
            ),
        }
    control_scene = resolve_control_scene_path(
        scene_source,
        initial_joint_positions=initial_joint_positions,
        sorting_bins_config=Path(bins_path) if bins_path else None,
    )
    manifest_path = default_manifest_path().resolve()
    manifest = load_handeye_scene_manifest(manifest_path)
    preflight_manifest(manifest, profile='simulation')
    camera = manifest.camera_contract

    description_share = Path(
        get_package_share_directory('cleany_description')
    )
    control_xacro = description_share / 'urdf' / 'cleany_control.urdf.xacro'
    camera_name = LaunchConfiguration('camera_name').perform(context)
    color_topic = LaunchConfiguration('color_image_topic').perform(context)
    info_topic = LaunchConfiguration('camera_info_topic').perform(context)
    depth_topic = LaunchConfiguration('depth_image_topic').perform(context)
    if any((color_topic, info_topic, depth_topic)) and (
        LaunchConfiguration('enable_camera_contract_adapter').perform(context)
        == 'true'
    ):
        raise ValueError('Custom topics require contract adapter disabled')
    robot_description_xml = xacro.process_file(
        str(control_xacro),
        mappings={
            'mujoco_model': str(control_scene),
            'mujoco_hardware_plugin': hardware_plugin,
            'headless': LaunchConfiguration('headless').perform(context),
            'sim_speed_factor': LaunchConfiguration(
                'sim_speed_factor'
            ).perform(context),
            'camera_publish_rate': f'{camera.publish_rate_hz:g}',
            'scheduled_cameras': LaunchConfiguration('scheduled_cameras').perform(context),
            'camera_name': camera_name,
            'camera_frame_name': LaunchConfiguration(
                'camera_frame_name'
            ).perform(context),
            'enable_gripper_command': LaunchConfiguration(
                'enable_gripper_controllers'
            ).perform(context),
            **{
                name: LaunchConfiguration(name).perform(context)
                for name in _INITIAL_JOINT_ARGUMENTS
            },
        },
    ).toxml()
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_xml,
            value_type=str,
        )
    }

    controller_config = LaunchConfiguration('controller_config')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen',
    )
    control_node = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        parameters=[
            {'use_sim_time': True},
            ParameterFile(controller_config),
            observer_parameters,
        ],
        remappings=[
            ('~/robot_description', '/robot_description'),
            (f'/{camera_name}/color',
             color_topic or camera.internal_image_topic),
            (f'/{camera_name}/camera_info',
             info_topic or camera.internal_info_topic),
            (f'/{camera_name}/depth',
             depth_topic or camera.internal_depth_topic),
        ],
        emulate_tty=True,
        output='screen',
        on_exit=Shutdown(reason='MuJoCo ros2_control backend stopped'),
    )
    camera_contract_adapter = Node(
        package='cleany_mujoco_sim',
        executable='camera_contract_adapter',
        condition=IfCondition(
            LaunchConfiguration('enable_camera_contract_adapter')
        ),
        parameters=[
            {'use_sim_time': True, 'manifest_path': str(manifest_path)},
        ],
        output='screen',
    )

    controller_names = [
        'joint_state_broadcaster',
        'left_arm_controller',
        'right_arm_controller',
    ]
    if LaunchConfiguration('enable_gripper_controllers').perform(
        context
    ).lower() in ('true', '1', 'yes'):
        controller_names.extend(
            ('left_gripper_controller', 'right_gripper_controller')
        )
    spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                controller_name,
                '--controller-manager',
                '/controller_manager',
                '--controller-manager-timeout',
                '120',
                '--param-file',
                controller_config,
            ],
            output='screen',
        )
        for controller_name in controller_names
    ]
    return [
        robot_state_publisher,
        control_node,
        camera_contract_adapter,
        *spawners,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument('scheduled_cameras', default_value='false'),
            DeclareLaunchArgument(
                'scene_path',
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare('cleany_mujoco_sim'),
                        'scenes',
                        'handeye.xml.in',
                    ]
                ),
                description=(
                    'MuJoCo scene XML, or an XML template to materialize as '
                    'a MuJoCo 3.4-compatible control scene.'
                ),
            ),
            DeclareLaunchArgument(
                'controller_config',
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare('cleany_mujoco_sim'),
                        'config',
                        'handeye_ros2_controllers.yaml',
                    ]
                ),
                description='ros2_control YAML for this simulation workflow.',
            ),
            DeclareLaunchArgument(
                'headless',
                default_value='true',
                description='Run MuJoCo without its native viewer.',
            ),
            DeclareLaunchArgument(
                'sim_speed_factor',
                default_value='1.0',
                description='MuJoCo simulation speed relative to wall time.',
            ),
            DeclareLaunchArgument(
                'camera_name',
                default_value='left_wrist_rgb',
                description='MJCF camera exposed by mujoco_ros2_control.',
            ),
            DeclareLaunchArgument(
                'camera_frame_name',
                default_value='left_wrist_rgb_vendor_frame',
            ),
            DeclareLaunchArgument('color_image_topic', default_value=''),
            DeclareLaunchArgument('camera_info_topic', default_value=''),
            DeclareLaunchArgument('depth_image_topic', default_value=''),
            DeclareLaunchArgument('sorting_bins_config', default_value=''),
            DeclareLaunchArgument('sorting_contact_diagnostics', default_value='false'),
            DeclareLaunchArgument(
                'enable_camera_contract_adapter',
                default_value='true',
                description='Publish the calibrated left-wrist RGB contract.',
            ),
            DeclareLaunchArgument(
                'enable_gripper_controllers',
                default_value='false',
                description=(
                    'Expose and start left/right gripper trajectory actions.'
                ),
            ),
            DeclareLaunchArgument(
                'head_tilt_initial',
                default_value='0.0',
                description=(
                    'Initial read-only MuJoCo head tilt position in radians.'
                ),
            ),
            *(
                DeclareLaunchArgument(
                    name,
                    default_value='0.0',
                    description=(
                        'Initial MuJoCo joint position in radians; defaults '
                        'to the existing zero-state pose.'
                    ),
                )
                for name in _INITIAL_JOINT_ARGUMENTS
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
