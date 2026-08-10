from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
import xacro

from cleany_mujoco_sim.scene_loader import resolve_control_scene_path


def _launch_setup(context: LaunchContext) -> list[Node]:
    scene_source = Path(
        LaunchConfiguration('scene_path').perform(context)
    ).expanduser().resolve()
    control_scene = resolve_control_scene_path(scene_source)

    description_share = Path(
        get_package_share_directory('cleany_description')
    )
    control_xacro = description_share / 'urdf' / 'cleany_control.urdf.xacro'
    robot_description_xml = xacro.process_file(
        str(control_xacro),
        mappings={
            'mujoco_model': str(control_scene),
            'headless': LaunchConfiguration('headless').perform(context),
            'sim_speed_factor': LaunchConfiguration(
                'sim_speed_factor'
            ).perform(context),
        },
    ).toxml()
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_xml,
            value_type=str,
        )
    }

    controller_config = PathJoinSubstitution(
        [
            FindPackageShare('cleany_mujoco_sim'),
            'config',
            'handeye_ros2_controllers.yaml',
        ]
    )
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
        ],
        remappings=[('~/robot_description', '/robot_description')],
        emulate_tty=True,
        output='screen',
        on_exit=Shutdown(reason='MuJoCo ros2_control backend stopped'),
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
        for controller_name in (
            'joint_state_broadcaster',
            'left_arm_controller',
            'right_arm_controller',
        )
    ]
    return [robot_state_publisher, control_node, *spawners]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'scene_path',
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare('cleany_mujoco_sim'),
                        'scenes',
                        'default.xml.in',
                    ]
                ),
                description=(
                    'MuJoCo scene XML, or an XML template to materialize as '
                    'a MuJoCo 3.4-compatible control scene.'
                ),
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
            OpaqueFunction(function=_launch_setup),
        ]
    )
