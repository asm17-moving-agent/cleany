"""Display calibrated CAD or a model selected for a simulation preview."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from launch import LaunchContext, LaunchDescription
    from launch.action import Action


def select_model(model_variant: str, calibration_profile: str) -> str:
    """Reject measured extrinsics from a different mechanical assembly."""
    if model_variant == "freecad_calibrated":
        if calibration_profile:
            raise ValueError("Use model_file to select a calibrated CAD model")
        return "cleany.urdf"
    if model_variant in ("freecad_final", "freecad_kinematic"):
        if calibration_profile:
            raise ValueError(f"{model_variant} has no validated calibration profile")
        return f"cleany_{model_variant}.urdf"
    if model_variant == "legacy_runtime":
        if calibration_profile not in ("", "jetson_20261009", "jetson_20261107", "jetson_20261108"):
            raise ValueError(f"Unknown calibration profile: {calibration_profile}")
        return "cleany.urdf.xacro"
    raise ValueError(f"Unknown model variant: {model_variant}")


def _launch_model(context: LaunchContext) -> list[Action]:
    from launch.actions import LogInfo
    from launch.substitutions import Command, PathJoinSubstitution
    from launch_ros.parameter_descriptions import ParameterValue
    from launch_ros.substitutions import FindPackageShare

    variant = LaunchConfiguration("model_variant").perform(context)
    calibration = LaunchConfiguration("calibration_profile").perform(context)
    filename = select_model(variant, calibration)
    if variant == "freecad_calibrated":
        return _launch_calibrated(context, filename)
    model = PathJoinSubstitution(
        [FindPackageShare("cleany_description"), "urdf", filename]
    )
    command = ["xacro ", model]
    if variant == "legacy_runtime":
        command.extend([" model_profile:=", LaunchConfiguration("model_profile").perform(context),
                        " calibration_profile:=", calibration])
    robot_description = ParameterValue(
        Command(command),
        value_type=str,
    )
    return [
        LogInfo(
            msg=f"Cleany model: {variant}; CAD variants are geometry previews without actuator calibration."
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=LaunchConfiguration("namespace"),
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            parameters=[
                {
                    "robot_description": robot_description,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "frame_prefix": ParameterValue(
                        LaunchConfiguration("frame_prefix"), value_type=str
                    ),
                }
            ],
            output="screen",
        ),
    ]


def _launch_calibrated(context: LaunchContext, filename: str) -> list[Action]:
    def value(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    package = Path(get_package_share_directory('cleany_description'))
    model = Path(value('model_file')) if value('model_file') else package / 'urdf' / filename
    description = model.read_text()
    if 'cleany_cad_calibration' not in ET.fromstring(description).attrib:
        raise ValueError('The display requires the calibrated CAD runtime model')
    namespace = value('namespace').strip('/') or 'runtime_model'
    transforms = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=namespace, name='runtime_robot_state_publisher', output='screen',
             remappings=transforms, parameters=[{'robot_description': description,
                                                'use_sim_time': False,
                                                'frame_prefix': value('frame_prefix')}]),
        Node(package='cleany_description', executable='runtime_model_state',
             namespace=namespace, output='screen',
             parameters=[{'model_file': str(model), 'input_topic': value('input_joint_states')}]),
        Node(package='rviz2', executable='rviz2', namespace=namespace, name='runtime_rviz',
             condition=IfCondition(LaunchConfiguration('use_rviz')), output='log',
             remappings=transforms,
             arguments=['-d', str(package/'config/runtime_model.rviz')])]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("model_profile", default_value="jetson_20261009",
                                  choices=["jetson_20261009"]),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace for description, joint state and TF topics.",
            ),
            DeclareLaunchArgument(
                "frame_prefix",
                default_value="",
                description="Prefix for published TF frame IDs, such as cad_reference/.",
            ),
            DeclareLaunchArgument(
                "model_variant",
                default_value="freecad_calibrated",
                choices=["freecad_calibrated", "freecad_final", "freecad_kinematic", "legacy_runtime"],
                description="Static CAD, CAD-relative joint preview, or existing runtime model.",
            ),
            DeclareLaunchArgument(
                "calibration_profile",
                default_value="",
                choices=["", "jetson_20261009", "jetson_20261107", "jetson_20261108"],
                description="Optional provisional camera extrinsics; empty keeps nominal TF.",
            ),
            DeclareLaunchArgument("model_file", default_value=""),
            DeclareLaunchArgument("input_joint_states", default_value="/joint_states"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            OpaqueFunction(function=_launch_model),
        ]
    )
