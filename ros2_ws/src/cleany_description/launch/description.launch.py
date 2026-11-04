"""Select a CAD geometry preview or the existing calibrated joint model."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from launch import LaunchContext, LaunchDescription
    from launch.action import Action


def select_model(model_variant: str, calibration_profile: str) -> str:
    """Reject measured extrinsics from a different mechanical assembly."""
    if model_variant in ("freecad_final", "freecad_kinematic"):
        if calibration_profile:
            raise ValueError(f"{model_variant} has no validated calibration profile")
        return f"cleany_{model_variant}.urdf"
    if model_variant == "legacy_runtime":
        if calibration_profile not in ("", "jetson_20261009"):
            raise ValueError(f"Unknown calibration profile: {calibration_profile}")
        return "cleany.urdf.xacro"
    raise ValueError(f"Unknown model variant: {model_variant}")


def _launch_model(context: LaunchContext) -> list[Action]:
    from launch.actions import LogInfo
    from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
    from launch_ros.actions import Node
    from launch_ros.parameter_descriptions import ParameterValue
    from launch_ros.substitutions import FindPackageShare

    variant = LaunchConfiguration("model_variant").perform(context)
    calibration = LaunchConfiguration("calibration_profile").perform(context)
    filename = select_model(variant, calibration)
    model = PathJoinSubstitution(
        [FindPackageShare("cleany_description"), "urdf", filename]
    )
    command = ["xacro ", model]
    if variant == "legacy_runtime":
        command.extend([" calibration_profile:=", calibration])
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
            parameters=[
                {
                    "robot_description": robot_description,
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
            output="screen",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "model_variant",
                default_value="freecad_final",
                choices=["freecad_final", "freecad_kinematic", "legacy_runtime"],
                description="Static CAD, CAD-relative joint preview, or existing runtime model.",
            ),
            DeclareLaunchArgument(
                "calibration_profile",
                default_value="",
                choices=["", "jetson_20261009"],
                description="Optional provisional camera extrinsics; empty keeps nominal TF.",
            ),
            OpaqueFunction(function=_launch_model),
        ]
    )
