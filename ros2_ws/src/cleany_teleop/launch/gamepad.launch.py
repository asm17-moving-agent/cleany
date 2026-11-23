"""Inspect gamepad input on the Jetson and optionally enable manual control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("cleany_teleop"), "config", "gamepad.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "drive", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument("device", default_value="auto"),
            DeclareLaunchArgument(
                "head", default_value=LaunchConfiguration("drive"), choices=["true", "false"]
            ),
            Node(
                package="cleany_teleop",
                executable="gamepad_input",
                parameters=[{"device": LaunchConfiguration("device")}],
                output="screen",
            ),
            Node(
                package="cleany_teleop",
                executable="gamepad_head",
                condition=IfCondition(LaunchConfiguration("head")),
                output="screen",
            ),
            Node(
                package="cleany_teleop",
                executable="gamepad_teleop",
                parameters=[config],
                condition=IfCondition(LaunchConfiguration("drive")),
                output="screen",
            ),
        ]
    )
