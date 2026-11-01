from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "base_config",
                description="Same reviewed profile used by the running base driver",
            ),
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8766"),
            DeclareLaunchArgument(
                "token_file", default_value="~/.local/state/cleany/teleop/web-token"
            ),
            Node(
                package="cleany_teleop",
                executable="phone_remote",
                output="screen",
                arguments=[
                    "--base-config",
                    LaunchConfiguration("base_config"),
                    "--host",
                    LaunchConfiguration("host"),
                    "--port",
                    LaunchConfiguration("port"),
                    "--token-file",
                    LaunchConfiguration("token_file"),
                ],
            ),
        ]
    )
