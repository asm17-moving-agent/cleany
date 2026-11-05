"""One lifecycle owner keeps hardware alive until the mapping session is saved."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = str(
        Path(get_package_share_directory("cleany_bringup"))
        / "config/mapping_legacy.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("profile", default_value=default),
            DeclareLaunchArgument(
                "mock", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "state_root", default_value="~/.local/state/cleany/hardware"
            ),
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("phone_port", default_value="8766"),
            Node(
                package="cleany_bringup",
                executable="hardware_runtime",
                output="screen",
                sigterm_timeout="90",
                sigkill_timeout="15",
                arguments=[
                    "--profile",
                    LaunchConfiguration("profile"),
                    "--mock",
                    LaunchConfiguration("mock"),
                    "--state-root",
                    LaunchConfiguration("state_root"),
                    "--host",
                    LaunchConfiguration("host"),
                    "--phone-port",
                    LaunchConfiguration("phone_port"),
                ],
            ),
        ]
    )
