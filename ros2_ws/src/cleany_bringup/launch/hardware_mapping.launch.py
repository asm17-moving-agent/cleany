"""한 실행 관리자가 매핑 세션 저장이 끝날 때까지 하드웨어를 유지합니다."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from cleany_navigation.motion_profile import motion_profile_arguments


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
            DeclareLaunchArgument("navigation_map", default_value=""),
            DeclareLaunchArgument("robot_params_file", default_value=""),
            DeclareLaunchArgument("depth_params_file", default_value=""),
            *motion_profile_arguments("omni"),
            DeclareLaunchArgument("slip_recovery", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("phone_port", default_value="8766"),
            Node(
                package="cleany_bringup",
                executable="hardware_runtime",
                output="screen",
                sigterm_timeout="90",
                sigkill_timeout="15",
                arguments=[
                    "--navigation-map", LaunchConfiguration("navigation_map"),
                    "--robot-params", LaunchConfiguration("robot_params_file"),
                    "--depth-params", LaunchConfiguration("depth_params_file"),
                    "--motion-profile", LaunchConfiguration("motion_profile"),
                    "--slip-recovery", LaunchConfiguration("slip_recovery"),
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
