"""Start orchestration beside an already-running localization/navigation stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("cleany_mission_manager"))
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("runtime_config", default_value=str(share / "config/runtime.yaml")),
        DeclareLaunchArgument("targets_file", default_value=str(share / "config/study_cafe_targets.yaml")),
        DeclareLaunchArgument("navigation_backend", default_value="nav2", choices=["nav2", "mock"]),
        DeclareLaunchArgument("post_mission", default_value="return_home",
                              choices=["return_home", "wait_for_next"]),
        DeclareLaunchArgument("journal_path", default_value="~/.local/state/cleany/missions.db"),
        DeclareLaunchArgument("bridge_journal_path", default_value="~/.local/state/cleany/control-bridge.db"),
        DeclareLaunchArgument("gateway_url", default_value="ws://127.0.0.1:8080/api/robots/cleany-01/gateway/ws"),
        DeclareLaunchArgument("enable_bridge", default_value="true"),
        Node(package="cleany_mission_manager", executable="mission_runtime", output="screen",
             parameters=[arg("runtime_config"), {
                 "use_sim_time": ParameterValue(arg("use_sim_time"), value_type=bool),
                 "targets_file": arg("targets_file"), "journal_path": arg("journal_path"),
                 "navigation_backend": arg("navigation_backend"), "post_mission": arg("post_mission"),
             }]),
        Node(package="cleany_control_bridge", executable="control_bridge", output="screen",
             condition=IfCondition(arg("enable_bridge")), parameters=[{
                 "url": arg("gateway_url"), "journal_path": arg("bridge_journal_path"),
             }]),
    ])
