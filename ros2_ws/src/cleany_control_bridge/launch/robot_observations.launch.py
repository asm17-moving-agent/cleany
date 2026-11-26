"""기존 ROS 센서만 구독하는 관측 노드와 Backend 전송기를 함께 실행합니다."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(get_package_share_directory("cleany_control_bridge")) / "config/robot_observations.yaml"
    clock = {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument("url", description="Authenticated robot telemetry WebSocket URL"),
        DeclareLaunchArgument("device_token_file", description="Robot device token file"),
        DeclareLaunchArgument("provenance", choices=["real", "sim", "mock"]),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("config", default_value=str(config)),
        Node(package="cleany_telemetry", executable="robot_observer", output="screen",
             parameters=[LaunchConfiguration("config"), clock,
                         {"provenance": LaunchConfiguration("provenance")}]),
        Node(package="cleany_control_bridge", executable="robot_observation_bridge", output="screen",
             parameters=[LaunchConfiguration("config"), clock,
                         {"url": LaunchConfiguration("url"),
                          "device_token_file": LaunchConfiguration("device_token_file")}]),
    ])
