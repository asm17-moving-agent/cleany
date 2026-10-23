"""Run the recorder independently of the developer dashboard."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory("cleany_logger")) / "config/logger.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=config),
        DeclareLaunchArgument("record_on_start", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
        Node(package="cleany_logger", executable="logger_node", output="screen", parameters=[
            LaunchConfiguration("config"),
            {"record_on_start": ParameterValue(LaunchConfiguration("record_on_start"), value_type=bool),
             "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
        ]),
    ])
