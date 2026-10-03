"""Gazebo + existing AMCL/Nav2 + mission runtime. Never starts SLAM concurrently."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from cleany_mission_manager.adapters.config import load_targets


def _start(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    sim = Path(get_package_share_directory("cleany_gazebo_sim"))
    mission = Path(get_package_share_directory("cleany_mission_manager"))
    targets = load_targets(arg("targets_file"))
    if Path(arg("map")).stem != targets.map_id:
        raise ValueError("map filename and targets map_id must match; provide a matching targets_file")
    # SCRUM navigation extraction can replace the legacy launch without copying its nodes here.
    navigation = Path(get_package_share_directory("cleany_navigation")) / "launch/amcl_nav2.launch.py"
    if not navigation.is_file():
        navigation = sim / "launch/amcl_nav2.launch.py"
    nodes = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim / "launch" / (
            "gazebo_study_cafe_fortress.launch.py" if arg("profile") == "fortress"
            else "gazebo_study_cafe.launch.py"
        ))), launch_arguments={"headless": arg("headless"), "use_sim_time": "true"}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(navigation)),
                                 launch_arguments={"use_sim_time": "true", "map": arg("map")}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(mission / "launch/mission_runtime.launch.py")),
                                 launch_arguments={name: arg(name) for name in (
                                     "runtime_config", "targets_file", "gateway_url", "post_mission", "journal_path",
                                     "bridge_journal_path", "enable_bridge",
                                 )}.items()),
    ]
    if arg("enable_telemetry") == "true":
        pose_url = arg("pose_url") or arg("gateway_url").removesuffix("/gateway/ws") + "/pose/ws"
        nodes.append(Node(package="cleany_telemetry", executable="telemetry_node", output="screen",
                          parameters=[{"odom_topic": "/ground_truth/odom", "url": pose_url}]))
    return nodes


def generate_launch_description():
    sim = Path(get_package_share_directory("cleany_gazebo_sim"))
    mission = Path(get_package_share_directory("cleany_mission_manager"))
    default_profile = "harmonic" if os.environ.get("ROS_DISTRO") == "jazzy" else "fortress"
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value=default_profile, choices=["fortress", "harmonic"]),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("runtime_config", default_value=str(mission / "config/runtime.yaml")),
        DeclareLaunchArgument("map", default_value=str(sim / "maps/study_cafe_26cm.yaml")),
        DeclareLaunchArgument("targets_file", default_value=str(mission / "config/study_cafe_targets.yaml")),
        DeclareLaunchArgument("gateway_url", default_value="ws://127.0.0.1:8080/api/robots/cleany-01/gateway/ws"),
        DeclareLaunchArgument("post_mission", default_value="return_home",
                              choices=["return_home", "wait_for_next"]),
        DeclareLaunchArgument("journal_path", default_value="~/.local/state/cleany/missions.db"),
        DeclareLaunchArgument("bridge_journal_path", default_value="~/.local/state/cleany/control-bridge.db"),
        DeclareLaunchArgument("enable_bridge", default_value="true"),
        DeclareLaunchArgument("enable_telemetry", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("pose_url", default_value="", description="Empty uses gateway host's pose endpoint."),
        # Isolate Gazebo transport from other simulations on the same host.
        DeclareLaunchArgument("sim_partition", default_value="cleany-mission"),
        SetEnvironmentVariable("GZ_PARTITION", LaunchConfiguration("sim_partition")),
        SetEnvironmentVariable("IGN_PARTITION", LaunchConfiguration("sim_partition")),
        OpaqueFunction(function=_start),
    ])
