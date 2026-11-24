"""이미 실행 중인 위치 추정 및 주행 스택과 함께 임무 조율을 시작합니다."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_nodes(context):
    arg = LaunchConfiguration
    paths = {}
    for name in ("collection_config", "observation_directory"):
        value = arg(name).perform(context)
        if value:
            paths[name] = ParameterValue(value, value_type=str)
    runtime = Node(package="cleany_mission_manager", executable="mission_runtime", output="screen",
             parameters=[arg("runtime_config"), {
                 "use_sim_time": ParameterValue(arg("use_sim_time"), value_type=bool),
                 **paths,
                 "targets_file": arg("targets_file"), "journal_path": arg("journal_path"),
                 "navigation_backend": arg("navigation_backend"), "post_mission": arg("post_mission"),
                 "manipulation_mock_fixture": arg("manipulation_mock_fixture"),
                 "supervision_mode": arg("supervision_mode"),
                 "desk_approach_enabled": ParameterValue(arg("desk_approach_enabled"), value_type=bool),
                 "desk_distance_m": ParameterValue(arg("desk_distance_m"), value_type=float),
             }])
    bridge = Node(package="cleany_control_bridge", executable="control_bridge", output="screen",
             condition=IfCondition(arg("enable_bridge")), parameters=[{
                 "url": arg("gateway_url"), "device_token_file": arg("device_token_file"), "journal_path": arg("bridge_journal_path"),
             }])
    stop_on_exit = [
        RegisterEventHandler(OnProcessExit(target_action=process,
            on_exit=[EmitEvent(event=Shutdown(reason="Field mission process exited"))]),
            condition=IfCondition(arg("restart_as_unit")))
        for process in (runtime, bridge)
    ]
    return [*stop_on_exit, runtime, bridge]


def generate_launch_description():
    share = Path(get_package_share_directory("cleany_mission_manager"))
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("runtime_config", default_value=str(share / "config/runtime.yaml")),
        DeclareLaunchArgument("collection_config", default_value="", description="Override the arm collection profile"),
        DeclareLaunchArgument("observation_directory", default_value="", description="Override the Backend observation directory"),
        DeclareLaunchArgument("manipulation_mock_fixture", default_value=""),
        DeclareLaunchArgument("targets_file", default_value=str(share / "config/study_cafe_targets.yaml")),
        DeclareLaunchArgument("desk_approach_enabled", default_value="false"),
        DeclareLaunchArgument("desk_distance_m", default_value="0.75"),
        DeclareLaunchArgument("navigation_backend", default_value="nav2", choices=["nav2", "mock", "mission_drive"]),
        DeclareLaunchArgument("post_mission", default_value="return_home",
                              choices=["return_home", "wait_for_next"]),
        DeclareLaunchArgument("journal_path", default_value="~/.local/state/cleany/missions.db"),
        DeclareLaunchArgument("bridge_journal_path", default_value="~/.local/state/cleany/control-bridge.db"),
        DeclareLaunchArgument("gateway_url", default_value="ws://127.0.0.1:8080/api/robots/cleany-01/gateway/ws"),
        DeclareLaunchArgument("device_token_file", default_value=""),
        DeclareLaunchArgument("enable_bridge", default_value="true"),
        # required는 System Supervisor(cleany_system_supervisor supervision.launch.py)가 필요합니다.
        DeclareLaunchArgument("supervision_mode", default_value="required", choices=["required", "disabled"]),
        DeclareLaunchArgument("restart_as_unit", default_value="false"),
        OpaqueFunction(function=launch_nodes),
    ])
