"""Gazebo, 기존 AMCL/Nav2와 Mission Runtime을 연결하며 SLAM을 동시에 시작하지 않습니다."""

import os
from pathlib import Path
from tempfile import mkdtemp

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

from cleany_mission_manager.adapters.config import (
    load_runtime_parameters, load_targets, validate_manipulation_fixture,
)


def _start(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    sim = Path(get_package_share_directory("cleany_gazebo_sim"))
    mission = Path(get_package_share_directory("cleany_mission_manager"))
    targets = load_targets(arg("targets_file"))
    if Path(arg("map")).stem != targets.map_id:
        raise ValueError("map filename and targets map_id must match; provide a matching targets_file")
    parameters = load_runtime_parameters(arg("runtime_config"))
    start_mock = arg("start_manipulation_mock") == "true"
    backend = parameters.get("manipulation_backend", "mock")
    if backend not in ("mock", "action_mock"):
        raise ValueError("manipulation_backend must be mock or action_mock")
    if start_mock and backend != "action_mock":
        raise ValueError("start_manipulation_mock=true requires manipulation_backend=action_mock")
    fixture_path = str(Path(arg("manipulation_mock_fixture")).expanduser().resolve())
    if backend == "action_mock":
        from cleany_navigation.runtime_config import adapt_nav2_parameters, merge_parameters
        validate_manipulation_fixture(parameters, fixture_path,
                                      arg("manipulation_scenario") if start_mock else None)
        collision_parameters = adapt_nav2_parameters({"collision_monitor":
            yaml.safe_load((sim / "config/nav2_safety.yaml").read_text())["collision_monitor"]},
            os.environ.get("ROS_DISTRO", "humble"))["collision_monitor"]["ros__parameters"]
    mock_node = None
    if start_mock:
        from cleany_skill_executor.manipulation.mock import MockAdapter, MockConfig
        # launch action을 반환하기 전에 전체 executor 설정을 검증합니다.
        MockAdapter(MockConfig.load(fixture_path), arg("manipulation_scenario"))
        mission_db = Path(arg("journal_path")).expanduser().resolve()
        manipulation_db = Path(arg("manipulation_database_path")).expanduser().resolve()
        if manipulation_db in (mission_db, Path(arg("bridge_journal_path")).expanduser().resolve()):
            raise ValueError("manipulation database must be separate from Mission and Bridge databases")
        mock_node = Node(package="cleany_skill_executor", executable="manipulation_server",
                         namespace="/mock", output="screen", parameters=[{
                             "backend": "mock", "mock_config": fixture_path,
                             "scenario": arg("manipulation_scenario"),
                             "database_path": str(manipulation_db), "use_sim_time": False,
                         }], remappings=[
                             ("manipulation/execute_skill", "/" + parameters.get(
                                 "manipulation_action", "mock/manipulation/execute_skill").lstrip("/")),
                             ("manipulation/get_execution", "/" + parameters.get(
                                 "manipulation_query", "mock/manipulation/get_execution").lstrip("/")),
                         ])
    navigation_share = Path(get_package_share_directory("cleany_navigation"))
    navigation = navigation_share / "launch/amcl_nav2.launch.py"
    navigation_arguments = {"use_sim_time": "true", "map": arg("map")}
    if backend == "action_mock":
        # 기존 메카넘 controller에 허용 오차 3 cm를 적용합니다.
        # 이는 변경하지 않은 Mission 도착 판정 8 cm 안에 들어갑니다.
        nav_parameters = yaml.safe_load((navigation_share / "config/nav2_amcl.yaml").read_text())
        from cleany_navigation.runtime_config import load_navigation_profile
        omni = load_navigation_profile(navigation_share / "config/nav2_omni.yaml",
                                       navigation_share / "config/nav2_omni_low_speed.yaml")
        merge_parameters(nav_parameters, {"controller_server": omni["controller_server"]})
        safety = yaml.safe_load((sim / "config/nav2_safety.yaml").read_text())
        for name in ("local_costmap", "global_costmap"):
            # 기존 monitor의 보호 영역으로 계획해 더 큰 정지 영역 때문에 기본 베이스
            # footprint로 승인된 경로가 뒤에서 차단되는 상황을 피합니다.
            nav_parameters[name][name]["ros__parameters"]["footprint"] = safety[name][name][
                "ros__parameters"]["footprint"]
        nav_config = Path(mkdtemp(prefix="cleany-mission-mock-navigation-")) / "parameters.yaml"
        nav_config.write_text(yaml.safe_dump(nav_parameters, sort_keys=False))
        navigation_arguments["params_file"] = str(nav_config)
    nav_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(navigation)),
                                         launch_arguments=navigation_arguments.items())
    if backend == "action_mock":
        # 기존 Runtime은 collision_monitor의 ACTIVE 상태를 요구합니다. 준비 조건을
        # 완화하지 않고 Nav2 명령이 해당 monitor를 통과하도록 연결합니다.
        nav_launch = GroupAction([SetRemap(src="cmd_vel", dst="nav2/cmd_vel"), nav_launch])
    runtime_arguments = {name: arg(name) for name in (
        "runtime_config", "targets_file", "gateway_url", "post_mission", "journal_path",
        "bridge_journal_path", "enable_bridge",
    )}
    runtime_arguments["manipulation_mock_fixture"] = fixture_path
    supervised = arg("supervision") == "true"
    runtime_arguments["supervision_mode"] = "required" if supervised else "disabled"
    nodes = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            sim / "launch/gazebo_study_cafe_fortress.launch.py"
        )), launch_arguments={"headless": arg("headless"), "use_sim_time": "true",
                              **({"sensor_profile": "lidar_depth_nav"} if backend == "action_mock" else {})}.items()),
        nav_launch,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(mission / "launch/mission_runtime.launch.py")),
                                 launch_arguments=runtime_arguments.items()),
    ]
    if mock_node is not None:
        nodes.insert(0, mock_node)
    if supervised:
        # 시뮬레이션에는 E-stop이나 보호 하드웨어가 없습니다. 증거 토픽을 명시하지
        # 않으면 Supervisor는 FAULT_LOCKED를 유지하고 Runtime은 미션을 거부합니다.
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory("cleany_system_supervisor")) / "launch/supervision.launch.py")),
            launch_arguments={"use_sim_time": "true", "odom_topic": "odom",
                              "estop_released_topic": arg("estop_released_topic"),
                              "protection_ready_topic": arg("protection_ready_topic")}.items()))
    if backend == "action_mock":
        nodes.extend([
            Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor",
                 output="screen", parameters=[collision_parameters, {"use_sim_time": True}]),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_safety",
                 output="screen", parameters=[{"use_sim_time": True, "autostart": True,
                                               "node_names": ["collision_monitor"]}]),
        ])
    if arg("enable_telemetry") == "true":
        pose_url = arg("pose_url") or arg("gateway_url").removesuffix("/gateway/ws") + "/pose/ws"
        nodes.extend([
            Node(package="cleany_telemetry", executable="pose_observer", output="screen",
                 parameters=[{"odom_topic": "/ground_truth/odom", "use_sim_time": True}]),
            Node(package="cleany_control_bridge", executable="pose_backend_bridge", output="screen",
                 parameters=[{"url": pose_url, "use_sim_time": True,
                              "device_token_file": ParameterValue(arg("pose_device_token_file"), value_type=str)}]),
        ])
    if arg("enable_logger") == "true":
        logger = Path(get_package_share_directory("cleany_logger"))
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(logger / "launch/logger.launch.py")),
            launch_arguments={"use_sim_time": "true", "record_on_start": arg("record_on_start"),
                              "config": arg("logger_config")}.items()))
    return nodes


def generate_launch_description():
    sim = Path(get_package_share_directory("cleany_gazebo_sim"))
    mission = Path(get_package_share_directory("cleany_mission_manager"))
    logger = Path(get_package_share_directory("cleany_logger"))
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("runtime_config", default_value=str(mission / "config/runtime.yaml")),
        DeclareLaunchArgument("start_manipulation_mock", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("manipulation_scenario", default_value="success"),
        DeclareLaunchArgument("manipulation_mock_fixture",
                              default_value=str(mission / "config/manipulation_server_fixture.yaml")),
        DeclareLaunchArgument("manipulation_database_path",
                              default_value="~/.local/state/cleany/mission-mock/manipulation.sqlite3"),
        DeclareLaunchArgument("map", default_value=str(sim / "maps/study_cafe_26cm.yaml")),
        DeclareLaunchArgument("targets_file", default_value=str(mission / "config/study_cafe_targets.yaml")),
        DeclareLaunchArgument("gateway_url", default_value="ws://127.0.0.1:8080/api/robots/cleany-01/gateway/ws"),
        DeclareLaunchArgument("post_mission", default_value="return_home",
                              choices=["return_home", "wait_for_next"]),
        DeclareLaunchArgument("journal_path", default_value="~/.local/state/cleany/missions.db"),
        DeclareLaunchArgument("bridge_journal_path", default_value="~/.local/state/cleany/control-bridge.db"),
        DeclareLaunchArgument("enable_bridge", default_value="true"),
        DeclareLaunchArgument("enable_logger", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("record_on_start", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("logger_config", default_value=str(logger / "config/logger.yaml")),
        DeclareLaunchArgument("enable_telemetry", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("supervision", default_value="false", choices=["true", "false"],
                              description="Run the System Supervisor stack; false runs the Runtime unsupervised."),
        DeclareLaunchArgument("estop_released_topic", default_value=""),
        DeclareLaunchArgument("protection_ready_topic", default_value=""),
        DeclareLaunchArgument("pose_device_token_file", default_value=""),
        DeclareLaunchArgument("pose_url", default_value="", description="Empty uses gateway host's pose endpoint."),
        # 같은 호스트의 다른 시뮬레이션과 Gazebo 통신을 격리합니다.
        DeclareLaunchArgument("sim_partition", default_value="cleany-mission"),
        SetEnvironmentVariable("IGN_PARTITION", LaunchConfiguration("sim_partition")),
        OpaqueFunction(function=_start),
    ])
