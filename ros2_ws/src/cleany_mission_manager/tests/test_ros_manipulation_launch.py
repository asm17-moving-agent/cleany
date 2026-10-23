"""Composition preflight rejects errors before returning simulator launch actions."""

import importlib.util
from pathlib import Path

import pytest
import yaml

pytest.importorskip("launch")
pytest.importorskip("launch_ros")

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node


@pytest.fixture
def composition(tmp_path, monkeypatch):
    source = Path(__file__).parents[2]
    monkeypatch.syspath_prepend(str(source / "cleany_navigation"))
    spec = importlib.util.spec_from_file_location("mission_sim", source / "cleany_bringup/launch/mission_sim.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: str(source / name))
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update({
        "map": str(source / "cleany_gazebo_sim/maps/study_cafe_26cm.yaml"),
        "targets_file": str(source / "cleany_mission_manager/config/study_cafe_targets.yaml"),
        "runtime_config": str(source / "cleany_mission_manager/config/runtime_gazebo_manipulation_mock.yaml"),
        "manipulation_mock_fixture": str(source / "cleany_mission_manager/config/manipulation_server_fixture.yaml"),
        "start_manipulation_mock": "true", "manipulation_scenario": "success",
        "manipulation_database_path": str(tmp_path / "executions.db"),
        "journal_path": str(tmp_path / "missions.db"), "bridge_journal_path": str(tmp_path / "bridge.db"),
        "headless": "true", "gateway_url": "ws://localhost/gateway/ws",
        "post_mission": "wait_for_next", "enable_bridge": "false", "enable_telemetry": "false",
    })
    return module, context


@pytest.mark.parametrize("backend,start,count", [("mock", "false", 3),
    ("action_mock", "false", 5), ("action_mock", "true", 6)])
@pytest.mark.parametrize("enable_logger", ["false", "true"])
def test_supported_compositions_start_exactly_one_simulator(
        composition, tmp_path, backend, start, count, enable_logger):
    module, context = composition
    assert context.launch_configurations["enable_logger"] == "false"
    parameters = yaml.safe_load(Path(context.launch_configurations["runtime_config"]).read_text())
    parameters["mission_runtime"]["ros__parameters"]["manipulation_backend"] = backend
    path = tmp_path / "runtime.yaml"
    path.write_text(yaml.safe_dump(parameters))
    context.launch_configurations.update(runtime_config=str(path), start_manipulation_mock=start,
                                         enable_logger=enable_logger)
    actions = module._start(context)
    logger_count = int(enable_logger == "true")
    assert len(actions) == count + logger_count
    assert sum(isinstance(action, (IncludeLaunchDescription, GroupAction)) for action in actions) == 3 + logger_count
    assert sum(isinstance(action, IncludeLaunchDescription)
               and "headless" in dict(action.launch_arguments) for action in actions) == 1
    assert sum(isinstance(action, IncludeLaunchDescription)
               and "record_on_start" in dict(action.launch_arguments) for action in actions) == logger_count
    expected = (2 if backend == "action_mock" else 0) + (1 if start == "true" else 0)
    assert sum(isinstance(action, Node) for action in actions) == expected
    if backend == "action_mock":
        group = next(action for action in actions if isinstance(action, GroupAction))
        nav = next(action for action in group.get_sub_entities() if isinstance(action, IncludeLaunchDescription))
        arguments = dict(nav.launch_arguments)
        parameters = yaml.safe_load(Path(arguments["params_file"]).read_text())
        controller = parameters["controller_server"]["ros__parameters"]
        assert controller["goal_checker"]["xy_goal_tolerance"] < .08
        assert controller["FollowPath"]["plugin"] == "cleany_navigation::GoalDirectedController"
        safety = yaml.safe_load((Path(__file__).parents[2] / "cleany_gazebo_sim/config/nav2_safety.yaml").read_text())
        for name in ("local_costmap", "global_costmap"):
            assert parameters[name][name]["ros__parameters"]["footprint"] == safety[name][name]["ros__parameters"]["footprint"]


@pytest.mark.parametrize("record_on_start", ["false", "true"])
def test_logger_include_forwards_recording_configuration(composition, tmp_path, record_on_start):
    module, context = composition
    config = tmp_path / "logger.yaml"
    context.launch_configurations.update(enable_logger="true", record_on_start=record_on_start,
                                         logger_config=str(config))
    actions = module._start(context)
    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)
                and "record_on_start" in dict(action.launch_arguments)]
    assert len(includes) == 1
    assert dict(includes[0].launch_arguments) == {
        "use_sim_time": "true", "record_on_start": record_on_start, "config": str(config),
    }


@pytest.mark.parametrize("error", ["backend", "scenario", "snapshots", "destination", "database", "timeout"])
def test_configuration_errors_return_no_launch_actions(composition, tmp_path, error):
    module, context = composition
    if error == "backend":
        context.launch_configurations["runtime_config"] = str(
            Path(__file__).parents[1] / "config/runtime.yaml")
    if error == "scenario":
        context.launch_configurations["manipulation_scenario"] = "unregistered"
    if error == "database":
        context.launch_configurations["manipulation_database_path"] = context.launch_configurations["journal_path"]
    if error in ("snapshots", "destination", "timeout"):
        fixture = yaml.safe_load(Path(context.launch_configurations["manipulation_mock_fixture"]).read_text())
        if error == "snapshots":
            fixture["snapshots"].pop("mock-snapshot-002")
        if error == "destination":
            fixture["destination_id"] = "other-bin"
        if error == "timeout":
            fixture["motion_timeout_sec"] = 0.0
        path = tmp_path / "fixture.yaml"
        path.write_text(yaml.safe_dump(fixture))
        context.launch_configurations["manipulation_mock_fixture"] = str(path)
    with pytest.raises(ValueError):
        module._start(context)
