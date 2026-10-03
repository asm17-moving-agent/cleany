"""Opt-in integration with the unmodified teammate mock server in an external overlay."""

import os
import time
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(os.environ.get("CLEANY_RUN_MANIPULATION_TESTS") != "1",
                                reason="requires teammate mock server overlay and isolated ROS_DOMAIN_ID")


@pytest.fixture
def manipulation_runtime(tmp_path):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from cleany_skill_executor.manipulation_node import ManipulationNode
    from cleany_mission_manager.node import MissionRuntimeNode

    root = Path(__file__).parents[1]
    rclpy.init()
    server = ManipulationNode(namespace="/mock", parameter_overrides=[
        Parameter("database_path", value=str(tmp_path / "executions.db")),
        Parameter("mock_config", value=str(root / "config/manipulation_server_fixture.yaml")),
    ])
    runtime = MissionRuntimeNode(parameter_overrides=[
        Parameter("targets_file", value=str(root / "config/study_cafe_targets.yaml")),
        Parameter("journal_path", value=str(tmp_path / "missions.db")),
        Parameter("navigation_backend", value="mock"),
        Parameter("manipulation_backend", value="action_mock"),
        Parameter("manipulation_initial_mock_safe", value=True),
        Parameter("mock_operation_polls", value=0),
        Parameter("tick_hz", value=50.0),
        Parameter("cancel_timeout", value=15.0),
        Parameter("operation_timeout", value=120.0),
    ])
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    executor.add_node(runtime)

    def spin(check, timeout=20):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if check():
                return
        pytest.fail(f"manipulation checkpoint timeout: {runtime.runtime.snapshot()}")

    try:
        spin(lambda: runtime.runtime.readiness()[0])
        yield runtime, server, spin
    finally:
        runtime.destroy_node()
        server.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_peer_action_two_objects_verified_success_then_home(manipulation_runtime):
    from cleany_mission_manager.core.runtime_models import RuntimeRequest
    node, server, spin = manipulation_runtime
    assert node.runtime.offer(RuntimeRequest("success", "seat-12", "test")).accepted
    spin(lambda: node.runtime.last_report is not None)
    report = node.runtime.last_report
    assert report.outcome == "SUCCESS"
    assert node.navigator.commands == ["seat-12", "home"]
    assert report.completed_tasks == ["trash-1", "trash-2"]
    assert len({item.execution_id for item in report.actions}) == 2
    for item in report.actions:
        result = item.manipulation
        assert result.status == "SUCCESS" and result.placement_state == "CONFIRMED"
        assert result.stop_confirmed and result.arm_recovered
        record = server.core.get(item.execution_id)
        assert record.goal.snapshot_id == item.snapshot_id
        assert record.goal.destination_id == item.destination_id == "mock_trash_bin"
    assert node.runtime.readiness()[0]


def test_peer_cancel_while_holding_object_blocks_home_reset_and_next_mission(manipulation_runtime):
    from cleany_mission_manager.core.runtime_models import RuntimeRequest
    node, server, spin = manipulation_runtime
    assert node.runtime.offer(RuntimeRequest("cancel", "seat-12", "test")).accepted
    spin(lambda: server.core.record is not None and server.core.record.stage.value == "TRANSPORTING")
    assert node.runtime.cancel("cancel").accepted
    spin(lambda: node.runtime.last_report is not None)
    report = node.runtime.last_report
    assert report.outcome == "CANCELLED" and report.needs_human_review
    assert report.actions[0].manipulation.object_state == "HELD"
    assert report.actions[0].manipulation.stop_confirmed
    assert node.runtime.state.value == "ERROR"
    assert node.navigator.commands == ["seat-12"]
    assert not node.runtime.reset_error().accepted
    assert not node.runtime.offer(RuntimeRequest("next", "seat-13", "test")).accepted


def test_peer_native_succeeded_blocked_payload_is_preserved(manipulation_runtime):
    from cleany_mission_manager.core.runtime_models import RuntimeRequest
    node, server, spin = manipulation_runtime
    # Goal validation is successful; backend VALIDATING produces BLOCKED payload.
    server.core.port.scenario["stage_errors"] = {"VALIDATING": "VERIFICATION_UNAVAILABLE"}
    assert node.runtime.offer(RuntimeRequest("blocked", "seat-12", "test")).accepted
    spin(lambda: node.runtime.last_report is not None)
    report = node.runtime.last_report
    assert report.outcome == "BLOCKED"
    assert report.actions[0].manipulation.status == "BLOCKED"
    assert node.runtime.state.value == "ERROR"  # arm recovery is not established
    assert not report.completed_tasks
    assert node.navigator.commands == ["seat-12"]
