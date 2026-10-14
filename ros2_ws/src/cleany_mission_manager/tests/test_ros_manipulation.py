"""Opt-in integration with the selectively imported Mock Action in one prefix."""

import os
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

pytestmark = pytest.mark.skipif(os.environ.get("CLEANY_RUN_MANIPULATION_TESTS") != "1",
                                reason="requires integrated mock install and isolated ROS_DOMAIN_ID")


@pytest.fixture
def manipulation_runtime(tmp_path):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from cleany_skill_executor.manipulation_node import ManipulationNode
    from cleany_mission_manager.node import MissionRuntimeNode

    root = Path(__file__).parents[1]
    # Contract regressions use the same logical fixture with accelerated timing.
    # The Gazebo smoke test exercises the default 40-second observation profile.
    fixture = yaml.safe_load((root / "config/manipulation_server_fixture.yaml").read_text())
    fixture.update(stage_duration_sec=0.5, stage_durations_sec={})
    fixture_path = tmp_path / "fast_manipulation_fixture.yaml"
    fixture_path.write_text(yaml.safe_dump(fixture))
    rclpy.init()
    server_parameters = [
        Parameter("database_path", value=str(tmp_path / "executions.db")),
        Parameter("mock_config", value=str(fixture_path)),
    ]
    runtime_parameters = [
        Parameter("targets_file", value=str(root / "config/study_cafe_targets.yaml")),
        Parameter("journal_path", value=str(tmp_path / "missions.db")),
        Parameter("navigation_backend", value="mock"),
        Parameter("manipulation_backend", value="action_mock"),
        Parameter("manipulation_initial_mock_safe", value=True),
        Parameter("mock_operation_polls", value=0),
        Parameter("tick_hz", value=50.0),
        Parameter("cancel_timeout", value=15.0),
        Parameter("operation_timeout", value=120.0),
    ]
    executor = SingleThreadedExecutor()
    harness = SimpleNamespace(runtime=None, server=None)

    def start():
        harness.server = ManipulationNode(namespace="/mock", parameter_overrides=server_parameters)
        harness.runtime = MissionRuntimeNode(parameter_overrides=runtime_parameters)
        executor.add_node(harness.server)
        executor.add_node(harness.runtime)

    def stop():
        for node in (harness.runtime, harness.server):
            executor.remove_node(node)
            node.destroy_node()

    def restart():
        stop()
        start()
    start()

    def spin(check, timeout=20):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if check():
                return
        pytest.fail(f"manipulation checkpoint timeout: {harness.runtime.runtime.snapshot()}")

    try:
        harness.spin, harness.restart = spin, restart
        spin(lambda: harness.runtime.runtime.readiness()[0])
        yield harness
    finally:
        stop()
        executor.shutdown()
        rclpy.shutdown()


def test_peer_action_two_objects_verified_success_then_home(manipulation_runtime):
    from cleany_mission_manager.core.runtime_models import RuntimeRequest
    node, server, spin = manipulation_runtime.runtime, manipulation_runtime.server, manipulation_runtime.spin
    assert node.runtime.offer(RuntimeRequest("success", "seat-12", "test")).accepted
    spin(lambda: node.runtime.last_report is not None)
    report = node.runtime.last_report
    assert report.outcome == "SUCCESS"
    assert node.navigator.commands == ["seat-12", "home"]
    assert report.completed_tasks == ["trash-1", "trash-2"]
    assert len({item.execution_id for item in report.actions}) == 2
    assert node.desk.observations == 4 and not node.desk.objects
    assert report.execution_profile == {"navigation": "mock", "perception": "mock",
                                     "planning": "mock", "execution": "mock"}
    for item in report.actions:
        result = item.manipulation
        assert result.status == "SUCCESS" and result.placement_state == "CONFIRMED"
        assert result.stop_confirmed and result.arm_recovered
        record = server.core.get(item.execution_id)
        assert record.goal.snapshot_id == item.snapshot_id
        assert record.goal.destination_id == item.destination_id == "mock_trash_bin"
        assert "failed_substage" not in report.to_dict()["actions"][report.actions.index(item)]["manipulation"]
    assert node.runtime.readiness()[0]


def test_peer_cancel_while_holding_object_blocks_home_reset_and_next_mission(manipulation_runtime):
    from cleany_mission_manager.core.runtime_models import RuntimeRequest
    node, server, spin = manipulation_runtime.runtime, manipulation_runtime.server, manipulation_runtime.spin
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
    assert node.manipulation.details["cancel_mode"] == "CHECKPOINT"


def test_peer_native_succeeded_blocked_payload_is_preserved(manipulation_runtime):
    from cleany_mission_manager.core.runtime_models import RuntimeRequest
    node, server, spin = manipulation_runtime.runtime, manipulation_runtime.server, manipulation_runtime.spin
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


def test_restart_preserves_both_databases_and_never_resumes_held_mission(manipulation_runtime):
    from cleany_mission_manager.core.runtime_models import RuntimeRequest
    harness = manipulation_runtime
    node, server = harness.runtime, harness.server
    assert node.runtime.offer(RuntimeRequest("interrupted", "seat-12", "test")).accepted
    harness.spin(lambda: server.core.record is not None and server.core.record.stage.value == "TRANSPORTING")
    execution_id = server.core.record.goal.execution_id
    harness.restart()
    node, server = harness.runtime, harness.server
    harness.spin(lambda: node.manipulation.lookup_future is not None and node.manipulation.lookup_future.done())
    node.manipulation.maintain()
    assert node.runtime.last_report.outcome == "INTERRUPTED"
    assert node.runtime.last_report.needs_human_review
    assert node.runtime.state.value == "ERROR"
    assert not node.runtime.reset_error().accepted
    assert not node.runtime.offer(RuntimeRequest("next", "seat-13", "test")).accepted
    assert node.navigator.commands == [] and server.core.port.commands == []
    record = server.core.get(execution_id)
    assert record.record_state.value == "INTERRUPTED" and record.human_confirmation_required
    assert record.object_state.value == "HELD" and not record.stop_confirmed
    assert not node.manipulation.initial_mock_safe
    assert node.manipulation.goal["execution_id"] == execution_id
    assert len(server.core.store.all_records()) == 1


def test_lost_action_result_queries_original_id_without_reexecution(manipulation_runtime, monkeypatch):
    from rclpy.task import Future
    from cleany_mission_manager.adapters.manipulation import GoalHandle
    from cleany_mission_manager.core.runtime_models import RuntimeRequest
    node, server, spin = manipulation_runtime.runtime, manipulation_runtime.server, manipulation_runtime.spin

    def lose_result(self):
        future = Future()
        future.set_exception(ConnectionError("injected result response loss"))
        return future
    monkeypatch.setattr(GoalHandle, "get_result_async", lose_result)
    assert node.runtime.offer(RuntimeRequest("lost-result", "seat-12", "test")).accepted
    spin(lambda: node.runtime.last_report is not None)
    records = server.core.store.all_records()
    assert len(records) == len(node.runtime.last_report.actions) == 1
    assert records[0].goal.execution_id == node.runtime.last_report.actions[0].execution_id
    assert records[0].result.cancel_mode == "CHECKPOINT"


def test_debug_failure_does_not_stop_control_or_change_report(manipulation_runtime, monkeypatch):
    from cleany_mission_manager.core.runtime_models import RuntimeRequest
    node, spin = manipulation_runtime.runtime, manipulation_runtime.spin

    def fail_debug():
        raise ValueError("injected diagnostic serialization failure")
    monkeypatch.setattr(node, "_publish_debug", fail_debug)
    assert node.runtime.offer(RuntimeRequest("debug-failure", "seat-12", "test")).accepted
    spin(lambda: node.runtime.last_report is not None)
    assert node.runtime.last_report.outcome == "SUCCESS"
    assert node.runtime.last_report.completed_tasks == ["trash-1", "trash-2"]
    assert node.runtime.state.value == "IDLE"


def test_bridge_websocket_offer_and_legacy_terminal_payload(manipulation_runtime, tmp_path):
    import asyncio
    import json
    import queue
    import subprocess
    import sys
    import threading
    from uuid import uuid4
    from cleany_control_bridge.session import BridgeSession

    websockets = pytest.importorskip("websockets")
    pytest.importorskip("websocket")
    node, spin = manipulation_runtime.runtime, manipulation_runtime.spin
    received, addresses, errors = [], queue.Queue(), []
    finished = threading.Event()
    session = BridgeSession()
    mission_id = str(uuid4())
    offered = session.envelope("mission.offer", {
        "mission_type": "clean_seat", "target_id": "seat-12", "requested_by": "gateway-test",
    }, mission_id)

    async def handler(socket, path=None):
        sent_offer = False
        async for raw in socket:
            event = json.loads(raw)
            received.append(event)
            if event["event_type"] != "ack":
                await socket.send(json.dumps(session.envelope("ack", {
                    "ack_event_id": event["event_id"], "disposition": "applied",
                })))
            if event["event_type"] == "robot.snapshot" and event["payload"]["state"] == "IDLE" and not sent_offer:
                await socket.send(json.dumps(offered))
                sent_offer = True

    async def serve():
        async with websockets.serve(handler, "127.0.0.1", 0) as server:
            addresses.put(server.sockets[0].getsockname()[1])
            while not finished.is_set():
                await asyncio.sleep(.02)

    def run_server():
        try:
            asyncio.run(serve())
        except Exception as exc:
            errors.append(exc)
    thread = threading.Thread(target=run_server, daemon=True)
    thread.start()
    port = addresses.get(timeout=5)
    log_path = tmp_path / "bridge.log"
    with log_path.open("w") as log:
        process = subprocess.Popen([sys.executable, "-c",
            "from cleany_control_bridge.node import main; main()", "--ros-args",
            "-p", f"url:=ws://127.0.0.1:{port}", "-p", f"journal_path:={tmp_path / 'bridge.db'}",
            "-p", "heartbeat_interval:=0.2"], stdout=log, stderr=subprocess.STDOUT)
        try:
            spin(lambda: any(event["event_type"] == "mission.result" for event in received), timeout=25)
            terminal = next(event for event in received if event["event_type"] == "mission.result")
            assert terminal["mission_id"] == mission_id
            payload = terminal["payload"]
            assert payload["outcome"] == "SUCCESS"
            assert payload["completed_tasks"] == ["trash-1", "trash-2"]
            assert payload["execution_profile"] == node.runtime.last_report.execution_profile
            assert payload["before_observation"] is None and payload["after_observation"] is None
            assert "failed_substage" not in payload and "cancel_mode" not in payload
            assert len(manipulation_runtime.server.core.store.all_records()) == 2
            assert not errors, errors
        except Exception:
            print(log_path.read_text())
            raise
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            finished.set()
            thread.join(timeout=5)
            session.close()
