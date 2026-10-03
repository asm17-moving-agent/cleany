"""Boundary failures that could otherwise cause replay or unsafe base movement."""

from concurrent.futures import Future
from dataclasses import asdict, replace
from uuid import uuid4

import pytest

from cleany_mission_manager.core.journal import MissionJournal
from cleany_mission_manager.core.manipulation import (
    ManipulationPort, goal_fields, map_outcome, validate_backend,
)
from cleany_mission_manager.core.runtime_models import ManipulationOutcome, SceneObject, TaskProposal
from cleany_mission_manager.core.result import ResultStatus
from cleany_mission_manager.mocks.desk import MockDesk
from test_runtime import Clock, drive, make_runtime, request


def resolved(value):
    future = Future()
    future.set_result(value)
    return future


def outcome(execution_id, **changes):
    return replace(ManipulationOutcome(
        execution_id, "mock", "SUCCESS", "NONE", "", "VERIFYING_PLACEMENT",
        "LEFT_GRIPPER", "CONFIRMED", "left", True, True, False, "confirmed",
    ), **changes)


def command():
    return TaskProposal("collect_trash", "scene", "display-id", "bin", 3,
                        "mission", "task", str(uuid4()))


class Handle:
    accepted = True

    def __init__(self):
        self.future = Future()

    def get_result_async(self):
        return self.future


class Transport:
    def __init__(self, journal):
        self.journal = journal
        self.sent, self.cancelled, self.queries = [], [], []
        self.acceptance = Future()
        self.handle = Handle()
        self.lookup_result = None

    def ready(self):
        return True

    def send(self, goal):
        assert goal["execution_id"] in self.journal.value(ManipulationPort.KEY)
        self.sent.append(goal)
        return self.acceptance

    def cancel(self, execution_id):
        self.cancelled.append(execution_id)
        return resolved(True)  # ACK is not physical stop evidence.

    def lookup(self, execution_id):
        self.queries.append(execution_id)
        return resolved(self.lookup_result)

    def complete(self, result, ros_status=4):
        if not self.acceptance.done():
            self.acceptance.set_result(self.handle)
        self.handle.future.set_result(dict(status=ros_status, result=asdict(result)))


@pytest.mark.parametrize("field,value", [
    ("placement_state", "NOT_CONFIRMED"), ("object_state", "HELD"),
    ("stop_confirmed", False), ("arm_recovered", False), ("execution_id", "other"),
])
def test_success_requires_matching_id_and_verified_physical_postconditions(field, value):
    with pytest.raises(ValueError):
        map_outcome(replace(outcome("id"), **{field: value}), "id", 4)


def test_ros_succeeded_with_blocked_payload_is_not_success():
    result = map_outcome(outcome("id", status="BLOCKED", error_code="TARGET_UNAVAILABLE",
                                object_state="NOT_TOUCHED", placement_state="NOT_CHECKED"), "id", 4)
    assert result.status == ResultStatus.BLOCKED


def test_display_id_is_not_converted_to_a_numeric_target():
    with pytest.raises(ValueError):
        goal_fields(replace(command(), object_id="3", wire_object_id=None))
    assert goal_fields(command())["object_id"] == 3


def test_late_acceptance_cancel_ack_and_lookup_miss_never_release_drive_gate():
    clock, journal = Clock(), MissionJournal()
    transport = Transport(journal)
    port = ManipulationPort(transport, journal, clock, initial_mock_safe=True)
    cmd = command()
    port.start(cmd)
    port.cancel(cmd.execution_id)
    assert not port.stopped() and not port.safe_to_drive()[0]
    transport.acceptance.set_result(transport.handle)  # Accepted after cancel request.
    clock.now += 2
    port.maintain()
    assert transport.cancelled == [cmd.execution_id, cmd.execution_id]
    assert port.poll(cmd.execution_id) is None
    assert not port.safe_to_drive()[0]
    result = outcome(cmd.execution_id, status="CANCELED", error_code="CANCELED",
                     object_state="HELD", arm_recovered=False)
    transport.complete(result, 5)
    assert port.poll(cmd.execution_id).status == ResultStatus.CANCELLED
    assert port.stopped() and not port.safe_to_drive()[0]


@pytest.mark.parametrize("safe", [False, True])
def test_cancelled_manipulation_records_physical_state_and_only_safe_cancel_returns_home(safe):
    clock, journal = Clock(), MissionJournal()
    transport = Transport(journal)
    desk = MockDesk(clock, (SceneObject("can", wire_object_id=3),))
    port = ManipulationPort(transport, journal, clock, initial_mock_safe=True)
    desk.executor = port
    runtime, _, nav, _ = make_runtime(clock=clock, desk=desk, journal=journal)
    runtime.offer(request())
    for _ in range(100):
        runtime.tick()
        clock.now += .1
        if transport.sent:
            break
    assert transport.sent
    goal = transport.sent[0]
    assert goal["mission_id"] == "one" and goal["destination_id"] == "mock_trash_bin"
    runtime.cancel("one")
    transport.complete(outcome(goal["execution_id"], status="CANCELED", error_code="CANCELED",
                               object_state="NOT_TOUCHED" if safe else "HELD",
                               arm_recovered=safe, placement_state="NOT_CHECKED"), 5)
    drive(runtime, clock)
    report = runtime.last_report
    assert report.outcome == "CANCELLED"
    assert report.actions[0].manipulation.stop_confirmed
    assert journal.latest().actions[0].execution_id == goal["execution_id"]
    assert nav.commands == (["seat-12", "home"] if safe else ["seat-12"])
    if not safe:
        assert report.needs_human_review and runtime.state.value == "ERROR"
        assert not runtime.reset_error().accepted
        assert not runtime.offer(request("two")).accepted


def test_restart_queries_exact_execution_without_resend_and_persists_held_state(tmp_path):
    clock = Clock()
    path = str(tmp_path / "missions.db")
    journal = MissionJournal(path)
    transport = Transport(journal)
    first = ManipulationPort(transport, journal, clock, initial_mock_safe=True)
    cmd = command()
    first.start(cmd)
    journal.close()
    journal = MissionJournal(path)
    second_transport = Transport(journal)
    second = ManipulationPort(second_transport, journal, clock, initial_mock_safe=True)
    second.maintain()
    assert not second_transport.sent
    assert second_transport.cancelled == [cmd.execution_id]
    assert second_transport.queries == [cmd.execution_id]
    assert not second.stopped() and not second.ready()[0]
    physical = outcome(cmd.execution_id, status="CANCELED", error_code="CANCELED",
                       object_state="HELD", arm_recovered=False)
    second_transport.lookup_result = dict(goal_fields(cmd), **asdict(physical),
                                         record_state="FINISHED", has_result=True,
                                         human_confirmation_required=True)
    # Finish prior lookup miss, then query new evidence.
    clock.now += 2
    second.maintain()
    second.maintain()
    assert second.stopped() and not second.safe_to_drive()[0]
    third = ManipulationPort(second_transport, journal, clock, initial_mock_safe=True)
    assert third.stopped() and not third.safe_to_drive()[0]
    with pytest.raises(ValueError, match="cannot bypass"):
        validate_backend("mock", journal)
    validate_backend("action_mock", journal)


def test_changed_destination_is_rejected_before_action_send():
    runtime, desk, _, clock = make_runtime()
    original = desk.plan
    desk.planner.handler = lambda cmd: replace(
        original(cmd), data=replace(original(cmd).data, destination_id="unapproved-bin"))
    runtime.offer(request())
    drive(runtime, clock)
    assert runtime.last_report.outcome == "BLOCKED"
    assert not desk.executor.commands
