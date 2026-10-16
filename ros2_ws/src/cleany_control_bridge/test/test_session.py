from copy import deepcopy
from uuid import uuid4

import pytest

from cleany_control_bridge.session import BridgeSession


PROFILE = {"navigation": "sim", "perception": "mock", "planning": "mock", "execution": "mock"}


def snapshot(mid=None):
    return {
        "boot_id": "runtime-boot-1", "robot_state": "BUSY" if mid else "IDLE", "ready": not mid,
        "active_request": {"mission_id": mid} if mid else None, "phase": "NAVIGATING",
        "state": "NAVIGATE_TO_TARGET" if mid else "IDLE", "bt_stage": "",
        "supported_seat_ids": ["seat-12"], "execution_profile": PROFILE, "last_result": None,
    }


def report(mid):
    return {"request": {"mission_id": mid}, "outcome": "SUCCESS", "summary": "Completed",
            "failure_code": "", "completed_tasks": ["trash-1"], "skipped_tasks": [],
            "failed_task": "", "needs_human_review": False,
            "before_observation": "mock://before", "after_observation": "mock://after",
            "navigation_result": "OK", "return_result": "OK", "execution_profile": PROFILE}


def test_outbox_replays_identical_events_and_terminal_report_after_ack_and_restart(tmp_path):
    path = str(tmp_path / "bridge.db")
    mid = str(uuid4())
    session = BridgeSession(path)
    session.observe(snapshot(mid))
    first = session.pending()
    session.observe(snapshot(mid))
    assert session.pending() == first
    assert [event["sequence"] for event in first] == [1, 2]
    session.close()
    session = BridgeSession(path)
    assert session.pending() == first
    session.result(report(mid), {**PROFILE, "navigation": "mock"})
    terminal = session.pending()[-1]
    assert terminal["payload"]["execution_profile"]["navigation"] == "sim"
    assert terminal["payload"]["before_observation"] is None
    assert terminal["payload"]["after_observation"] is None
    for event in session.pending():
        session.acknowledge(event["event_id"])
    session.close()
    session = BridgeSession(path)
    session.observe({**snapshot(), "last_result": report(mid)})
    assert not session.pending()
    assert session.sync_event()["payload"]["completed_reports"][0]["event_id"] == terminal["event_id"]
    session.close()


def test_command_ack_and_live_observation_survive_reconnect(tmp_path):
    path = str(tmp_path / "bridge.db")
    session = BridgeSession(path)
    mid = str(uuid4())
    event = session.envelope("mission.offer", {"mission_type": "clean_seat", "target_id": "seat-12",
                                              "requested_by": "operator"}, mid)
    session.validate_command(event)
    session.command_done(event)
    live = {**snapshot(mid), "phase": "WORKING", "state": "WORKING", "bt_stage": "REOBSERVE",
            "before_observation": "mock://live-before"}
    session.observe(live)
    assert session.pending()[-1]["payload"]["before_observation"] is None
    session.close()
    session = BridgeSession(path)
    assert session.command_seen(event["event_id"])
    session.observe(live)
    sync = session.sync_event()["payload"]
    assert sync["boot_id"] == "runtime-boot-1"
    assert sync["active_phase"] == "WORKING"
    assert sync["active_sequence"] == 2
    assert len(session.pending()) == 2
    session.close()


def test_unready_robot_cannot_receive_offers_via_backend_idle_filter():
    session = BridgeSession()
    session.observe({**snapshot(), "ready": False})
    assert session.sync_event()["payload"]["state"] == "ERROR"
    session.observe(snapshot())
    assert session.heartbeat()["payload"]["state"] == "IDLE"


def test_retryable_rejection_does_not_suppress_later_admission_or_offline_reports():
    session = BridgeSession()
    mid = str(uuid4())
    session.enqueue("mission.rejected", mid, {"message": "BUSY", "retryable": True})
    session.observe(snapshot(mid))
    assert [e["event_type"] for e in session.pending()] == [
        "mission.rejected", "mission.accepted", "mission.phase",
    ]
    second = str(uuid4())
    session.observe({**snapshot(), "completed_reports": [report(mid), report(second)],
                     "last_result": report(second)})
    reports = session.sync_event()["payload"]["completed_reports"]
    assert [e["mission_id"] for e in reports] == [mid, second]


@pytest.mark.parametrize("change", [
    lambda e: e.update(robot_id="another-robot"),
    lambda e: e.update(sequence=True),
    lambda e: e.update(occurred_at="2026-09-30T01:00:00"),
    lambda e: e.update(extra="not-in-v1"),
    lambda e: e["payload"].update(target_id=""),
    lambda e: e["payload"].update(extra="not-in-v1"),
])
def test_invalid_backend_commands_are_rejected_before_any_ros_operation(change):
    session = BridgeSession()
    event = session.envelope("mission.offer", {"mission_type": "clean_seat", "target_id": "seat-12",
                                              "requested_by": "operator"}, str(uuid4()))
    invalid = deepcopy(event)
    change(invalid)
    with pytest.raises((ValueError, TypeError)):
        session.validate_command(invalid)
    assert not session.pending()
