"""Execution timing and replay must remain observations of server evidence."""

import pytest

from cleany_dev_monitor.core import Hub, Recorder
from cleany_dev_monitor.manipulation import ManipulationObserver


def record(revision=0, execution_id="execution-1", **values):
    return dict(execution_id=execution_id, revision=revision, updated_at_ns=100_000_000_000 + revision * 1_000_000_000,
                accepted_at_ns=100_000_000_000, stage="VALIDATING", substage="ValidateGoal",
                record_state="ACTIVE", has_result=False, **values)


def test_intervals_survive_log_eviction_and_snapshots_do_not_change_retroactively():
    observer, hub = ManipulationObserver(), Hub(event_limit=2)
    observer.ingest(record(), 100.)
    first = observer.snapshot()
    next_record = record(1)
    next_record.update(stage="GRASPING", substage="GraspObject")
    observer.ingest(next_record, 101.)
    final = record(2)
    final.update(stage="FINALIZING", substage="", has_result=True, record_state="FINISHED", status="SUCCESS")
    observer.ingest(final, 102.)
    hub.put("manipulation", "executions", observer.snapshot())
    for i in range(100):
        hub.put("event", "bt", {"tick": i}, critical=True)
    run = next(row["data"] for row in hub.snapshot()["rows"] if row["kind"] == "manipulation")["executions"][0]
    assert len(run["intervals"]) == 3
    assert run["intervals"][0]["end_ns"] == 101_000_000_000
    assert run["record"]["status"] == "SUCCESS" and not run["partial"]
    assert first["executions"][0]["intervals"][0]["end_ns"] is None


def test_late_join_gaps_duplicates_and_clock_regression_never_fabricate_timing():
    observer = ManipulationObserver()
    observer.ingest(record(10, completed_substages=["GraspObject"]), 110.)
    assert observer.snapshot()["executions"][0]["partial"]
    assert len(observer.snapshot()["executions"][0]["intervals"]) == 1
    assert not observer.ingest(record(9), 120.)
    assert not observer.ingest(record(10), 120.)
    assert observer.snapshot()["executions"][0]["received_at"] == 110.
    later = record(12)
    later.update(stage="RETURNING_ARM", substage="ReturnArm")
    observer.ingest(later, 112.)
    regression = record(13)
    regression["updated_at_ns"] = 90_000_000_000
    observer.ingest(regression, 113.)
    run = observer.snapshot()["executions"][0]
    assert run["partial"] and run["clock_regressed"]


def test_execution_and_interval_history_is_bounded():
    observer = ManipulationObserver(execution_limit=2, interval_limit=2)
    for revision in range(5):
        item = record(revision)
        item["substage"] = str(revision)
        observer.ingest(item, 100. + revision)
    assert len(observer.snapshot()["executions"][0]["intervals"]) == 2
    assert observer.snapshot()["executions"][0]["partial"]
    observer.ingest(record(execution_id="execution-2"), 106.)
    observer.ingest(record(execution_id="execution-3"), 107.)
    snapshot = observer.snapshot()
    assert snapshot["evicted"] == 1 and len(snapshot["executions"]) == 2
    assert {run["record"]["execution_id"] for run in snapshot["executions"]} == {"execution-2", "execution-3"}


def test_manipulation_replay_cannot_read_a_future_success(tmp_path):
    observer, hub, recorder = ManipulationObserver(), Hub(), Recorder(tmp_path)
    observer.ingest(record(), 100.)
    before = hub.put("manipulation", "executions", observer.snapshot(), received_at=100.)
    recorder.append(hub.snapshot())
    final = record(1)
    final.update(has_result=True, status="SUCCESS", record_state="FINISHED")
    observer.ingest(final, 101.)
    hub.put("manipulation", "executions", observer.snapshot(), received_at=101.)
    recorder.append(hub.snapshot())
    replay = recorder.replay(recorder.session_id, before["received_at"])
    run = next(row for row in replay["rows"] if row["kind"] == "manipulation")["data"]["executions"][0]
    assert run["intervals"][0]["received_at"] == before["received_at"]
    assert not run["record"]["has_result"]
    recorder.close()


@pytest.mark.parametrize("changes", [{"execution_id": ""}, {"revision": -1}, {"updated_at_ns": 0}])
def test_invalid_execution_records_are_rejected(changes):
    item = record()
    item.update(changes)
    with pytest.raises(ValueError):
        ManipulationObserver().ingest(item, 100.)
