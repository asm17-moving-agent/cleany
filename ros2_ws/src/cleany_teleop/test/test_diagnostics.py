import json
import threading

import pytest

from cleany_teleop.event_journal import EventJournal, _CheckedHandler
from cleany_teleop.mapping_status import MappingStatusCache
from cleany_teleop.web_control import Control


def test_mapping_heartbeat_expires_even_while_http_can_read_cached_state():
    now = [10.0]
    cache = MappingStatusCache(now=lambda: now[0])
    assert cache.status()["phase"] == "unavailable"
    cache.observe(dict(phase="running", output="/session", operation_id="reset-1"))
    assert cache.status()["connected"]
    now[0] += 1.01
    status = cache.status()
    assert status["phase"] == "unavailable" and not status["connected"]
    assert status["last_phase"] == "running" and status["status_age_ms"] == 1010.0
    assert status["output"] == "/session" and status["operation_id"] == "reset-1"
    cache.observe(dict(phase="idle"))
    assert cache.status()["phase"] == "idle" and cache.status()["connected"]


@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf")])
def test_mapping_timeout_rejects_invalid_configuration(value):
    with pytest.raises(ValueError):
        MappingStatusCache(value)


def test_control_events_survive_recent_event_window_and_restart(tmp_path):
    path = tmp_path / "events.jsonl"
    journal = EventJournal(path)
    control = Control(lambda _: None, now=lambda: 10.0, event_sink=journal.emit)
    for sequence in range(50):
        control.event("press", client="phone", sequence=sequence)
    journal.close()
    assert len(control.events) == 24
    assert journal.status()["written"] == 50 and journal.status()["dropped"] == 0
    restarted = EventJournal(path)
    restarted.emit(dict(kind="restart"))
    restarted.close()
    records = [json.loads(line) for line in path.read_text().splitlines()]
    assert len(records) == 51
    assert records[0]["run_id"] != records[-1]["run_id"]
    assert records[0]["utc"] and records[0]["at"] == 10.0
    assert [record["event_id"] for record in records[:50]] == list(range(1, 51))


def test_journal_rotates_and_keeps_valid_json_records(tmp_path):
    path = tmp_path / "events.jsonl"
    journal = EventJournal(path, max_bytes=600, backups=2)
    for sequence in range(20):
        journal.emit(dict(kind="stop", sequence=sequence))
    journal.close()
    files = list(tmp_path.glob("events.jsonl*"))
    assert len(files) == 3
    for file in files:
        for line in file.read_text().splitlines():
            assert json.loads(line)["kind"] == "stop"
    assert journal.status()["written"] == 20


def test_full_queue_drops_records_without_waiting_for_disk(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = _CheckedHandler.emit

    def blocked(handler, record):
        entered.set()
        assert release.wait(3)
        original(handler, record)

    monkeypatch.setattr(_CheckedHandler, "emit", blocked)
    journal = EventJournal(tmp_path / "events.jsonl", capacity=1)
    try:
        journal.emit(dict(kind="first"))
        assert entered.wait(1)
        journal.emit(dict(kind="second"))
        journal.emit(dict(kind="overflow"))
        assert journal.status()["dropped"] == 1
    finally:
        release.set()
        journal.close()
    assert journal.status()["written"] == 2


def test_disk_failure_is_visible_and_does_not_break_stop(tmp_path):
    blocker = tmp_path / "file"
    blocker.write_text("not a directory")
    journal = EventJournal(blocker / "events.jsonl")
    control = Control(lambda _: None, now=lambda: 1.0, event_sink=journal.emit)
    control.stop("test stop")
    journal.close()
    assert control.phase == "stopping"
    assert journal.status()["error"] and journal.status()["dropped"] == 1
    assert journal.status()["written"] == 0
