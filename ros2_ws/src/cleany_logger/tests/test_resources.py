"""Shared host sampling and recording policies, independent of Monitor HTTP."""
from contextlib import nullcontext
from types import SimpleNamespace

import psutil

from cleany_logger import Recorder
from cleany_logger.observation import ObservationBuffer
from cleany_logger.recording import allowed, defaults, validate
from cleany_logger.resources import ResourceSampler, linux_process_snapshot, psutil_process_snapshot


def test_counter_rates_first_sample_reset_and_actual_elapsed():
    sampler = ResourceSampler()
    assert sampler.rate("net", [100, 200], None) == [None, None]
    assert sampler.rate("net", [160, 300], 2) == [30, 50]
    assert sampler.rate("net", [1, 2], 1) == [None, None]
    assert sampler.rate("net", [11, 12], 0) == [None, None]


class Process:
    pid = 42

    def __init__(self, start=100, cpu=1, failure=None):
        self.start, self.cpu, self.failure = start, cpu, failure

    def oneshot(self):
        return nullcontext()

    def create_time(self):
        if self.failure:
            raise self.failure(self.pid)
        return self.start

    def cpu_times(self):
        return SimpleNamespace(user=self.cpu, system=0)

    def name(self):
        return "test-process"

    def memory_info(self):
        return SimpleNamespace(rss=1024)


def test_process_exit_permission_pid_reuse_and_bounded_history(monkeypatch):
    sampler = ResourceSampler(history_limit=2)
    monkeypatch.setattr('cleany_logger.resources.process_snapshot', psutil_process_snapshot)
    clock = iter([10, 12, 14, 16])
    monkeypatch.setattr(
        "cleany_logger.resources.time.monotonic", lambda: next(clock)
    )
    items = [
        Process(),
        Process(failure=psutil.NoSuchProcess),
        Process(failure=psutil.AccessDenied),
    ]
    monkeypatch.setattr(psutil, "process_iter", lambda: items)
    first = sampler.sample()
    assert first["processes_cpu"][0]["cpu_percent"] is None
    assert first["denied_processes"] == 1
    items[:] = [Process(cpu=2)]
    assert sampler.sample()["processes_cpu"][0]["cpu_percent"] == 50
    items[:] = [Process(start=200, cpu=20)]
    assert sampler.sample()["processes_cpu"][0]["cpu_percent"] is None
    items.clear()
    result = sampler.sample()
    assert result["processes_cpu"] == []
    assert len(result["history"]) == 2
    assert sampler.processes == {}


def test_linux_process_snapshot_reads_identity_cpu_and_rss_together(tmp_path):
    import os

    process = tmp_path / '42'
    process.mkdir()
    fields = ['0'] * 22
    fields[0], fields[11], fields[12], fields[19], fields[21] = 'S', '120', '30', '987654', '17'
    (process / 'stat').write_text('42 (worker ) name) ' + ' '.join(fields))
    (tmp_path / 'not-a-pid').mkdir()
    (tmp_path / '43').mkdir()  # Exited between enumeration and stat read.
    (tmp_path / '44').mkdir()
    (tmp_path / '44/stat').write_text('invalid stat')
    rows, denied = linux_process_snapshot(tmp_path)
    assert rows == [((42, 987654), 'worker ) name', 150 / os.sysconf('SC_CLK_TCK'), 17 * os.sysconf('SC_PAGE_SIZE'))]
    assert denied == 1


def test_linux_process_snapshot_pid_reuse_resets_cpu_baseline(tmp_path, monkeypatch):
    process = tmp_path / '42'
    process.mkdir()
    fields = ['0'] * 22
    fields[0], fields[11], fields[19], fields[21] = 'S', '100', '1000', '2'

    def write():
        (process / 'stat').write_text('42 (worker) ' + ' '.join(fields))

    write()
    monkeypatch.setattr('cleany_logger.resources.process_snapshot', lambda: linux_process_snapshot(tmp_path))
    clock = iter([10, 12, 14])
    monkeypatch.setattr('cleany_logger.resources.time.monotonic', lambda: next(clock))
    sampler = ResourceSampler()
    assert sampler.sample()['processes_cpu'][0]['cpu_percent'] is None
    fields[11] = '200'
    write()
    assert sampler.sample()['processes_cpu'][0]['cpu_percent'] > 0
    fields[19] = '2000'
    write()
    assert sampler.sample()['processes_cpu'][0]['cpu_percent'] is None


def test_read_failure_is_unknown_and_recovers(monkeypatch):
    sampler = ResourceSampler()
    original = psutil.virtual_memory

    def denied():
        raise PermissionError("denied")

    monkeypatch.setattr(psutil, "virtual_memory", denied)
    result = sampler.sample()
    assert result["memory"] is None
    assert result["memory_percent"] is None
    assert "memory: PermissionError" in result["errors"]
    monkeypatch.setattr(psutil, "virtual_memory", original)
    assert sampler.sample()["memory"] is not None


def test_unreadable_container_temperature_does_not_stop_other_metrics(monkeypatch):
    sampler = ResourceSampler()
    # Two immediate reads can share a kernel CPU tick in a container. Advance
    # counters explicitly so this test isolates temperature failure/recovery.
    from collections import namedtuple
    CpuTimes = namedtuple("CpuTimes", "user system idle")
    counters = iter([CpuTimes(10, 5, 85), CpuTimes(12, 6, 92)])
    monkeypatch.setattr(psutil, "cpu_times", lambda percpu=True: [next(counters)])

    def unavailable():
        raise AttributeError("'NoneType' object has no attribute 'strip'")

    monkeypatch.setattr(psutil, "sensors_temperatures", unavailable)
    result = sampler.sample()
    assert result['temperatures'] == []
    assert 'temperature: AttributeError' in result['errors']
    assert result['memory']['total'] > 0
    monkeypatch.setattr(psutil, "sensors_temperatures", lambda: {})
    recovered = sampler.sample()
    assert recovered['cpu_percent'] is not None
    assert 'temperature: AttributeError' not in recovered['errors']


def test_resource_recording_switch_and_old_settings_migration(tmp_path):
    settings = defaults()
    del settings["enabled"]["resources"]
    settings["enabled"]["logs"] = False
    migrated = validate(settings)
    assert migrated["enabled"]["resources"]
    assert not migrated["enabled"]["logs"]
    hub = ObservationBuffer()
    recorder = Recorder(tmp_path, controls=hub.recording)
    row = hub.put("resources", "host", {"host": "recorded", "at": 1})
    recorder.append(hub.snapshot())
    assert (
        recorder.replay(recorder.session_id, row["received_at"] + 0.01)["rows"][0][
            "data"
        ]["host"]
        == "recorded"
    )
    settings = defaults()
    settings["enabled"]["resources"] = False
    hub.recording.configure(settings)
    assert not allowed(row, settings)
    hidden = hub.put("resources", "host", {"host": "hidden"})
    recorder.append(hub.snapshot())
    replay = recorder.replay(recorder.session_id, hidden["received_at"] + 0.01)
    assert all(r["data"].get("host") != "hidden" for r in replay["rows"])
    recorder.close()
