import asyncio
from contextlib import nullcontext
from types import SimpleNamespace

import psutil
import pytest
from aiohttp import ClientSession
from aiohttp.test_utils import TestServer

from cleany_dev_monitor.core import Hub, Recorder
from cleany_dev_monitor.recording import allowed, defaults, validate
from cleany_dev_monitor.resources import ResourceSampler
from cleany_dev_monitor.server import create_app


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
    clock = iter([10, 12, 14, 16])
    monkeypatch.setattr(
        "cleany_dev_monitor.resources.time.monotonic", lambda: next(clock)
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
    hub = Hub()
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


def test_http_worker_live_metrics_and_shutdown(tmp_path):
    async def run():
        hub = Hub()
        recorder = Recorder(tmp_path)
        app = create_app(hub, {"resource_interval_s": 0.5}, recorder, tmp_path)
        server = TestServer(app)
        await server.start_server()
        async with ClientSession() as client:
            response = await client.get(server.make_url('/api/recording/settings'))
            policy = await response.json()
            assert policy['observation']['resources']['summary'] == '수집 설정 0.5초 (2 Hz)'
            assert policy['labels']['resources'] == '호스트 리소스'
            settings = policy['settings']
            settings['enabled']['resources'] = False
            response = await client.put(server.make_url('/api/recording/settings'), json=settings)
            assert response.status == 200
            assert not (await response.json())['settings']['enabled']['resources']
        async with ClientSession() as client:
            for _ in range(30):
                response = await client.get(server.make_url("/api/snapshot"))
                packet = await response.json()
                resources = [r for r in packet["rows"] if r["kind"] == "resources"]
                if resources and resources[0]["data"].get("cpu_percent") is not None:
                    break
                await asyncio.sleep(0.1)
            assert resources[0]["data"]["host"]
            assert resources[0]["data"]["cpu_percent"] is not None
            assert len(resources[0]["data"]["history"]) >= 2
        await server.close()
        seq = hub.sequence
        await asyncio.sleep(0.6)
        assert hub.sequence == seq
        recorder.close()

    asyncio.run(run())
