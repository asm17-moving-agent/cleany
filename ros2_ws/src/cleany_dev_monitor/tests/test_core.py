import asyncio
import json
import math
import sqlite3

import pytest
from aiohttp import ClientSession
from aiohttp.test_utils import TestServer
from cleany_dev_monitor.core import Hub, Recorder, grid_origin, preview, transform_point
from cleany_dev_monitor.server import create_app


def test_preview_bounds_large_ros_payload_without_expanding_it():
    class Large:
        reads = 0

        def __len__(self):
            return 10_000_000

        def __iter__(self):
            raise AssertionError("must not iterate entire payload")

        def __getitem__(self, index):
            self.reads += 1
            return index

    value = Large()
    data = preview({"image": value, "nan": float("nan")}, budget=40)
    assert value.reads < 40
    assert data["image"]["truncated"]
    assert data["image"]["length"] == 10_000_000
    json.dumps(data, allow_nan=False)


def test_slow_reader_gap_is_only_reported_when_critical_events_were_evicted():
    hub = Hub(event_limit=2)
    hub.put("event", "fsm", {"state": "IDLE"}, critical=True)
    for i in range(1000):
        hub.put("sample", "fast", {"value": i})
    assert len(hub.latest) == 1
    hub.put("event", "fsm", {"state": "WORKING"}, critical=True)
    assert not hub.snapshot(1)["gap"]
    hub.put("event", "fsm", {"state": "ERROR"}, critical=True)
    assert hub.snapshot(0)["gap"]
    assert not hub.snapshot(1)["gap"]
    hub.select("a", ["/odom"])
    hub.select("b", ["/odom", "/map"])
    hub.release("a")
    assert hub.selections() == {"/odom", "/map"}
    hub.release("b")
    assert not hub.selections()


def test_rotated_grid_origin_and_cell_transform():
    origin = grid_origin([2, 3, math.pi / 4], [10, 20, math.pi / 2])
    assert origin == pytest.approx([7, 22, 3 * math.pi / 4])
    assert transform_point(1, 0, origin) == pytest.approx([7 - 2**-0.5, 22 + 2**-0.5])


def test_record_replay_layers_dedup_and_limit_preserves_evidence(tmp_path):
    hub = Hub()
    recorder = Recorder(tmp_path, session_limit=100_000)
    first = hub.put("layer", "map", {"cells": [0, 100]})
    runtime = hub.put("runtime", "fsm", {"state": "WORKING"})
    hub.put("event", "fsm", {"kind": "transition", "to": "WORKING"}, critical=True)
    recorder.append(hub.snapshot())
    hub.put("layer", "map", {"cells": [0, 100]})
    recorder.append(hub.snapshot(recorder.last))
    with sqlite3.connect(recorder.path) as db:
        assert (
            db.execute("SELECT count(*) FROM records WHERE kind='layer'").fetchone()[0]
            == 1
        )
    earlier = recorder.replay(recorder.session_id, first["received_at"])
    assert {r["kind"] for r in earlier["rows"]} == {"layer"}
    later = recorder.replay(recorder.session_id, runtime["received_at"])
    assert {r["kind"] for r in later["rows"]} == {"layer", "runtime"}
    hub.put("sample", "big", {"value": __import__("os").urandom(100000).hex()})
    recorder.append(hub.snapshot(recorder.last))
    assert recorder.reason
    assert recorder.path.exists()
    assert recorder.path.stat().st_size < 100_000
    assert len(recorder.sessions()) == 1
    assert recorder.sessions()[0]["runtime_start"] == runtime["received_at"]
    assert recorder.sessions()[0]["runtime_count"] == 1
    with pytest.raises(ValueError):
        recorder.resolve("../mission-journal")
    recorder.close()


def test_read_only_http_and_shared_subscription_cleanup(tmp_path):
    async def scenario():
        hub = Hub()
        recorder = Recorder(tmp_path)
        hub.put("graph", "ros", {"topics": [{"name": "/odom"}]})
        app = create_app(hub, {"max_selected_topics": 2}, recorder, tmp_path)
        async with TestServer(app) as server, ClientSession() as client:
            assert (
                await client.post(server.make_url("/api/mission/cancel"))
            ).status == 404
            assert (
                await client.get(
                    server.make_url("/api/snapshot"),
                    headers={"Origin": "http://evil.example"},
                )
            ).status == 403
            async with client.ws_connect(server.make_url("/ws")) as ws:
                assert (await ws.receive_json())["initial"]
                await ws.send_json({"op": "select", "topics": ["/odom"]})
                for _ in range(20):
                    reply = await ws.receive_json()
                    if "selection" in reply:
                        break
                assert hub.selections() == {"/odom"}
                await ws.send_json({"op": "publish", "topic": "/cmd_vel"})
                for _ in range(20):
                    reply = await ws.receive_json()
                    if "error" in reply:
                        break
                assert "read-only" in reply["error"]
            for _ in range(20):
                if not hub.selections():
                    break
                await asyncio.sleep(0.01)
            assert not hub.selections()
        recorder.close()

    asyncio.run(scenario())


def test_storage_unavailable_does_not_break_observation(tmp_path):
    path = tmp_path / "not-a-directory"
    path.write_text("existing evidence")
    recorder = Recorder(path / "child")
    assert recorder.reason and recorder.db is None
    hub = Hub()
    hub.put("runtime", "fsm", {"state": "ERROR"})
    recorder.append(hub.snapshot())
    assert hub.snapshot()["rows"][0]["data"]["state"] == "ERROR"
    assert path.read_text() == "existing evidence"
    hub.select("one", ["/a", "/b"], limit=2)
    assert hub.select("one", ["/b", "/c"], limit=2)
    assert not hub.select("two", ["/d"], limit=2)
