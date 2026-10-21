"""Slow recording catalogs must not starve live telemetry or operator requests."""
import asyncio
import sqlite3
import threading

from aiohttp import ClientSession
from aiohttp.test_utils import TestServer

from cleany_dev_monitor.core import Hub, Recorder
from cleany_dev_monitor.server import create_app


def test_websocket_deltas_keep_initial_state_reconnect_and_source_reset(tmp_path):
    async def scenario():
        hub, recorder = Hub(), Recorder(tmp_path)
        hub.put('layer', 'map', {'cells': [0, 100]})
        try:
            async with TestServer(create_app(hub, {}, recorder, tmp_path)) as server, ClientSession() as client:
                async with client.ws_connect(server.make_url('/ws')) as ws:
                    initial = await ws.receive_json(timeout=2)
                    assert initial['initial']
                    assert any(row['key'] == 'map' for row in initial['rows'])
                    hub.put('pose', 'robot', {'valid': False})
                    while True:
                        update = await ws.receive_json(timeout=2)
                        if any(row['kind'] == 'pose' for row in update['rows']):
                            break
                    assert not update['initial']
                    assert all(row['key'] != 'map' for row in update['rows'])
                    hub.reset_sources({'map_topic': '/replacement'}, 1)
                    hub.put('layer', 'new-map', {'cells': [100, 0]})
                    while True:
                        reset = await ws.receive_json(timeout=2)
                        if reset['sources_changed']:
                            break
                    assert reset['initial'] and reset['topic_revision'] == 1
                    assert any(row['key'] == 'new-map' for row in reset['rows'])
                    assert all(row['key'] != 'map' for row in reset['rows'])
                async with client.ws_connect(server.make_url('/ws')) as ws:
                    reconnected = await ws.receive_json(timeout=2)
                    assert reconnected['initial']
                    assert any(row['key'] == 'new-map' for row in reconnected['rows'])
        finally:
            recorder.close()

    asyncio.run(scenario())


def test_recording_catalog_does_not_block_live_requests_and_refresh_is_shared(tmp_path, monkeypatch):
    async def scenario():
        hub, recorder = Hub(), Recorder(tmp_path)
        hub.put('config', 'monitor', {'control_enabled': False})
        started, release, finished = threading.Event(), threading.Event(), threading.Event()
        calls = []
        def slow_catalog():
            calls.append(1)
            started.set()
            release.wait(4)
            finished.set()
            return []
        monkeypatch.setattr(recorder, 'sessions', slow_catalog)
        try:
            async with TestServer(create_app(hub, {}, recorder, tmp_path)) as server, ClientSession() as client:
                first = asyncio.create_task(client.get(server.make_url('/api/recordings')))
                second = None
                try:
                    assert await asyncio.to_thread(started.wait, 2)
                    second = asyncio.create_task(client.get(server.make_url('/api/recordings')))
                    response = await asyncio.wait_for(client.get(server.make_url('/api/snapshot')), timeout=1)
                    assert response.status == 200 and not finished.is_set()
                    response = await asyncio.wait_for(client.get(server.make_url('/api/control/status')), timeout=1)
                    assert response.status == 200
                finally:
                    release.set()
                    responses = await asyncio.gather(first, *([second] if second else []))
                assert all(response.status == 200 for response in responses)
                assert calls == [1]
        finally:
            release.set()
            recorder.close()
    asyncio.run(scenario())


def test_closed_recording_catalog_is_cached_and_file_changes_invalidate_it(tmp_path, monkeypatch):
    old = Recorder(tmp_path)
    hub = Hub()
    hub.put('runtime', 'state', {'state': 'IDLE'})
    old.append(hub.snapshot())
    old.close()
    live = Recorder(tmp_path)
    try:
        before = next(item for item in live.sessions() if item['id'] == old.session_id)
        connect, opened = sqlite3.connect, []
        def tracked(*args, **kwargs):
            opened.append(str(args[0]))
            return connect(*args, **kwargs)
        monkeypatch.setattr(sqlite3, 'connect', tracked)
        assert next(item for item in live.sessions() if item['id'] == old.session_id) == before
        assert not any(old.session_id in path for path in opened)
        with connect(old.path) as db:
            db.execute('UPDATE records SET received=received+10')
        after = next(item for item in live.sessions() if item['id'] == old.session_id)
        assert after['start'] == before['start'] + 10
    finally:
        live.close()
