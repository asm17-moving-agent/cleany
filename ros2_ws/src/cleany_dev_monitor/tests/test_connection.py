"""Live WebSocket deltas preserve reconnect and source reset behavior."""
import asyncio

from aiohttp import ClientSession
from aiohttp.test_utils import TestServer

from cleany_dev_monitor.core import Hub
from cleany_logger import Recorder
from logger_fixture import create_app


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
