"""HTTP resource collection remains live independently of recording settings."""
import asyncio

from aiohttp import ClientSession
from aiohttp.test_utils import TestServer

from cleany_dev_monitor.core import Hub
from cleany_logger import Recorder
from cleany_logger.resources import ResourceWorker
from logger_fixture import create_app


def test_http_worker_live_metrics_and_shutdown(tmp_path, monkeypatch):
    workers = []

    def worker(hub, config):
        instance = ResourceWorker(hub, config)
        workers.append(instance)
        return instance

    monkeypatch.setattr('cleany_dev_monitor.server.ResourceWorker', worker)

    async def run():
        hub = Hub()
        recorder = Recorder(tmp_path)
        try:
            app = create_app(hub, {"resource_interval_s": 0.5}, recorder, tmp_path)
            async with TestServer(app) as server, ClientSession() as client:
                response = await client.get(server.make_url('/api/recording/settings'))
                policy = await response.json()
                assert policy['observation']['resources']['summary'] == '수집 설정 0.5초 (2 Hz)'
                assert policy['labels']['resources'] == '호스트 리소스'
                settings = policy['settings']
                settings['active'] = False
                await client.put(server.make_url('/api/recording/settings'), json=settings)
                settings['enabled']['resources'] = False
                response = await client.put(server.make_url('/api/recording/settings'), json=settings)
                assert response.status == 200
                assert not (await response.json())['settings']['enabled']['resources']
                for _ in range(30):
                    response = await client.get(server.make_url('/api/snapshot'))
                    packet = await response.json()
                    resources = [r for r in packet['rows'] if r['kind'] == 'resources']
                    if resources and resources[0]['data'].get('cpu_percent') is not None:
                        break
                    await asyncio.sleep(0.1)
                assert resources[0]['data']['host']
                assert resources[0]['data']['cpu_percent'] is not None
                assert len(resources[0]['data']['history']) >= 2
            assert len(workers) == 1
            assert workers[0].stop.is_set()
            assert not workers[0].thread.is_alive()
        finally:
            recorder.close()

    asyncio.run(run())
