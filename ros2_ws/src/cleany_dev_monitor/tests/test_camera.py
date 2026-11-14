import asyncio

from aiohttp import ClientSession, web
from aiohttp.test_utils import TestServer
from cleany_dev_monitor.core import Hub
from cleany_logger import Recorder
from logger_fixture import create_app


def test_monitor_proxies_camera_without_owning_frames(tmp_path):
    async def scenario():
        hub=Hub()
        assert not hasattr(hub,'cameras')
        recorder=Recorder(tmp_path)
        replies=[]
        state={'status':200}
        async def camera(request):
            replies.append(request.query.get('topic'))
            return web.Response(status=state['status'],body=b'jpeg fixture',headers={
                'Content-Type':'image/jpeg','X-Frame-Age':'.1'})
        upstream=web.Application()
        upstream.router.add_get('/api/camera',camera)
        try:
            async with TestServer(upstream) as source:
                config={'camera_preview_url':str(source.make_url('')).rstrip('/')}
                async with TestServer(create_app(hub,config,recorder,tmp_path)) as monitor, ClientSession() as client:
                    url=monitor.make_url('/api/camera?topic=/head')
                    response=await client.get(url)
                    assert response.status==200 and await response.read()==b'jpeg fixture'
                    assert response.headers['X-Frame-Age']=='.1'
                    assert response.headers['Cache-Control']=='no-store'
                    assert replies==['/head'] and not hub.selections()
                    for status in (202,400,422,429):
                        state['status']=status
                        assert (await client.get(url)).status==status
                    state['status']=302
                    assert (await client.get(url)).status==503
                    await source.close()
                    assert (await client.get(url)).status==503
        finally: recorder.close()
    asyncio.run(scenario())
