"""Shared authenticated HTTP fixture; command dispatch stays in memory."""
from contextlib import asynccontextmanager
import json

from aiohttp import ClientSession, CookieJar, web
from aiohttp.test_utils import TestServer

from cleany_dev_monitor.operator_http import attach_control
from cleany_dev_monitor.server import local_origin


class Port:
    def __init__(self):
        self.state = dict(schema=1, active_execution_id='', arm_return_active=False,
            recovery_active=False, backend_ready=True, inhibited=False, blocked=[],
            guard_fresh=True, guard={'state': 'ready'}, hardware_fault='', payload_attached=False)
        self.calls = []

    def snapshot(self):
        return dict(self.state)

    def cancel(self, execution_id, mode):
        self.calls.append((execution_id, mode))
        return dict(accepted=True, message='Requested')

    def close(self):
        pass


@asynccontextmanager
async def environment(tmp_path, monkeypatch, *, enabled=True):
    root = tmp_path / 'operator'
    root.mkdir()
    (root / 'enabled').touch()
    (root / 'key').write_text('test-private-key')
    (root / 'key').chmod(0o600)
    port, launched = Port(), []
    async def launch(path, repo):
        launched.append(json.loads(path.read_text()))
    monkeypatch.setattr('cleany_dev_monitor.operator_http.runtime_info', lambda: dict(
        managed=True, client_busy=False, pipeline_busy=False, processes={}))
    app = web.Application(middlewares=[local_origin])
    control = attach_control(app, dict(control_enabled=enabled, control_directory=str(root),
        control_repository_root=str(tmp_path), _operator_port_factory=lambda: port, _operator_launch=launch))
    async with TestServer(app) as server, ClientSession(cookie_jar=CookieJar(unsafe=True)) as client:
        url = lambda suffix: server.make_url('/api/control/' + suffix)
        origin = {'Origin': str(server.make_url('')).rstrip('/')}
        async def login(**changes):
            return await client.post(url('login'), json=dict(name='operator', key='test-private-key', **changes), headers=origin)
        yield control, port, launched, client, url, origin, login
