"""Persistent worker admission, reuse, cancellation and restart boundaries."""

import asyncio
from contextlib import asynccontextmanager
import json
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from uuid import uuid4

import pytest

from cleany_dev_monitor.operator_jobs import atomic_json, execute_job
from cleany_dev_monitor.operator_service import (
    DispatchUnavailable, DispatchUncertain, OperatorPorts, OperatorService, submit_job,
)


def job(root, *, action='inspect', created_at=None):
    (root / 'jobs').mkdir(exist_ok=True)
    (root / 'enabled').touch()
    request_id = str(uuid4())
    path = root / 'jobs' / f'{request_id}.json'
    atomic_json(path, dict(id=request_id, action=action, args={}, state='PENDING',
                          created_at=time.time() if created_at is None else created_at))
    return path


@asynccontextmanager
async def serving(root, runner):
    stop, ready = asyncio.Event(), asyncio.Event()
    service = OperatorService(root, root, runner)
    task = asyncio.create_task(service.serve(stop, ready.set))
    await asyncio.wait_for(ready.wait(), 2.)
    try:
        yield service
    finally:
        stop.set()
        await asyncio.wait_for(task, 3.)


def test_single_admission_repeated_id_and_completed_job_are_not_replayed(tmp_path):
    entered, release = threading.Event(), threading.Event()
    calls = []

    def run(path, repo):
        calls.append(path.stem)
        entered.set()
        assert release.wait(2.)
        atomic_json(path, {**json.loads(path.read_text()), 'state': 'SUCCEEDED'})

    async def scenario():
        async with serving(tmp_path, run) as service:
            first = job(tmp_path)
            second = job(tmp_path)
            try:
                await submit_job(first, tmp_path)
                assert await asyncio.to_thread(entered.wait, 1.)
                await submit_job(first, tmp_path)
                with pytest.raises(DispatchUnavailable, match='다른 웹 명령'):
                    await submit_job(second, tmp_path)
                assert calls == [first.stem]
            finally:
                release.set()
            await service.task
            completed = first.read_bytes()
            with pytest.raises(DispatchUnavailable, match='재실행'):
                await submit_job(first, tmp_path)
            assert first.read_bytes() == completed and calls == [first.stem]
            await submit_job(second, tmp_path)
            await service.task
            assert calls == [first.stem, second.stem]
    asyncio.run(scenario())


def test_restart_never_resumes_pending_or_running_files(tmp_path):
    old = job(tmp_path, created_at=time.time()-1)
    running = job(tmp_path, created_at=time.time()-1)
    atomic_json(running, {**json.loads(running.read_text()), 'state': 'RUNNING', 'owner_pid': 999999})
    calls = []

    async def scenario():
        async with serving(tmp_path, lambda path, repo: calls.append(path.stem)):
            for path in (old, running):
                with pytest.raises(DispatchUnavailable):
                    await submit_job(path, tmp_path)
            assert not calls
    asyncio.run(scenario())
    assert json.loads(old.read_text())['state'] == 'PENDING'
    assert json.loads(running.read_text())['state'] == 'RUNNING'


def test_uncertain_ack_keeps_durable_job_pending(tmp_path):
    async def scenario():
        async def drop(reader, writer):
            await reader.readline()
            writer.close()
            await writer.wait_closed()
        async with await asyncio.start_unix_server(drop, str(tmp_path / 'worker.sock')):
            path = job(tmp_path)
            with pytest.raises(DispatchUncertain):
                await submit_job(path, tmp_path)
            assert json.loads(path.read_text())['state'] == 'PENDING'
    asyncio.run(scenario())


def test_missing_worker_and_disabled_control_do_not_execute(tmp_path):
    async def scenario():
        path = job(tmp_path)
        with pytest.raises(DispatchUnavailable):
            await submit_job(path, tmp_path)
        async with serving(tmp_path, lambda *_: pytest.fail('Must not execute')):
            path = job(tmp_path)
            (tmp_path / 'enabled').unlink()
            with pytest.raises(DispatchUnavailable):
                await submit_job(path, tmp_path)
    asyncio.run(scenario())


def test_unhandled_finalization_error_stops_admission(tmp_path):
    def fail(*_):
        raise OSError('disk unavailable')
    async def scenario():
        async with serving(tmp_path, fail) as service:
            await submit_job(job(tmp_path), tmp_path)
            await service.task
            assert service.stopping
            with pytest.raises(DispatchUnavailable):
                await submit_job(job(tmp_path), tmp_path)
    asyncio.run(scenario())


def test_reused_port_cancels_current_request_and_accepts_next(tmp_path, monkeypatch):
    import cleany_dev_monitor.operator_ros as ros
    entered = threading.Event()

    class Port:
        calls = 0
        closed = False
        def snapshot(self):
            return dict(backend_ready=True, guard_fresh=True, guard={'state': 'ready'},
                        active_execution_id='', inhibited=False)
        def inspect_for_monitor(self, _job_id, cancelled):
            self.calls += 1
            entered.set()
            if self.calls == 1:
                deadline = time.monotonic()+2
                while not cancelled() and time.monotonic() < deadline:
                    time.sleep(.005)
                assert cancelled()
                raise InterruptedError('cancelled')
            return {'snapshot_id': 'second-request'}
        def close(self):
            self.closed = True

    port = Port()
    monkeypatch.setattr(ros, 'OperatorROS', lambda: pytest.fail('Must reuse the supplied port'))
    monkeypatch.setattr('cleany_dev_monitor.operator_jobs.runtime_info', lambda: dict(
        managed=True, pipeline_busy=False, processes={'manipulation_action': True}))
    monkeypatch.delenv('CLEANY_CONTROL_RUNTIME_LOCKED', raising=False)
    monkeypatch.setenv('CLEANY_WEB_CONTROL_DIRECTORY', str(tmp_path))
    ports = SimpleNamespace(get=lambda _: port)

    async def scenario():
        async with serving(tmp_path, lambda path, repo: execute_job(path, repo, ports=ports)) as service:
            first = job(tmp_path)
            await submit_job(first, tmp_path)
            assert await asyncio.to_thread(entered.wait, 1.)
            first.with_suffix('.cancel').touch()
            await service.task
            assert json.loads(first.read_text())['state'] == 'INTERRUPTED'
            second = job(tmp_path)
            await submit_job(second, tmp_path)
            await service.task
            assert json.loads(second.read_text())['result'] == {'snapshot_id': 'second-request'}
            assert port.calls == 2 and not port.closed
            assert 'CLEANY_CONTROL_RUNTIME_LOCKED' not in os.environ
    asyncio.run(scenario())


def test_port_policy_reuse_and_replacement(tmp_path, monkeypatch):
    import cleany_dev_monitor.proposal_ros as ros
    created = []
    class Port:
        def __init__(self, root, policy):
            self.closed = False
            created.append(self)
        def close(self):
            self.closed = True
    monkeypatch.setattr(ros, 'ProposalROS', Port)
    ports = OperatorPorts(tmp_path)
    first = ports.get()
    assert ports.get() is first
    assert ports.get(dict(ports.policy)) is first
    second = ports.get({**ports.policy, 'proposal_debug_image_topic': '/different'})
    assert second is not first and first.closed
    assert len(created) == 2
    ports.close()
    assert second.closed


def test_http_uncertain_dispatch_does_not_overwrite_worker_result(tmp_path, monkeypatch):
    from operator_fixture import environment
    async def scenario():
        async with environment(tmp_path, monkeypatch) as (control, port, jobs, client, url, origin, login):
            async def uncertain(path, repo):
                atomic_json(path, {**json.loads(path.read_text()), 'state': 'SUCCEEDED'})
                raise DispatchUncertain('ack lost')
            control.config['_operator_launch'] = uncertain
            await login()
            request = dict(request_id=str(uuid4()), action='inspect', args={})
            response = await client.post(url('jobs'), json=request, headers=origin)
            assert response.status == 503
            response = await client.post(url('jobs'), json=request, headers=origin)
            assert response.status == 200 and (await response.json())['state'] == 'SUCCEEDED'
    asyncio.run(scenario())


def test_running_command_cancel_keeps_worker_available(tmp_path, monkeypatch):
    monkeypatch.setenv('CLEANY_WEB_CONTROL_DIRECTORY', str(tmp_path))
    monkeypatch.delenv('CLEANY_CONTROL_RUNTIME_LOCKED', raising=False)
    (tmp_path / 'tools').mkdir()
    marker = tmp_path / 'child.pid'
    script = tmp_path / 'tools/collection-runtime'
    script.write_text('#!/usr/bin/python3\nimport os,time\nfrom pathlib import Path\n'
                      f'Path({str(marker)!r}).write_text(str(os.getpid()))\ntime.sleep(30)\n')
    script.chmod(0o700)

    async def scenario():
        async with serving(tmp_path, execute_job) as service:
            path = job(tmp_path, action='prepare')
            data = json.loads(path.read_text())
            atomic_json(path, {**data, 'args': {'confirm_torque_on': True}})
            await submit_job(path, tmp_path)
            deadline = time.monotonic()+2
            while not marker.exists() and time.monotonic() < deadline:
                await asyncio.sleep(.005)
            assert marker.exists()
            path.with_suffix('.cancel').touch()
            await asyncio.wait_for(service.task, 3.)
            assert json.loads(path.read_text())['state'] == 'INTERRUPTED'
            with pytest.raises(ProcessLookupError):
                os.kill(int(marker.read_text()), 0)
            assert not service.stopping
            assert 'CLEANY_CONTROL_RUNTIME_LOCKED' not in os.environ
    asyncio.run(scenario())


def test_only_one_service_can_own_the_socket(tmp_path):
    async def scenario():
        async with serving(tmp_path, lambda *_: None):
            other = OperatorService(tmp_path, tmp_path, lambda *_: None)
            with pytest.raises(BlockingIOError):
                await other.serve(asyncio.Event())
            assert (tmp_path / 'worker.sock').exists()
    asyncio.run(scenario())
