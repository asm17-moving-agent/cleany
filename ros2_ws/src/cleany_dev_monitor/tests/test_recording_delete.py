"""Explicit recording deletion preserves open writers and unrelated evidence."""

import asyncio
import threading

import pytest
from aiohttp import ClientSession
from aiohttp.test_utils import TestServer

from cleany_dev_monitor.core import Hub, Recorder, RecordingInUseError
from cleany_dev_monitor.recording import defaults
from cleany_dev_monitor.server import create_app


def closed_recording(directory, *, bag=False):
    recorder = Recorder(directory)
    hub = Hub()
    hub.put('runtime', 'state', {'state': 'IDLE'})
    recorder.append(hub.snapshot())
    recorder.close()
    if bag:
        recorder.bag_path.mkdir(parents=True)
        (recorder.bag_path / 'metadata.yaml').write_text('recording fixture')
        (recorder.bag_path / 'nested').mkdir()
        (recorder.bag_path / 'nested' / 'data.db3').write_bytes(b'raw sensor data')
    return recorder


@pytest.mark.parametrize('bag', [False, True])
def test_delete_removes_only_selected_session_and_its_sidecars(tmp_path, bag):
    old = closed_recording(tmp_path, bag=bag)
    current = Recorder(tmp_path)
    settings = tmp_path / 'recording-settings.json'
    settings.write_text('preserve settings')
    jobs = tmp_path / 'operator' / 'jobs'
    jobs.mkdir(parents=True)
    (jobs / 'history.json').write_text('preserve command history')
    sidecars = [old.path.with_name(old.path.name + suffix) for suffix in ('-journal', '-wal', '-shm')]
    try:
        # Warm the closed-session summary cache before deleting it.
        assert next(s for s in current.sessions() if s['id'] == old.session_id)['deletable']
        # Stale sidecars must be removed without opening them as SQLite input.
        for sidecar in sidecars:
            sidecar.write_bytes(b'sidecar fixture')
        before = current.disk_bytes()
        current.delete_session(old.session_id)
        assert not old.path.exists() and not old.bag_path.exists()
        assert not any(p.exists() for p in sidecars)
        assert current.disk_bytes() < before
        assert [s['id'] for s in current.sessions()] == [current.session_id]
        assert settings.read_text() == 'preserve settings'
        assert (jobs / 'history.json').read_text() == 'preserve command history'
        assert current.path.exists() and current.db is not None
        with pytest.raises(FileNotFoundError):
            current.delete_session(old.session_id)
    finally:
        current.close()


def test_open_writer_is_protected_even_after_a_limit_stop(tmp_path):
    recorder = Recorder(tmp_path)
    try:
        assert not recorder.sessions()[0]['deletable']
        with pytest.raises(RecordingInUseError):
            recorder.delete_session(recorder.session_id)
        recorder.stop_recording('size limit reached')
        with pytest.raises(RecordingInUseError):
            recorder.delete_session(recorder.session_id)
        assert recorder.path.exists()
        recorder.close()
        assert recorder.sessions()[0]['deletable']
        recorder.delete_session(recorder.session_id)
        assert not recorder.path.exists()
    finally:
        recorder.close()


@pytest.mark.parametrize('session', ['../recording-settings', 'operator/jobs', 'not-a-uuid', 'A' * 32])
def test_delete_rejects_noncanonical_session_paths(tmp_path, session):
    recorder = closed_recording(tmp_path)
    with pytest.raises(ValueError):
        recorder.delete_session(session)
    assert recorder.path.exists()


@pytest.mark.parametrize('redirect', ['database', 'bags_root', 'bag'])
def test_delete_refuses_symlinked_recording_roots(tmp_path, redirect):
    root, outside = tmp_path / 'monitor', tmp_path / 'outside'
    outside.mkdir()
    evidence = outside / 'evidence'
    evidence.write_text('preserve outside data')
    recorder = closed_recording(root)
    if redirect == 'database':
        recorder.path.unlink()
        recorder.path.symlink_to(evidence)
    elif redirect == 'bags_root':
        recorder.bag_path.parent.symlink_to(outside, target_is_directory=True)
    else:
        recorder.bag_path.parent.mkdir()
        recorder.bag_path.symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError, match='심볼릭 링크'):
        recorder.delete_session(recorder.session_id)
    assert evidence.read_text() == 'preserve outside data'
    assert recorder.path.exists()


def test_delete_http_requires_origin_protects_writer_and_invalidates_catalog(tmp_path):
    async def scenario():
        old = closed_recording(tmp_path, bag=True)
        hub, recorder = Hub(), Recorder(tmp_path)
        try:
            async with TestServer(create_app(hub, {}, recorder, tmp_path)) as server, ClientSession() as client:
                origin = {'Origin': str(server.make_url('/')).rstrip('/')}
                url = server.make_url('/api/recordings/' + old.session_id)
                catalog = await (await client.get(server.make_url('/api/recordings'))).json()
                assert len(catalog['sessions']) == 2
                assert (await client.delete(url)).status == 403
                assert (await client.delete(url, headers={'Origin': 'http://other.invalid'})).status == 403
                assert old.path.exists() and old.bag_path.exists()
                active = server.make_url('/api/recordings/' + recorder.session_id)
                assert (await client.delete(active, headers=origin)).status == 409
                response = await client.delete(url, headers=origin)
                assert response.status == 200 and await response.json() == {'deleted': old.session_id}
                catalog = await (await client.get(server.make_url('/api/recordings'))).json()
                assert [s['id'] for s in catalog['sessions']] == [recorder.session_id]
                assert (await client.delete(url, headers=origin)).status == 404
                bad = server.make_url('/api/recordings/not-a-uuid')
                assert (await client.delete(bad, headers=origin)).status == 400
        finally:
            recorder.close()
    asyncio.run(scenario())


def test_failed_delete_keeps_catalog_entry_for_retry(tmp_path, monkeypatch):
    async def scenario():
        old = closed_recording(tmp_path, bag=True)
        hub, recorder = Hub(), Recorder(tmp_path)
        def denied(_):
            raise PermissionError('fixture cannot remove bag')
        monkeypatch.setattr('cleany_dev_monitor.core.shutil.rmtree', denied)
        try:
            async with TestServer(create_app(hub, {}, recorder, tmp_path)) as server, ClientSession() as client:
                response = await client.delete(server.make_url('/api/recordings/' + old.session_id),
                    headers={'Origin': str(server.make_url('/')).rstrip('/')})
                assert response.status == 500 and '다시 시도' in await response.text()
                assert old.path.exists() and old.bag_path.exists()
                catalog = await (await client.get(server.make_url('/api/recordings'))).json()
                assert old.session_id in {s['id'] for s in catalog['sessions']}
        finally:
            recorder.close()
    asyncio.run(scenario())


def test_limit_stopped_current_can_be_deleted_without_closing_sqlite_in_worker(tmp_path):
    async def scenario():
        hub, recorder = Hub(), Recorder(tmp_path)
        recorder.stop_recording('Recording size limit reached; evidence retained.')
        path = recorder.path
        assert recorder.db is not None and recorder.sessions()[0]['deletable']
        try:
            async with TestServer(create_app(hub, {}, recorder, tmp_path)) as server, ClientSession() as client:
                response = await client.delete(server.make_url('/api/recordings/' + recorder.session_id),
                    headers={'Origin': str(server.make_url('/')).rstrip('/')})
                assert response.status == 200
                assert not path.exists() and recorder.db is None
                assert recorder.reason  # Space release does not restart a stopped recording.
                assert (await client.get(server.make_url('/api/snapshot'))).status == 200
        finally:
            recorder.close()
    asyncio.run(scenario())


def test_slow_delete_does_not_block_live_requests_and_catalog_waits(tmp_path, monkeypatch):
    async def scenario():
        old = closed_recording(tmp_path)
        hub, recorder = Hub(), Recorder(tmp_path)
        started, release = threading.Event(), threading.Event()
        delete = recorder.delete_session
        def slow_delete(session):
            started.set()
            assert release.wait(4)
            delete(session)
        monkeypatch.setattr(recorder, 'delete_session', slow_delete)
        try:
            async with TestServer(create_app(hub, {}, recorder, tmp_path)) as server, ClientSession() as client:
                task = asyncio.create_task(client.delete(server.make_url('/api/recordings/' + old.session_id),
                    headers={'Origin': str(server.make_url('/')).rstrip('/')}))
                pending = None
                try:
                    assert await asyncio.to_thread(started.wait, 2)
                    pending = asyncio.create_task(client.get(server.make_url('/api/recordings')))
                    response = await asyncio.wait_for(client.get(server.make_url('/api/snapshot')), 1)
                    assert response.status == 200 and not task.done()
                    async with client.ws_connect(server.make_url('/ws')) as ws:
                        assert (await ws.receive_json(timeout=1))['initial']
                    assert not pending.done()
                finally:
                    release.set()
                    responses = await asyncio.gather(task, *([pending] if pending else []))
                assert responses[0].status == 200
                assert old.session_id not in {s['id'] for s in (await responses[1].json())['sessions']}
        finally:
            release.set()
            recorder.close()
    asyncio.run(scenario())


def test_deleting_old_recording_allows_explicit_restart_after_total_limit(tmp_path):
    async def scenario():
        old = closed_recording(tmp_path, bag=True)
        (old.bag_path / 'full.db3').write_bytes(b'x' * 256_000)
        hub, recorder = Hub(), Recorder(tmp_path, total_limit=200_000)
        assert recorder.reason and recorder.db is None
        try:
            async with TestServer(create_app(hub, {}, recorder, tmp_path)) as server, ClientSession() as client:
                response = await client.delete(server.make_url('/api/recordings/' + old.session_id),
                    headers={'Origin': str(server.make_url('/')).rstrip('/')})
                assert response.status == 200
                assert recorder.reason and recorder.db is None  # No automatic recording restart.
                response = await client.post(server.make_url('/api/recording/session'), json=defaults())
                assert response.status == 200 and not (await response.json())['reason']
                assert recorder.db is not None
                hub.put('runtime', 'state', {'state': 'RECORDING'})
                recorder.append(hub.snapshot(recorder.last))
                assert recorder.db.execute("SELECT count(*) FROM records WHERE kind='runtime'").fetchone()[0] > 0
        finally:
            recorder.close()
    asyncio.run(scenario())
