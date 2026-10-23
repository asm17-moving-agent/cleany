"""Operator admission, durable requests and rollback without hardware commands."""
import asyncio
import fcntl
import json
import os
from pathlib import Path
import subprocess
import time
from uuid import uuid4

from aiohttp import ClientSession, CookieJar
import pytest

from cleany_dev_monitor.operator_http import OperatorControl
from cleany_dev_monitor.operator_jobs import alive, atomic_json, cancel_external_client, command_report, command_vectors, execute_job, process_start, runtime_info, runtime_settings
from cleany_dev_monitor.operator_policy import capabilities, validate_action
from operator_fixture import Port, environment


def test_authentication_origin_takeover_logout_and_feature_off(tmp_path, monkeypatch):
    async def scenario():
        async with environment(tmp_path, monkeypatch) as (control, port, jobs, client, url, origin, login):
            assert not (await (await client.get(url('status'))).json())['authenticated']
            request = dict(request_id=str(uuid4()), action='run', args={})
            assert (await client.post(url('jobs'), json=request, headers=origin)).status == 401
            assert (await client.post(url('login'), json={'key': 'bad'}, headers=origin)).status == 401
            assert (await client.post(url('login'), json=[], headers=origin)).status == 400
            assert (await client.post(url('login'), json={'key': '한글'}, headers=origin)).status == 401
            response = await login()
            assert response.status == 200
            cookie = response.cookies['cleany_operator']
            assert cookie['httponly'] and cookie['samesite'] == 'Strict'
            assert (await client.post(url('jobs'), json=request)).status == 403
            assert (await client.post(url('jobs'), json=request, headers={'Origin': 'http://other'})).status == 403
            async with ClientSession(cookie_jar=CookieJar(unsafe=True)) as other:
                data = dict(name='other', key='test-private-key')
                assert (await other.post(url('login'), json=data, headers=origin)).status == 409
                data['takeover'] = True
                assert (await other.post(url('login'), json=data, headers=origin)).status == 200
                assert (await client.post(url('jobs'), json=request, headers=origin)).status == 401
                assert (await other.post(url('logout'), json={}, headers=origin)).status == 200
                assert (await other.post(url('jobs'), json=request, headers=origin)).status == 401
            (control.root / 'enabled').unlink()
            assert await (await client.get(url('status'))).json() == dict(enabled=False, authenticated=False)
            assert (await login()).status == 404
            assert not jobs and not port.calls
    asyncio.run(scenario())


def test_disabled_never_creates_ros_clients(tmp_path):
    control = OperatorControl(dict(control_enabled=False, control_directory=str(tmp_path)))
    assert not control.enabled() and control.port is None


def test_requests_are_serialized_idempotent_and_revision_bound(tmp_path, monkeypatch):
    async def scenario():
        async with environment(tmp_path, monkeypatch) as (control, port, jobs, client, url, origin, login):
            await login()
            request = dict(request_id=str(uuid4()), action='run', args={})
            responses = await asyncio.gather(*[client.post(url('jobs'), json=request, headers=origin) for _ in range(2)])
            assert sorted(r.status for r in responses) == [200, 202] and len(jobs) == 1
            assert (await client.post(url('jobs'), json={**request, 'action': 'inspect'}, headers=origin)).status == 409
            assert (await client.post(url('jobs'), json={**request, 'request_id': str(uuid4())}, headers=origin)).status == 409
            path = control.jobs / f"{request['request_id']}.json"
            job = json.loads(path.read_text())
            atomic_json(path, {**job, 'state': 'FAILED'})
            port.state.update(inhibited=True, blocked=[dict(execution_id='old', revision=4)])
            args = dict(execution_id='old', revision=3, stopped=True, empty=True, note='Confirmed stopped and empty')
            recovery = dict(request_id=str(uuid4()), action='recover', args=args)
            assert (await client.post(url('jobs'), json=recovery, headers=origin)).status == 409
            args['revision'] = 4
            assert (await client.post(url('jobs'), json=recovery, headers=origin)).status == 202
            assert jobs[-1]['args']['revision'] == 4
            assert port.state['inhibited'] and not port.calls  # HTTP admission never clears a fault.
    asyncio.run(scenario())


def test_cancel_ack_is_never_stop_evidence_and_preflight_can_cancel(tmp_path, monkeypatch):
    async def scenario():
        async with environment(tmp_path, monkeypatch) as (control, port, jobs, client, url, origin, login):
            await login()
            port.state['active_execution_id'] = 'exact-live-execution'
            response = await client.post(url('cancel'), json={'mode': 'CHECKPOINT'}, headers=origin)
            assert (await response.json())['stop_confirmed'] is False
            assert port.calls == [('exact-live-execution', 'CHECKPOINT')]
            port.state['active_execution_id'] = ''
            request = dict(request_id=str(uuid4()), action='inspect', args={})
            assert (await client.post(url('jobs'), json=request, headers=origin)).status == 202
            response = await client.post(url('cancel'), json={'mode': 'IMMEDIATE'}, headers=origin)
            assert (await response.json())['accepted'] is True
            assert (control.jobs / f"{request['request_id']}.cancel").exists()
    asyncio.run(scenario())


def test_unknown_owner_blocks_new_work_and_acknowledges_only_audit(tmp_path, monkeypatch):
    async def scenario():
        async with environment(tmp_path, monkeypatch) as (control, port, jobs, client, url, origin, login):
            await login()
            control.jobs.mkdir()
            job = dict(id=str(uuid4()), action='run', args={}, actor='prior', state='RUNNING',
                       owner_pid=os.getpid(), owner_start='wrong-process-start', created_at=time.time())
            atomic_json(control.jobs / f"{job['id']}.json", job)
            response = await client.get(url('status'))
            assert (await response.json())['jobs'][0]['state'] == 'UNKNOWN'
            request = dict(request_id=str(uuid4()), action='run', args={})
            assert (await client.post(url('jobs'), json=request, headers=origin)).status == 409
            port.state.update(inhibited=True, blocked=[dict(execution_id='old', revision=7)])
            response = await client.post(url(f"jobs/{job['id']}/acknowledge"),
                json=dict(confirmed=True, note='Personally checked'), headers=origin)
            assert response.status == 200
            assert control.job_history()[0]['state'] == 'ACKNOWLEDGED'
            assert port.state['inhibited'] and not port.calls
    asyncio.run(scenario())


@pytest.mark.parametrize('action,args', [
    ('prepare', {}), ('shutdown', {'confirm_torque_off': 1}),
    ('return_arm', {'arm': 'left; touch /tmp/unsafe', 'confirm_open': True}),
    ('run', {'command': 'anything'}), ('shell', {}),
    ('recover', dict(execution_id='old', revision=True, stopped=True, empty=True, note='ok')),
])
def test_unconfirmed_or_arbitrary_commands_are_rejected(action, args):
    with pytest.raises(ValueError):
        validate_action(action, args)


def test_capabilities_fail_closed_and_preserve_recovery_path():
    state = dict(Port().state, available=True)
    assert not capabilities(state, busy=False, managed=True)['run']
    for change in ({'guard_fresh': False}, {'inhibited': True}, {'hardware_fault': 'overload'},
                   {'backend_ready': False}, {'active_execution_id': 'live'}, {'payload_attached': True}):
        assert capabilities({**state, **change}, busy=False, managed=True)['run']
    assert capabilities({}, busy=False, managed=False)['prepare'] == ''
    assert all(capabilities(state, busy=True, managed=True).values())
    state.update(hardware_fault='Operator software stop', guard=dict(state='stopped',
        reason='Operator software stop', controllers_stopped=True, actions_cancelled=True))
    assert not capabilities(state, busy=False, managed=True)['return_arm']
    assert capabilities(state, busy=False, managed=True)['run']


def test_pid_identity_lock_and_disabled_worker_never_runs_a_command(tmp_path, monkeypatch):
    assert alive(os.getpid()) and process_start(os.getpid())
    assert not alive(-1) and not alive(None)
    root = tmp_path / 'runtime'
    root.mkdir()
    (root / 'pids').write_text(f'manipulation_action {os.getpid()}\n')
    monkeypatch.setenv('CLEANY_COLLECTION_STATE_DIR', str(root))
    monkeypatch.setenv('CLEANY_WEB_CONTROL_DIRECTORY', str(tmp_path))
    with (tmp_path / 'operation.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert runtime_info()['client_busy'] and runtime_info()['managed']
    assert not runtime_info()['client_busy']
    jobs = tmp_path / 'jobs'
    jobs.mkdir()
    path = jobs / f'{uuid4()}.json'
    atomic_json(path, dict(action='prepare', args={'confirm_torque_on': True}, state='PENDING', created_at=time.time()))
    monkeypatch.setenv('CLEANY_CONTROL_RUNTIME_LOCKED', '')
    monkeypatch.setattr(subprocess, 'Popen', lambda *a, **kw: pytest.fail('Disabled worker must not execute'))
    execute_job(path, tmp_path)
    assert json.loads(path.read_text())['state'] == 'FAILED'
    (tmp_path / 'enabled').touch()
    atomic_json(path, dict(action='prepare', args={'confirm_torque_on': True}, state='PENDING', created_at=time.time()-30))
    execute_job(path, tmp_path)
    assert json.loads(path.read_text())['state'] == 'FAILED'
    assert command_vectors(Path('/repo'), 'return_arm', {'arm': 'right'}) == [
        ['/repo/tools/collection-runtime', 'arm-return', '--arm', 'right']]


def test_cli_lock_contends_before_start_without_touching_hardware(tmp_path, monkeypatch):
    for name in ('CLEANY_MANIPULATION_COLLECTION_CONFIG', 'CLEANY_FEETECH_CONTROL_CONFIG', 'CLEANY_HARDWARE_MONITOR_CONFIG'):
        monkeypatch.setenv(name, __file__)
    (tmp_path / 'enabled').touch()
    repo = Path(__file__).resolve().parents[4]
    source = (repo / 'tools/collection-runtime').read_text()
    guard = source.split('command="${1:-}"', 1)[1].rsplit('case "${command}" in', 1)[0]
    # Exercise the real admission code in a harmless harness, with no runtime functions.
    harness = tmp_path / 'runtime-lock'
    harness.write_text('#!/usr/bin/env bash\nset -euo pipefail\ncommand="${1:-}"\n' + guard + '\nexit 93\n')
    harness.chmod(0o700)
    with (tmp_path / 'operation.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        response = subprocess.run([str(harness), 'start'],
            env={**os.environ, 'CLEANY_CONTROL_RUNTIME_LOCKED': '0', 'CLEANY_WEB_CONTROL_DIRECTORY': str(tmp_path)},
            capture_output=True, timeout=5)
        assert response.returncode == 75, response.stderr.decode()
    (tmp_path / 'enabled').unlink()
    assert subprocess.run([str(harness), 'start'], env={**os.environ,
        'CLEANY_WEB_CONTROL_DIRECTORY': str(tmp_path)}, timeout=5).returncode == 93


def test_worker_uses_existing_lifecycle_and_honors_cancel_before_start(tmp_path, monkeypatch):
    monkeypatch.setenv('CLEANY_CONTROL_RUNTIME_LOCKED', '')
    monkeypatch.setenv('CLEANY_WEB_CONTROL_DIRECTORY', str(tmp_path))
    (tmp_path / 'enabled').touch()
    (tmp_path / 'jobs').mkdir()
    (tmp_path / 'tools').mkdir()
    script = tmp_path / 'tools/collection-runtime'
    calls = tmp_path / 'calls'
    script.write_text('#!/usr/bin/python3\nimport sys\nfrom pathlib import Path\n'
        f'with Path({str(calls)!r}).open("a") as stream: stream.write(sys.argv[1]+"\\n")\n')
    script.chmod(0o700)
    path = tmp_path / 'jobs' / f'{uuid4()}.json'
    job = dict(action='prepare', args={'confirm_torque_on': True}, state='PENDING', created_at=time.time())
    atomic_json(path, job)
    execute_job(path, tmp_path)
    assert json.loads(path.read_text())['state'] == 'SUCCEEDED'
    assert calls.read_text().splitlines() == ['start', 'action-start']
    path.with_suffix('.cancel').touch()
    atomic_json(path, {**job, 'created_at': time.time()})
    execute_job(path, tmp_path)
    assert json.loads(path.read_text())['state'] == 'INTERRUPTED'
    assert calls.read_text().splitlines() == ['start', 'action-start']


def test_external_cancel_only_signals_verified_pipeline_groups(tmp_path, monkeypatch):
    monkeypatch.setenv('CLEANY_COLLECTION_STATE_DIR', str(tmp_path))
    with subprocess.Popen(['sleep', '15'], start_new_session=True) as unrelated:
        try:
            (tmp_path / 'pipeline.pid').write_text(str(unrelated.pid))
            assert not cancel_external_client() and unrelated.poll() is None
        finally:
            unrelated.terminate()
    with subprocess.Popen(['bash', '-c', 'exec -a manipulation_test_client sleep 15'], start_new_session=True) as client:
        try:
            (tmp_path / 'pipeline.pid').write_text(str(client.pid))
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                if Path(f'/proc/{client.pid}/cmdline').read_bytes().split(b'\0')[0] == b'manipulation_test_client':
                    break
                time.sleep(.01)
            assert cancel_external_client()
            assert client.wait(timeout=2) == -2
        finally:
            if client.poll() is None:
                client.terminate()


def test_failed_job_reports_its_terminal_execution_and_preserves_audit(tmp_path):
    control = OperatorControl(dict(control_directory=str(tmp_path)))
    control.jobs.mkdir()
    path = control.jobs / f'{uuid4()}.json'
    job = dict(id=path.stem, action='run', args={}, state='FAILED', message='Command exited 1')
    atomic_json(path, job)
    record = dict(execution_id='physical-execution', has_result=True, status='BLOCKED',
        error_code='TARGET_UNAVAILABLE', failed_substage='SelectArmAndPath',
        object_id=3, object_state='NOT_TOUCHED', stop_confirmed=True,
        message='No reachable commissioned arm')
    path.with_suffix('.log').write_text(json.dumps(dict(found=True, record=record))+'\n[ros2run]: Process exited with failure 1\n')
    reported = control.job_history()[0]
    assert reported['state'] == 'FAILED'
    assert reported['result']['execution_id'] == 'physical-execution'
    assert reported['result']['status'] == 'BLOCKED'
    assert reported['result']['object_state'] == 'NOT_TOUCHED'
    assert '양팔' in reported['message']
    assert json.loads(path.read_text()) == job  # Summaries do not rewrite history or clear blocks.


def test_partial_or_invalid_stdout_never_becomes_a_terminal_record(tmp_path):
    path = tmp_path / 'command.log'
    for payload in (dict(status='SUCCESS'), dict(found=True, record=dict(
            execution_id='unfinished', has_result=False, status='SUCCESS'))):
        path.write_text(json.dumps(payload)+'\n')
        assert command_report(path) == {}
    path.write_text('Collection readiness failed: Duplicate ROS node names: /duplicate\n')
    assert 'Duplicate ROS node names' in command_report(path)['message']


def test_web_run_and_cli_use_the_same_real_make_recipe_and_options(tmp_path):
    repo = Path(__file__).resolve().parents[4]
    (tmp_path / 'tools').mkdir()
    script = tmp_path / 'tools/collection-runtime'
    script.write_text('#!/usr/bin/python3\nimport json,os,sys\n'
        'print(json.dumps(dict(args=sys.argv[1:],cwd=os.getcwd(),query=os.environ.get("COLLECTION_QUERY"),'
        'config=os.environ.get("CLEANY_MANIPULATION_COLLECTION_CONFIG"),'
        'backend=os.environ.get("CLEANY_COLLECTION_MOTION_BACKEND"))))\n')
    script.chmod(0o700)
    override = f'REPO_ROOT={tmp_path}/'
    env = {**os.environ, 'COLLECTION_QUERY': 'visible objects with spaces',
        'CLEANY_MANIPULATION_COLLECTION_CONFIG': '/config/robot with spaces.json',
        'CLEANY_COLLECTION_MOTION_BACKEND': 'moveit'}
    cli = ['make', '--no-print-directory', '-C', str(repo), 'manipulation-run']
    web_command = command_vectors(repo, 'run', {})[0]
    cli_result = subprocess.run(cli + [override], env=env, capture_output=True, text=True, check=True, timeout=5)
    web_result = subprocess.run(web_command + [override], env=env, capture_output=True, text=True, check=True, timeout=5)
    assert cli_result.stdout == web_result.stdout
    result = json.loads(web_result.stdout.splitlines()[-1])
    assert result == dict(args=['action-run'], cwd=str(repo), query=env['COLLECTION_QUERY'],
        config=env['CLEANY_MANIPULATION_COLLECTION_CONFIG'], backend='moveit')


def test_saved_cli_options_are_loaded_without_accepting_unrelated_environment(tmp_path):
    assert runtime_settings(tmp_path) == {}
    path = tmp_path / 'runtime-settings.json'
    values = dict(GEMINI_API_KEY='test-secret', COLLECTION_QUERY='custom query',
        CLEANY_COLLECTION_MOTION_BACKEND='moveit', CLEANY_COLLECTION_GRASP_PREDICTOR='anygrasp',
        CLEANY_MANIPULATION_COLLECTION_CONFIG='/custom/robot.json', BASH_ENV='/untrusted',
        CLEANY_CONTROL_RUNTIME_LOCKED='1')
    atomic_json(path, values)
    loaded = runtime_settings(tmp_path)
    assert loaded['COLLECTION_QUERY'] == 'custom query'
    assert loaded['CLEANY_MANIPULATION_COLLECTION_CONFIG'] == '/custom/robot.json'
    assert loaded['CLEANY_COLLECTION_MOTION_BACKEND'] == 'moveit'
    assert loaded['CLEANY_COLLECTION_GRASP_PREDICTOR'] == 'anygrasp'
    assert 'BASH_ENV' not in loaded and 'CLEANY_CONTROL_RUNTIME_LOCKED' not in loaded
    path.chmod(0o644)
    with pytest.raises(ValueError, match='0600'):
        runtime_settings(tmp_path)


def test_control_on_captures_cli_options_without_starting_services(tmp_path):
    repo = Path(__file__).resolve().parents[4]
    source = (repo / 'tools/dev-monitor').read_text().split("<<'PY'\n", 1)[1].split('\nPY\n', 1)[0]
    root = tmp_path / 'operator'
    root.mkdir()
    env = {**os.environ, 'PYTHONPATH': str(repo/'ros2_ws/src/cleany_dev_monitor'),
        'GEMINI_API_KEY': 'test-secret', 'COLLECTION_QUERY': 'literal object; $(not executed)',
        'CLEANY_COLLECTION_MOTION_BACKEND': 'moveit',
        'CLEANY_MANIPULATION_COLLECTION_CONFIG': 'configs/custom robot.json'}
    result = subprocess.run(['python3', '-', str(root), 'control-on'], input=source,
        env=env, capture_output=True, text=True, check=True, timeout=5)
    captured = runtime_settings(root)
    assert captured['COLLECTION_QUERY'] == env['COLLECTION_QUERY']
    assert captured['CLEANY_MANIPULATION_COLLECTION_CONFIG'] == env['CLEANY_MANIPULATION_COLLECTION_CONFIG']
    assert captured['CLEANY_COLLECTION_MOTION_BACKEND'] == 'moveit'
    assert (root / 'enabled').is_file()
    assert (root / 'runtime-settings.json').stat().st_mode & 0o077 == 0
    assert 'test-secret' not in result.stdout + result.stderr


def test_reenabling_control_from_a_fresh_shell_preserves_profiles_and_explicit_overrides(tmp_path):
    from cleany_dev_monitor.operator_jobs import RUNTIME_ENV_KEYS
    repo = Path(__file__).resolve().parents[4]
    source = (repo / 'tools/dev-monitor').read_text().split("<<'PY'\n", 1)[1].split('\nPY\n', 1)[0]
    root = tmp_path / 'operator'
    root.mkdir()
    previous = {
        'CLEANY_MANIPULATION_COLLECTION_CONFIG': '/local/collection profile.json',
        'CLEANY_FEETECH_CONTROL_CONFIG': '/local/control.yaml',
        'CLEANY_HARDWARE_MONITOR_CONFIG': '/local/monitor.yaml',
        'CLEANY_COLLECTION_MOTION_BACKEND': 'moveit',
        'CLEANY_COLLECTION_GRASP_PREDICTOR': 'geometric',
        'COLLECTION_QUERY': 'saved objects', 'GEMINI_API_KEY': 'saved-test-secret',
        'BASH_ENV': '/untrusted', 'CLEANY_CONTROL_RUNTIME_LOCKED': '1',
    }
    atomic_json(root / 'runtime-settings.json', previous)
    env = {k: v for k, v in os.environ.items() if k not in RUNTIME_ENV_KEYS}
    env['PYTHONPATH'] = str(repo / 'ros2_ws/src/cleany_dev_monitor')
    for overrides in ({}, {'CLEANY_COLLECTION_GRASP_PREDICTOR': 'anygrasp',
                           'COLLECTION_QUERY': 'explicit new query'}):
        result = subprocess.run(['python3', '-', str(root), 'control-on'], input=source,
            env={**env, **overrides}, capture_output=True, text=True, check=True, timeout=5)
        expected = {**{k: v for k, v in previous.items() if k in RUNTIME_ENV_KEYS}, **overrides}
        assert runtime_settings(root) == expected
        assert json.loads((root / 'runtime-settings.json').read_text()) == expected
        assert (root / 'runtime-settings.json').stat().st_mode & 0o077 == 0
        assert 'saved-test-secret' not in result.stdout + result.stderr
