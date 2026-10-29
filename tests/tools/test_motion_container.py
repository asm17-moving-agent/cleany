"""Exercise launch arguments and lifecycle without Docker or GPU access."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest
import yaml

from motion_container import Runtime, WORKSPACE, runtime
from cleany_skill_executor.motion_runtime import resolve_motion_runtime, selector_prefix


@pytest.mark.parametrize('machine,jetson,backend,expected', [
    ('aarch64', True, 'curobo_ik_moveit', 'container'),
    ('aarch64', True, 'curobo', 'container'),
    ('aarch64', True, 'moveit', 'native'),
    ('x86_64', False, 'curobo_ik_moveit', 'native'),
    ('aarch64', False, 'curobo_ik_moveit', 'native'),
])
def test_auto_policy(machine, jetson, backend, expected):
    assert resolve_motion_runtime(backend, machine=machine, jetson_release=jetson) == expected


@pytest.mark.parametrize('backend', ['moveit', 'curobo', 'curobo_ik_moveit'])
@pytest.mark.parametrize('requested', ['native', 'container'])
def test_explicit_runtime(backend, requested):
    assert resolve_motion_runtime(backend, requested) == requested


def test_native_prefix_preserves_custom_python_and_needs_no_docker(tmp_path):
    import shlex
    python = str(tmp_path / 'custom python')
    assert shlex.split(selector_prefix('curobo', 'native', tmp_path, python)) == [python]
    assert selector_prefix('moveit', 'auto', tmp_path, python) == ''


def test_packaged_sphere_asset_invalidates_prepared_workspace(tmp_path):
    asset = tmp_path/'ros2_ws/src/cleany_skill_executor/cleany_skill_executor/data/spheres.json'
    asset.parent.mkdir(parents=True)
    asset.write_text('{"radius":0.01}')
    entrypoint = tmp_path/'containers/motion/runtime-entrypoint'
    entrypoint.parent.mkdir(parents=True)
    entrypoint.write_text('runtime')
    settings = Runtime(tmp_path)
    before = settings.fingerprint()
    asset.write_text('{"radius":0.02}')
    assert settings.fingerprint() != before


@pytest.mark.parametrize('launch_file', ['physical_grasp_selection', 'pregrasp_runtime',
    'pregrasp_hardware_support',
    'study_cafe_sorting', 'validate_pick', 'pregrasp_real_readonly'])
def test_launches_expose_shared_runtime_and_custom_python(launch_file, tmp_path):
    import importlib.util
    import shlex
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from cleany_skill_executor.motion_runtime import motion_runtime_prefix

    root = Path(__file__).resolve().parents[2]
    path = root / 'ros2_ws/src/cleany_skill_executor/launch' / (launch_file + '.launch.py')
    spec = importlib.util.spec_from_file_location(launch_file, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(repository_root=str(tmp_path / 'repo with spaces'),
                                        profile_file='/unused/profile.json')
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert context.launch_configurations['motion_runtime'] == 'auto'
    context.launch_configurations.update(motion_backend='curobo_ik_moveit', motion_runtime='native',
                                        curobo_python=str(tmp_path / 'python with spaces'))
    assert shlex.split(motion_runtime_prefix().perform(context)) == [str(tmp_path / 'python with spaces')]
    context.launch_configurations['motion_runtime'] = 'container'
    assert shlex.split(motion_runtime_prefix().perform(context)) == [str(tmp_path / 'repo with spaces/tools/launch-motion-container')]


def test_parameter_files_and_inline_paths_are_copied(tmp_path):
    root = tmp_path / 'repo with spaces'
    root.mkdir()
    settings = Runtime(root)
    source = tmp_path / 'launch params.yaml'
    config = {'/**': {'ros__parameters': {
        'nested': [str(root / 'ros2_ws/install/cleany_description/share/model.xacro'),
                   str(root / 'configs/robot/data.json')], 'use_sim_time': True}}}
    source.write_text(yaml.safe_dump(config))
    args = settings.parameters(['--ros-args', '--params-file', str(source),
        '--params-file=' + str(source), '-p', 'config_file:=' + str(root / 'configs/custom.yaml')], tmp_path)
    assert args[1:5] == ['--params-file', '/run/cleany-motion/parameters-1.yaml',
                         '--params-file', '/run/cleany-motion/parameters-3.yaml']
    result = yaml.safe_load((tmp_path / 'parameters-1.yaml').read_text())['/**']['ros__parameters']
    assert result['nested'] == [WORKSPACE + '/install/cleany_description/share/model.xacro',
                               '/workspace/cleany/configs/robot/data.json']
    assert result['use_sim_time'] is True
    assert args[-1] == 'config_file:=/workspace/cleany/configs/custom.yaml'
    with pytest.raises(ValueError, match='Missing ROS'):
        settings.parameters(['--params-file'], tmp_path)


def test_domain_and_dds_files_are_forwarded(tmp_path):
    profile = tmp_path / 'DDS profile.xml'
    profile.write_text('<profiles/>')
    settings = Runtime(tmp_path)
    args = settings.environment({'ROS_DOMAIN_ID': '53', 'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'FASTRTPS_DEFAULT_PROFILES_FILE': str(profile), 'CYCLONEDDS_URI': '<CycloneDDS/>',
        'PYTHONPATH': '/host/install', 'SECRET': 'not-forwarded'})
    assert 'ROS_DOMAIN_ID=53' in args
    assert 'RMW_IMPLEMENTATION=rmw_fastrtps_cpp' in args
    assert f'{profile}:{profile}:ro' in args
    assert 'CYCLONEDDS_URI=<CycloneDDS/>' in args
    assert not any('PYTHONPATH' in arg or 'SECRET' in arg for arg in args)


def test_unset_rmw_uses_host_default_instead_of_image_default(tmp_path, monkeypatch):
    monkeypatch.setattr('motion_container.default_host_rmw', lambda: 'host_default_rmw')
    args = Runtime(tmp_path).environment({})
    assert 'RMW_IMPLEMENTATION=host_default_rmw' in args
    assert 'ROS_DOMAIN_ID=0' in args
    assert 'OPENBLAS_NUM_THREADS=1' in args


@pytest.mark.parametrize('threads', ['1', '2', '8'])
def test_blas_budget_overrides_inherited_host_default(tmp_path, threads):
    args = Runtime(tmp_path).environment({
        'CLEANY_MOTION_BLAS_THREADS': threads, 'OPENBLAS_NUM_THREADS': '32',
        'OMP_NUM_THREADS': '8',
    })
    assert f'OPENBLAS_NUM_THREADS={threads}' in args
    assert 'OPENBLAS_NUM_THREADS=32' not in args
    assert not any(arg.startswith('OMP_NUM_THREADS=') for arg in args)


@pytest.mark.parametrize('threads', ['0', '-1', 'invalid'])
def test_invalid_blas_budget_fails_before_launch(tmp_path, threads):
    with pytest.raises(ValueError):
        Runtime(tmp_path).environment({'CLEANY_MOTION_BLAS_THREADS': threads})


@pytest.fixture
def fake_docker(tmp_path):
    binary = tmp_path / 'docker'
    binary.write_text('''#!/usr/bin/env python3
import json, os, sys, time
args = sys.argv[1:]
with open(os.environ['FAKE_DOCKER_LOG'], 'a') as output:
    output.write(json.dumps(args) + '\\n')
if args[:2] == ['image', 'inspect']:
    print('sha256:test')
elif args[0] == 'run':
    print(os.environ['FAKE_RECEIPT'])
elif args[0] == 'start':
    if os.environ.get('FAKE_WAIT') == '1':
        time.sleep(60)
    sys.exit(7)
''')
    binary.chmod(0o755)
    log = tmp_path / 'calls.jsonl'
    environment = dict(os.environ, PATH=str(tmp_path) + ':' + os.environ['PATH'],
        FAKE_DOCKER_LOG=str(log),
        FAKE_RECEIPT=json.dumps({'image_id': 'sha256:test', 'source_sha256': runtime().fingerprint()}),
        ROS_DOMAIN_ID='53', CLEANY_MOTION_OWNER='test owner')
    for key in ('CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
        environment.pop(key, None)
    return environment, log


@pytest.mark.parametrize('cancel', [None, signal.SIGINT, signal.SIGTERM])
def test_launch_propagates_failure_or_cancel_and_cleans_only_its_container(fake_docker, cancel):
    environment, log = fake_docker
    if cancel is not None:
        environment['FAKE_WAIT'] = '1'
    script = (Path(__file__).resolve().parents[2] / 'tools').joinpath('launch-motion-container')
    process = subprocess.Popen([sys.executable, str(script), '/host/install/physical_grasp_selection_server',
        '--ros-args', '-r', '__node:=custom_selector'], env=environment,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        if cancel is not None:
            deadline = time.monotonic() + 10
            while not log.exists() or not any(json.loads(line)[0] == 'start' for line in log.read_text().splitlines()):
                assert time.monotonic() < deadline and process.poll() is None
                time.sleep(.02)
            process.send_signal(cancel)
        stdout, stderr = process.communicate(timeout=15)
        assert process.returncode == (7 if cancel is None else 128 + cancel), stderr
        calls = [json.loads(line) for line in log.read_text().splitlines()]
        create = next(call for call in calls if call[0] == 'create')
        name = create[create.index('--name') + 1]
        assert name.startswith('cleany-motion-selector-')
        assert 'ROS_DOMAIN_ID=53' in create and 'cleany.owner=test owner' in create
        assert 'OPENBLAS_NUM_THREADS=1' in create
        assert '--network' in create and 'host' in create
        assert create[create.index('--user') + 1] == f'{os.getuid()}:{os.getgid()}'
        assert create[create.index('--pid') + 1] == 'host'
        assert 'cleany_skill_executor.physical_grasp_selection_node' in create
        assert '__node:=custom_selector' in create
        assert ['stop', '--time', '3', name] in calls
        assert ['rm', '--force', name] in calls
        assert 'MOTION_CONTAINER' in stdout
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()


def test_stale_workspace_fails_without_starting_selector(fake_docker):
    environment, log = fake_docker
    environment['FAKE_RECEIPT'] = '{}'
    result = subprocess.run([sys.executable, str((Path(__file__).resolve().parents[2] / 'tools').joinpath('launch-motion-container')),
        '/host/physical_grasp_selection_server'], env=environment, capture_output=True, text=True)
    assert result.returncode == 1
    assert 'make motion-container-image motion-container-prepare' in result.stderr
    assert not any(json.loads(line)[0] in ('create', 'start') for line in log.read_text().splitlines())


def test_cleanup_still_removes_its_container_after_stop_timeout(tmp_path, monkeypatch):
    calls = []
    def run(args, **kwargs):
        calls.append(args)
        if args[1] == 'stop':
            raise subprocess.TimeoutExpired(args, kwargs['timeout'])
        return subprocess.CompletedProcess(args, 0)
    monkeypatch.setattr('motion_container.subprocess.run', run)
    Runtime(tmp_path).cleanup('owned-selector')
    assert calls == [['docker', 'stop', '--time', '3', 'owned-selector'],
                     ['docker', 'rm', '--force', 'owned-selector']]
