"""Exercise the container entrypoint without Docker, ROS or inference."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize(
    'real53,custom_model,override,sim_time,qos,expected_model,expected_qos',
    [
        (False, None, None, False, None, 'gemini-decision-1', 'best_effort'),
        (True, None, None, False, None, 'gemini-decision-1', 'best_effort'),
        (True, None, None, True, None, 'gemini-decision-1', 'best_effort'),
        (True, 'gemini-3.5-flash-lite', None, False, None,
         'gemini-3.5-flash-lite', 'best_effort'),
        (True, None, 'gemini-3.5-flash-lite', False, 'reliable',
         'gemini-3.5-flash-lite', 'reliable'),
        (True, 'gemini-3.5-flash-lite', 'gemini-decision-1', False, None,
         'gemini-decision-1', 'best_effort'),
    ],
)
def test_container_launch_selects_profile_model_and_camera_qos(
    tmp_path, real53, custom_model, override, sim_time, qos,
    expected_model, expected_qos,
):
    workspace = tmp_path / 'workspace'
    models = tmp_path / 'models'
    build = workspace / 'containers/vision/scripts/build-workspace'
    build.parent.mkdir(parents=True)
    build.write_text('#!/bin/sh\nexit 0\n')
    build.chmod(0o755)
    setup = workspace / 'ros2_ws/install-perception/setup.bash'
    setup.parent.mkdir(parents=True)
    setup.write_text('true\n')
    config = workspace / 'ros2_ws/src/cleany_perception/config'
    config.mkdir(parents=True)
    for name in ('yoloe_real32.yaml', 'yoloe_real53.yaml'):
        (config / name).write_bytes(
            (ROOT / 'ros2_ws/src/cleany_perception/config' / name).read_bytes())
    if real53:
        (models / 'real53/m/active').mkdir(parents=True)
    profile = None
    if custom_model:
        profile = config / 'custom profile.yaml'
        profile.write_text('perception_inspector:\n  ros__parameters:\n'
                           f'    gemini_model: {custom_model}\n')
    binaries = tmp_path / 'bin'
    binaries.mkdir()
    for name, body in {
        'python3': f'exec {shlex.quote(sys.executable)} "$@"',
        'ros2': f'exec {shlex.quote(sys.executable)} -c '
                + shlex.quote('import json,sys; print(json.dumps(sys.argv[1:]))')
                + ' "$@"',
    }.items():
        path = binaries / name
        path.write_text(f'#!/bin/sh\n{body}\n')
        path.chmod(0o755)
    source = (ROOT / 'containers/vision/scripts/run-perception').read_text()
    script = tmp_path / 'run-perception'
    script.write_text(source.replace('/workspace/cleany', str(workspace))
                      .replace('/models/yoloe', str(models)))
    environment = {key: value for key, value in os.environ.items()
                   if not key.startswith(('GEMINI_', 'YOLOE_', 'CLEANY_'))}
    environment.update(PATH=f'{binaries}:{environment["PATH"]}',
                       CLEANY_PERCEPTION_USE_SIM_TIME=str(sim_time).lower())
    if profile:
        environment['YOLOE_CONFIG_FILE'] = str(profile)
    if override:
        environment['GEMINI_MODEL'] = override
    if qos:
        environment['CLEANY_RGBD_QOS_RELIABILITY'] = qos
    result = subprocess.run(['bash', str(script)], env=environment,
                            check=True, capture_output=True, text=True)
    arguments = json.loads(result.stdout)
    assert arguments[:3] == ['launch', 'cleany_perception', 'inspect_scene.launch.py']
    parameters = dict(argument.split(':=', 1) for argument in arguments[3:])
    expected_profile = profile or config / (
        'yoloe_real53.yaml' if real53 else 'yoloe_real32.yaml')
    assert parameters['yoloe_config_file'] == str(expected_profile)
    assert parameters['gemini_model'] == expected_model
    assert parameters['rgbd_qos_reliability'] == expected_qos
    assert parameters['use_sim_time'] == str(sim_time).lower()
    assert parameters['detector_type'] == 'yoloe_gemini'
    assert parameters['segmenter_type'] == 'yoloe_seg'
