"""Exercise collection ownership routing using fake external commands only."""
import os
from pathlib import Path
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[2]


def run_functions(tmp_path, body, *, shared=False, recorded_shared=None, recorded_motion=None):
    source = (ROOT/'tools/collection-runtime').read_text()
    library = tmp_path/'library.sh'
    library.write_text(source.split('\ncommand="${1:-}"\n', 1)[0])
    repo = tmp_path/'repo'
    tools = repo/'tools'
    tools.mkdir(parents=True)
    for name in ['vision-container', 'motion-container', 'feetech-torque']:
        path = tools/name
        path.write_text('#!/bin/bash\necho "$(basename "$0") $*" >> "$TRACE"\n')
        path.chmod(0o755)
    state = tmp_path/'state'
    state.mkdir()
    if recorded_shared is not None:
        (state/'perception-mode').write_text(str(recorded_shared).lower()+'\n')
    if recorded_motion is not None:
        (state/'motion-runtime').write_text(recorded_motion+'\n')
    script = f'''source "{library}"
repo_root="{repo}"
use_existing_perception={str(shared).lower()}
start_group() {{ echo "GROUP:$*"; }}
{body}
'''
    env = dict(os.environ, CLEANY_COLLECTION_STATE_DIR=str(state), TRACE=str(tmp_path/'trace'),
               CLEANY_MANIPULATION_COLLECTION_CONFIG=str(tmp_path/'collection.json'),
               CLEANY_COLLECTION_MOTION_BACKEND='curobo_ik_moveit',
               CLEANY_COLLECTION_GRASP_PREDICTOR='geometric')
    result = subprocess.run(['bash', '-c', script], env=env, text=True, capture_output=True)
    trace = (tmp_path/'trace').read_text() if (tmp_path/'trace').exists() else ''
    return result, trace


@pytest.mark.parametrize('shared', [False, True])
def test_shared_vision_does_not_start_or_stop_the_existing_provider(tmp_path, shared):
    result, trace = run_functions(tmp_path, 'start_vision ignored.log', shared=shared)
    assert result.returncode == 0, result.stderr
    if shared:
        assert trace == '' and 'GROUP:' not in result.stdout
        assert 'Reusing existing perception' in result.stdout
    else:
        assert 'vision-container hybrid-down' in trace
        assert 'vision-container perception-up' in trace
        assert 'GROUP:vision' in result.stdout


@pytest.mark.parametrize('shared', [False, True])
def test_camera_sharing_preserves_native_gpu_launch(tmp_path, shared):
    result, _ = run_functions(tmp_path, 'start_hardware_support ignored.log ignored', shared=shared)
    assert result.returncode == 0, result.stderr
    assert 'motion_backend:=curobo_ik_moveit' in result.stdout
    assert 'motion_runtime:=native' in result.stdout
    assert ('start_head_camera:=false' if shared else 'start_head_camera:=true') in result.stdout


@pytest.mark.parametrize('motion', ['native', 'container'])
def test_shutdown_preserves_recorded_shared_provider_even_from_a_default_shell(tmp_path, motion):
    body = '''managed_processes_alive() { return 1; }
execution_owners_alive() { return 1; }
wait_for_execution_shutdown() { return 0; }
source_ros() { :; }
timeout() { :; }
fuser() { return 1; }
stop_runtime'''
    result, trace = run_functions(tmp_path, body, recorded_shared=True, recorded_motion=motion)
    assert result.returncode == 0, result.stderr
    assert 'vision-container' not in trace
    assert ('motion-container stop' in trace) == (motion == 'container')
    assert 'feetech-torque off' in trace


def test_existing_runtime_cannot_change_provider_ownership_in_place(tmp_path):
    body = '''printf 'curobo_ik_moveit geometric\\n' > "$planning_mode_file"
printf 'native\\n' > "$motion_runtime_file"
validate_planning_mode
require_matching_planning_mode'''
    result, _ = run_functions(tmp_path, body, shared=True, recorded_shared=False)
    assert result.returncode == 1
    assert 'perception ownership differs' in result.stderr
