"""Runtime lifecycle and startup checks with no ROS or motor access."""
from pathlib import Path
import subprocess

import pytest
import os


@pytest.fixture(autouse=True)
def explicit_local_collection_profile(monkeypatch):
    profile = Path(__file__).resolve().parents[2] / 'ros2_ws/src/cleany_skill_executor/test/data/supervised_collection_fixture.json'
    monkeypatch.setenv('CLEANY_MANIPULATION_COLLECTION_CONFIG', str(profile))
    root = Path(__file__).resolve().parents[2]
    monkeypatch.setenv('CLEANY_FEETECH_CONTROL_CONFIG', str(root / 'ros2_ws/src/cleany_robot_interface/test/data/feetech_control.yaml'))
    monkeypatch.setenv('CLEANY_HARDWARE_MONITOR_CONFIG', str(root / 'ros2_ws/src/cleany_skill_executor/test/data/hardware_motion_monitor_fixture.yaml'))


def test_collection_endpoint_limits_share_one_profile():
    import yaml

    root = Path(__file__).resolve().parents[2]
    hardware = yaml.safe_load((root / 'ros2_ws/src/cleany_skill_executor/test/data/hardware_motion_monitor_fixture.yaml').read_text())
    limits = hardware['pregrasp_pipeline']['ros__parameters']
    controller = yaml.safe_load((root / 'ros2_ws/src/cleany_robot_interface/test/data/feetech_control.yaml').read_text())['controller']
    assert limits['hardware_arrival_tolerance_rad'] == controller['goal_tolerance'] == .10
    assert limits['hardware_motion_response_error_rad'] == limits['hardware_arrival_tolerance_rad']
    assert limits['hardware_tcp_position_tolerance_m'] == .030
    assert limits['hardware_tcp_orientation_tolerance_deg'] == 20.
    script = (root / 'tools/collection-runtime').read_text()
    execute = script.split('execute_once() {', 1)[1].split('start_action_server() {', 1)[0]
    assert '--params-file "${CLEANY_HARDWARE_MONITOR_CONFIG}"' in execute
    assert '-p hardware_tcp_position_tolerance_m:=' not in execute
    assert '-p hardware_tcp_orientation_tolerance_deg:=' not in execute
    assert '-p hardware_velocity_rad_s:=' not in execute
    assert '-p hardware_acceleration_rad_s2:=' not in execute


SCRIPT = (Path(__file__).resolve().parents[2] / 'tools').joinpath('collection-runtime')


@pytest.mark.parametrize('failure', ['', 'startup', 'execution'])
def test_action_command_reports_preparation_before_startup_and_keeps_exit_status(tmp_path, failure):
    import json
    definitions = SCRIPT.read_text().partition('\ncommand="${1:-}"\n')[0]
    shell = tmp_path / 'functions.sh'
    shell.write_text(definitions)
    progress = tmp_path / 'request.json'
    program = r'''
source "$1"
repo_root="$2"
export CLEANY_MANIPULATION_PROGRESS_FILE="$3"
failure="$4"
start_runtime() {
  python3 -c 'import json,sys; assert json.load(open(sys.argv[1]))["phase"] == "RUNTIME_PREPARING"' "${CLEANY_MANIPULATION_PROGRESS_FILE}"
  [[ "$failure" != startup ]]
}
start_action_server() { :; }
source_ros() { :; }
run_foreground() {
  printf '%s\n' "$@"
  [[ "$failure" != execution ]]
}
action_run_once
'''
    result = subprocess.run(['bash', '-c', program, 'progress-test', str(shell), str(SCRIPT.parent.parent), str(progress), failure],
                            text=True, capture_output=True, timeout=8)
    assert result.returncode == (0 if not failure else 1), result.stderr
    data = json.loads(progress.read_text())
    assert data['state'] == ('FINISHED' if not failure else 'FAILED')
    assert data['history'][0]['phase'] == 'RUNTIME_PREPARING'
    if failure != 'startup':
        args = result.stdout.splitlines()
        assert args[args.index('--execution-id') + 1] == data['execution_id']


def test_insufficient_dds_buffer_blocks_start_before_hardware_or_gpu(tmp_path):
    definitions = SCRIPT.read_text().partition('\ncommand="${1:-}"\n')[0]
    shell = tmp_path / 'runtime-functions.sh'
    shell.write_text(definitions)
    program = r'''
source "$1"
state_dir="$2/state"
check_dds_transport() { echo BUFFER_REJECTED; return 1; }
prepare_motion_runtime() { echo UNEXPECTED_GPU; }
start_real_control() { echo UNEXPECTED_MOTOR; }
if start_runtime; then exit 99; fi
test ! -e "$state_dir"
'''
    result = subprocess.run(['bash', '-c', program, 'dds-start-test', str(shell), str(tmp_path)],
                            text=True, capture_output=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == 'BUFFER_REJECTED'


@pytest.mark.parametrize('failure', ['', 'dead', 'client', 'stalled', 'health'])
def test_action_restart_preserves_control_and_does_not_replace_a_stalled_owner(tmp_path, failure):
    definitions, _, _ = SCRIPT.read_text().partition('\ncommand="${1:-}"\n')
    shell = tmp_path / 'runtime-functions.sh'
    shell.write_text(definitions)
    pids = tmp_path / 'pids'
    pids.write_text('real_control 42\nmanipulation_action 43\nvision 44\n')
    marker = tmp_path / 'action-alive'
    if failure != 'dead':
        marker.touch()
    pipeline = tmp_path / 'pipeline.pid'
    if failure == 'client':
        pipeline.write_text('42')
    transport = r'''
source "$1"
pids_file="$2/pids"
pipeline_pid_file="$2/pipeline.pid"
marker="$2/action-alive"
failure="$3"
execution_stop_timeout=1
require_runtime() { :; }
alive() {
  if [[ "$1" == 43 ]]; then [[ -f "$marker" ]]; else [[ "$1" == 42 || "$1" == 44 ]]; fi
}
kill() { echo "SIGNAL $*"; [[ "$failure" == stalled ]] || rm -f "$marker"; }
sleep() { SECONDS=$((SECONDS + 1)); }
check_managed_health() { echo HEALTH; [[ "$failure" != health ]]; }
start_action_server() { echo STARTED; }
restart_action_server
'''
    result = subprocess.run(['bash', '-c', transport, 'restart-test', str(shell), str(tmp_path), failure],
                            text=True, capture_output=True, timeout=5)
    assert result.returncode == (1 if failure in ('client', 'stalled', 'health') else 0), result.stderr
    assert ('STARTED' in result.stdout) == (failure in ('', 'dead'))
    assert 'real_control 42\n' in pids.read_text() and 'vision 44\n' in pids.read_text()
    assert 'SIGNAL -INT -- -42' not in result.stdout
    if failure in ('client', 'stalled'):
        assert 'manipulation_action 43\n' in pids.read_text()
    else:
        assert 'manipulation_action' not in pids.read_text()


@pytest.mark.parametrize('arm', ['left', 'right', 'both'])
def test_arm_return_prepares_server_and_forwards_selected_arm(tmp_path, arm):
    definitions, _, _ = SCRIPT.read_text().partition('\ncommand="${1:-}"\n')
    shell = tmp_path / 'runtime-functions.sh'
    shell.write_text(definitions)
    transport = '''
source "$1"
require_runtime() { echo RUNTIME; }
check_managed_health() { echo HEALTH; }
start_action_server() { echo SERVER; }
source_ros() { :; }
arm_return_control_status() { echo CONTROL; }
run_foreground() { printf '%s\\n' "$@"; }
return_arms --arm "$2"
'''
    result = subprocess.run(['bash', '-c', transport, 'arm-return-test', str(shell), arm],
                            text=True, capture_output=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == ['RUNTIME', 'HEALTH', 'CONTROL', 'SERVER',
        'ros2', 'run', 'cleany_manipulation_bt', 'arm_return', '--arm', arm]


def test_return_probe_reads_only_the_managed_control_process_log(tmp_path):
    definitions, _, _ = SCRIPT.read_text().partition('\ncommand="${1:-}"\n')
    shell = tmp_path / 'runtime-functions.sh'
    shell.write_text(definitions)
    (tmp_path / 'pids').write_text('vision 41\nreal_control 42\nmanipulation_action 43\n')
    transport = r'''
source "$1"
pids_file="$2/pids"
run_foreground() { printf '%s\n' "$@"; }
arm_return_control_status --arm right
'''
    result = subprocess.run(['bash', '-c', transport, 'return-probe-test', str(shell), str(tmp_path)],
                            text=True, capture_output=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == ['python3', '-m',
        'cleany_manipulation_bt.arm_return_client', '--check-control', '--control-log',
        '/proc/42/fd/1', '--arm', 'right']


@pytest.mark.parametrize('case', [
    'software_stop', 'cached_fault', 'stop_during_preflight', 'hard_fault', 'stale_guard',
    'owner_stalled', 'control_failed', 'server_failed', 'motion_failed', 'second_fault',
    'recovered_motion_failed', 'client_running',
])
def test_arm_return_recovers_once_only_before_motion_and_drains_the_owner(tmp_path, case):
    definitions, _, _ = SCRIPT.read_text().partition('\ncommand="${1:-}"\n')
    shell = tmp_path / 'runtime-functions.sh'
    shell.write_text(definitions)
    (tmp_path / 'pipeline.pid').write_text('42')
    transport = r'''
source "$1"
pipeline_pid_file="$2/pipeline.pid"
case="$3"
probe_count=0
motion_count=0
require_runtime() { :; }
check_managed_health() { :; }
source_ros() { :; }
alive() { [[ "$case" == client_running ]]; }
arm_return_control_status() {
  probe_count=$((probe_count + 1))
  echo "PROBE $probe_count $*"
  case "$case" in
    hard_fault|stale_guard) return 1 ;;
    cached_fault|motion_failed|second_fault|server_failed) return 0 ;;
    stop_during_preflight) [[ "$probe_count" -eq 1 ]] && return 0 ;;
  esac
  return 20
}
stop_action_server() { echo DRAIN; [[ "$case" != owner_stalled ]]; }
recover_control() { echo RECOVER_CONTROL; [[ "$case" != control_failed ]]; }
start_action_server() { echo SERVER; [[ "$case" != server_failed ]]; }
run_foreground() {
  motion_count=$((motion_count + 1))
  echo "REQUEST $motion_count $*"
  case "$case" in
    cached_fault|stop_during_preflight) [[ "$motion_count" -gt 1 ]] || return 21 ;;
    second_fault) return 21 ;;
    motion_failed|recovered_motion_failed) return 1 ;;
  esac
}
return_arms --arm right
'''
    result = subprocess.run(['bash', '-c', transport, 'return-recovery-test',
        str(shell), str(tmp_path), case], text=True, capture_output=True, timeout=5)
    success = case in ('software_stop', 'cached_fault', 'stop_during_preflight')
    assert (result.returncode == 0) == success, result.stdout + result.stderr
    lines = result.stdout.splitlines()
    assert lines.count('RECOVER_CONTROL') <= 1
    assert lines.count('DRAIN') <= 1
    if case in ('hard_fault', 'stale_guard', 'client_running'):
        assert not any(word in result.stdout for word in ('DRAIN', 'RECOVER_CONTROL', 'REQUEST'))
    elif case == 'owner_stalled':
        assert lines[-1] == 'DRAIN' and 'RECOVER_CONTROL' not in lines
    elif case == 'control_failed':
        assert lines[-1] == 'RECOVER_CONTROL' and 'SERVER' not in lines
    elif case == 'server_failed':
        assert not any(line.startswith('REQUEST') for line in lines)
    else:
        requests = [line for line in lines if line.startswith('REQUEST')]
        assert len(requests) == (2 if case in ('cached_fault', 'stop_during_preflight', 'second_fault') else 1)
        assert all(line.endswith('--arm right') for line in requests)
        if case in ('software_stop', 'stop_during_preflight', 'recovered_motion_failed'):
            assert lines.index('DRAIN') < lines.index('RECOVER_CONTROL') < lines.index(requests[-1])
        elif case == 'cached_fault':
            assert 'RECOVER_CONTROL' not in lines
        elif case == 'motion_failed':
            assert 'DRAIN' not in lines


def test_control_recovery_does_not_mask_failed_readiness_inside_conditional(tmp_path):
    definitions, _, _ = SCRIPT.read_text().partition('\ncommand="${1:-}"\n')
    shell = tmp_path / 'runtime-functions.sh'
    shell.write_text(definitions)
    (tmp_path / 'pids').write_text('real_control 42\nvision 44\n')
    transport = r'''
source "$1"
pids_file="$2/pids"
test_dir="$2"
require_runtime() { :; }
alive() { return 1; }
fuser() { return 1; }
source_ros() { :; }
current_run_dir() { echo "$test_dir"; }
start_real_control() { echo START_CONTROL; }
wait_for_graph() { echo READINESS_FAILED; return 7; }
recover_control || exit $?
'''
    result = subprocess.run(['bash', '-c', transport, 'recovery-failure-test', str(shell), str(tmp_path)],
                            text=True, capture_output=True, timeout=5)
    assert result.returncode == 7, result.stdout + result.stderr
    assert 'READINESS_FAILED' in result.stdout and 'REAL CONTROL RECOVERED' not in result.stdout


@pytest.mark.parametrize('backend,predictor,runtime', [
    ('', '', 'container'),
    ('moveit', 'geometric', 'native'),
    ('moveit', 'anygrasp', 'native'),
    ('curobo_ik_moveit', 'geometric', 'container'),
    ('curobo_ik_moveit', 'anygrasp', 'container'),
])
def test_hardware_support_keeps_automatic_arm_default_and_valid_launch_values(
        tmp_path, backend, predictor, runtime):
    definitions, separator, _ = SCRIPT.read_text().partition('\ncommand="${1:-}"\n')
    assert separator
    shell = tmp_path / 'runtime-functions.sh'
    shell.write_text(definitions)
    transport = r'''
start_group() { printf '%s\n' "$@"; }
start_hardware_support /tmp/hardware-support.log /tmp/collection-test
'''
    result = subprocess.run(
        ['bash', '-c', 'export CLEANY_COLLECTION_MOTION_BACKEND="$2"\n'
         'export CLEANY_COLLECTION_GRASP_PREDICTOR="$3"\nsource "$1"\n' + transport,
         'hardware-support-test', str(shell), backend, predictor],
        text=True, capture_output=True, timeout=5,
    )
    assert result.returncode == 0, result.stderr
    arguments = result.stdout.splitlines()
    assert arguments[2:6] == [
        'ros2', 'launch', 'cleany_skill_executor', 'pregrasp_hardware_support.launch.py',
    ]
    launch_arguments = arguments[6:]
    assert 'maximum_candidates:=32' in launch_arguments
    for argument in launch_arguments:
        name, separator, value = argument.partition(':=')
        assert name and separator and value, argument
    assert not any(argument.startswith('required_arm:=') for argument in launch_arguments)
    assert f'motion_runtime:={runtime}' in launch_arguments
    assert f'motion_backend:={backend or "curobo_ik_moveit"}' in launch_arguments
    assert f'grasp_predictor:={predictor or "geometric"}' in launch_arguments
    assert f'collection_config_file:={os.environ["CLEANY_MANIPULATION_COLLECTION_CONFIG"]}' in launch_arguments


@pytest.mark.parametrize('backend,predictor', [
    ('', ''),
    ('moveit', 'geometric'), ('moveit', 'anygrasp'),
    ('curobo_ik_moveit', 'geometric'), ('curobo_ik_moveit', 'anygrasp'),
])
def test_planning_mode_starts_only_required_gpu_services(tmp_path, backend, predictor):
    definitions, _, _ = SCRIPT.read_text().partition('\ncommand="${1:-}"\n')
    shell = tmp_path / 'runtime-functions.sh'
    shell.write_text(definitions)
    directory = tmp_path / 'tools'
    directory.mkdir()
    for name in ('vision-container', 'motion-container'):
        script = directory / name
        script.write_text('#!/bin/bash\nprintf "%s\\n" "' + name + ' $*"\n')
        script.chmod(0o755)
    transport = r'''
export CLEANY_COLLECTION_MOTION_BACKEND="$3"
export CLEANY_COLLECTION_GRASP_PREDICTOR="$4"
source "$1"
repo_root="$2"
start_group() { printf '%s\n' "$*"; }
validate_planning_mode
prepare_motion_runtime
start_vision /tmp/vision.log
'''
    result = subprocess.run(['bash', '-c', transport, 'planning-mode-test',
        str(shell), str(tmp_path), backend, predictor], text=True, capture_output=True, timeout=5)
    assert result.returncode == 0, result.stderr
    backend = backend or 'curobo_ik_moveit'
    predictor = predictor or 'geometric'
    assert ('motion-container check' in result.stdout) == (backend == 'curobo_ik_moveit')
    assert ('motion-container stop' in result.stdout) == (backend == 'curobo_ik_moveit')
    assert ('vision-container hybrid-up' in result.stdout) == (predictor == 'anygrasp')
    assert ('hybrid-run' in result.stdout) == (predictor == 'anygrasp')
    assert ('vision-container perception-up' in result.stdout) == (predictor == 'geometric')
    assert ('perception-run' in result.stdout) == (predictor == 'geometric')


@pytest.mark.parametrize('backend,recorded_mode,returncode', [
    ('moveit', 'moveit geometric', 0), ('moveit', 'curobo_ik_moveit anygrasp', 1),
    ('moveit', None, 1), ('', 'curobo_ik_moveit geometric', 0),
    ('', 'moveit geometric', 1), ('', 'curobo_ik_moveit anygrasp', 1),
])
def test_running_runtime_does_not_silently_reuse_a_different_mode(tmp_path, backend, recorded_mode, returncode):
    definitions, _, _ = SCRIPT.read_text().partition('\ncommand="${1:-}"\n')
    shell = tmp_path / 'runtime-functions.sh'
    shell.write_text(definitions)
    if recorded_mode is not None:
        (tmp_path / 'planning-mode').write_text(recorded_mode + '\n')
    transport = r'''
export CLEANY_COLLECTION_MOTION_BACKEND="$3" CLEANY_COLLECTION_GRASP_PREDICTOR=geometric
export CLEANY_COLLECTION_STATE_DIR="$2"
source "$1"
managed_processes_alive() { return 0; }
check_dds_transport() { return 0; }
source_ros() { echo ROS; }
wait_for_graph() { echo CHECKED; }
start_runtime
'''
    result = subprocess.run(['bash', '-c', transport, 'mode-reuse-test', str(shell), str(tmp_path), backend],
                            text=True, capture_output=True, timeout=5)
    assert result.returncode == returncode, result.stderr
    assert ('CHECKED' in result.stdout) == (returncode == 0)
    if returncode:
        assert 'make collection-stop' in result.stderr


@pytest.mark.parametrize('backend,predictor', [('curobo', 'geometric'), ('moveit', 'unknown')])
def test_invalid_planning_mode_fails_before_starting_support(tmp_path, backend, predictor):
    definitions, _, _ = SCRIPT.read_text().partition('\ncommand="${1:-}"\n')
    shell = tmp_path / 'runtime-functions.sh'
    shell.write_text(definitions)
    transport = r'''
export CLEANY_COLLECTION_MOTION_BACKEND="$2" CLEANY_COLLECTION_GRASP_PREDICTOR="$3"
source "$1"
start_group() { echo STARTED; }
if start_hardware_support /tmp/support.log /tmp/run; then exit 90; else exit $?; fi
'''
    result = subprocess.run(['bash', '-c', transport, 'invalid-mode-test', str(shell), backend, predictor],
                            text=True, capture_output=True, timeout=5)
    assert result.returncode == 2
    assert 'STARTED' not in result.stdout


@pytest.mark.parametrize('failure', ['', 'runtime', 'server', 'client'])
def test_one_command_starts_runtime_server_then_uses_actual_scene_selection(tmp_path, failure):
    definitions, separator, _ = SCRIPT.read_text().partition('\ncommand="${1:-}"\n')
    assert separator
    shell = tmp_path / 'runtime-functions.sh'
    shell.write_text(definitions)
    transport = r'''
start_runtime() { echo RUNTIME; [[ "$failure" != runtime ]]; }
start_action_server() { echo SERVER; [[ "$failure" != server ]]; }
source_ros() { :; }
run_foreground() { printf '%s\n' "$@"; if [[ "$failure" == client ]]; then return 7; fi; }
failure="$2"
action_run_once
'''
    result = subprocess.run(['bash', '-c', 'source "$1"\n' + transport, 'test', str(shell), failure],
                            text=True, capture_output=True)
    if failure == 'client':
        assert result.returncode == 7 and '--auto-select' in result.stdout
        assert '"event":"collection_command_timing"' in result.stdout
        assert '"exit_code":7' in result.stdout
    elif failure:
        assert result.returncode != 0 and '--auto-select' not in result.stdout
    else:
        assert result.returncode == 0
        assert result.stdout.index('RUNTIME') < result.stdout.index('SERVER') < result.stdout.index('--auto-select')
        assert '--collection-config' in result.stdout and 'mock-snapshot' not in result.stdout


TRANSPORTS = r'''
gui=false
start_timeout=30
health_checks=0
check_managed_health() {
  health_checks=$((health_checks + 1))
  [[ "$failure" != health && ! ( "$failure" == health_after_feedback && "$health_checks" -gt 1 ) ]]
}
python3() {
  [[ "$1" == */runtime_readiness.py && "$2" == --timeout && "$3" == 30 ]] || return 90
  [[ -z "$failure" || "$failure" == health_after_feedback ]]
}
sleep() { command sleep .01; }
if ! wait_for_graph; then
  exit 23
fi
printf '%s\n' READY
'''



def readiness(tmp_path: Path, failure: str) -> subprocess.CompletedProcess[str]:
    # Load function definitions without executing the real CLI dispatch.
    definitions, separator, _ = SCRIPT.read_text().partition('\ncommand="${1:-}"\n')
    assert separator
    shell = tmp_path / 'runtime-functions.sh'
    shell.write_text(definitions)
    return subprocess.run(
        ['bash', '-c', 'source "$1"; failure="$2"; ' + TRANSPORTS,
         'collection-readiness-test', str(shell), failure],
        capture_output=True, text=True, timeout=5,
    )


def test_readiness_accepts_complete_healthy_feedback(tmp_path):
    result = readiness(tmp_path, '')
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == 'READY'


@pytest.mark.parametrize('failure', [
    'health', 'health_after_feedback', 'action_query', 'parameter_query', 'controller_query',
    'service_list_query', 'action_list_query', 'node_list_query',
    'guard_query', '/joint_states', '/camera/depth/image_raw',
    '/perception/scene_cloud_receipt', 'duplicate_node', 'duplicate_action',
    'many_action_servers', 'moveit_execution_enabled', 'wrong_parameter_type',
    'inactive_controller', 'guard_fault',
])
def test_readiness_rejects_any_failed_required_check(tmp_path, failure):
    result = readiness(tmp_path, failure)
    assert result.returncode == 23, (result.stdout, result.stderr)
    assert 'READY' not in result.stdout


def test_reused_runtime_never_claims_ready_after_failed_preflight(tmp_path):
    definitions, _, _ = SCRIPT.read_text().partition('\ncommand="${1:-}"\n')
    shell = tmp_path / 'runtime-functions.sh'
    shell.write_text(definitions)
    transport = r'''
state_dir="$2"
managed_processes_alive() { return 0; }
require_matching_planning_mode() { return 0; }
check_dds_transport() { return 0; }
source_ros() { :; }
wait_for_graph() { return 1; }
if ! start_runtime; then exit 23; fi
'''
    result = subprocess.run(['bash', '-c', 'source "$1"\n' + transport,
                            'failed-reuse', str(shell), str(tmp_path)],
                           text=True, capture_output=True, timeout=5)
    assert result.returncode == 23
    assert 'COLLECTION RUNTIME READY' not in result.stdout


@pytest.mark.parametrize('failure', ['', 'missing_snapshot', 'legacy_running', 'duplicate_action'])
def test_action_start_requires_snapshot_and_one_server(tmp_path, failure):
    definitions, separator, _ = SCRIPT.read_text().partition('\ncommand="${1:-}"\n')
    assert separator
    shell = tmp_path / 'runtime-functions.sh'
    shell.write_text(definitions)
    pids = tmp_path / 'pids'
    pids.write_text('')
    pipeline = tmp_path / 'pipeline.pid'
    pipeline.write_text('123')
    transport = r'''
pids_file="$3"
pipeline_pid_file="$4"
require_runtime() { :; }
check_managed_health() { :; }
source_ros() { :; }
current_run_dir() { printf '%s\n' /tmp/action-start-test; }
alive() { [[ "$failure" == legacy_running ]]; }
start_group() { printf '%s\n' STARTED; }
timeout() { shift; "$@"; }
sleep() { SECONDS=$((SECONDS + 31)); }
ros2() {
  case "$1 $2" in
    'service list')
      if [[ "$failure" != missing_snapshot ]]; then
        printf '%s\n' /perception/get_scene_snapshot
      fi ;;
    'action info')
      if [[ "$failure" == duplicate_action ]]; then
        printf '%s\n' 'Action servers: 2'
      else
        printf '%s\n' 'Action servers: 1'
      fi ;;
    *) return 90 ;;
  esac
}
start_action_server
'''
    result = subprocess.run(
        ['bash', '-c', 'source "$1"; failure="$2"; ' + transport,
         'action-start-test', str(shell), failure, str(pids), str(pipeline)],
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == (0 if not failure else 1), (result.stdout, result.stderr)
    if failure in ('missing_snapshot', 'legacy_running'):
        assert 'STARTED' not in result.stdout
    if not failure:
        assert 'waiting for approved object Goals' in result.stdout


@pytest.mark.parametrize('failure', ['', 'runtime', 'health'])
def test_recovery_checks_runtime_before_ros_and_preserves_arguments(tmp_path, failure):
    definitions, separator, _ = SCRIPT.read_text().partition('\ncommand="${1:-}"\n')
    assert separator
    shell = tmp_path / 'runtime-functions.sh'
    shell.write_text(definitions)
    transport = r'''
require_runtime() { echo RUNTIME; [[ "$failure" != runtime ]]; }
check_managed_health() { echo HEALTH; [[ "$failure" != health ]]; }
source_ros() { echo ROS; }
ros2() { printf '%s\n' "$@"; }
failure="$2"
confirm_action_recovery --status --note 'operator checked stop and grippers'
'''
    result = subprocess.run(['bash', '-c', 'source "$1"\n' + transport,
        'recovery-test', str(shell), failure], text=True, capture_output=True, timeout=5.)
    assert result.returncode == (1 if failure else 0), result.stderr
    lines = result.stdout.splitlines()
    if failure:
        assert 'ROS' not in lines and 'manipulation_recovery' not in lines
    else:
        assert lines == ['RUNTIME', 'HEALTH', 'ROS', 'run', 'cleany_manipulation_bt',
                         'manipulation_recovery', '--status', '--note',
                         'operator checked stop and grippers']


@pytest.mark.parametrize('pids', [None, '', 'real_control 99999999\n'])
def test_recovery_cli_without_running_runtime_has_no_dds_traceback(tmp_path, monkeypatch, pids):
    monkeypatch.setenv('CLEANY_COLLECTION_STATE_DIR', str(tmp_path))
    if pids is not None:
        (tmp_path / 'pids').write_text(pids)
    result = subprocess.run([str(SCRIPT), 'action-recover', '--status'],
                            text=True, capture_output=True, timeout=5.)
    assert result.returncode == 1
    assert 'make collection-start' in result.stderr
    assert 'Traceback' not in result.stderr and 'rmw_create_node' not in result.stderr


@pytest.mark.parametrize('failure', [
    '', 'stalled_action', 'stalled_pipeline', 'stalled_pipeline_missing_pids', 'term_recovery',
])
def test_stop_keeps_feedback_alive_until_execution_drains_and_retains_stalled_pids(tmp_path, failure):
    definitions, separator, _ = SCRIPT.read_text().partition('\ncommand="${1:-}"\n')
    assert separator
    shell = tmp_path / 'runtime-functions.sh'
    shell.write_text(definitions)
    state = tmp_path / 'state'
    state.mkdir()
    pids = state / 'pids'
    pids.write_text('manipulation_action 302\nreal_control 303\nvision 304\n')
    if failure == 'stalled_pipeline_missing_pids':
        pids.unlink()
    (state / 'pipeline.pid').write_text('301')
    tools = tmp_path / 'tools'
    tools.mkdir()
    for name in ('vision-container', 'motion-container', 'feetech-torque', 'ros2'):
        tool = tools / name
        tool.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "' + name + ' $*" >> "$AUDIT_EVENT_LOG"\n')
        tool.chmod(0o755)
    log = tmp_path / 'events'
    transport = r'''
failure="$2"
state_dir="$3/state"
pids_file="$state_dir/pids"
pipeline_pid_file="$state_dir/pipeline.pid"
repo_root="$3"
export AUDIT_EVENT_LOG="$3/events"
export PATH="$3/tools:$PATH"
execution_stop_timeout=2
source_ros() { :; }
fuser() { return 1; }
timeout() { while [[ "$1" == -* ]]; do shift; done; shift; "$@"; }
declare -A live=([301]=1 [302]=1 [303]=1 [304]=1)
alive() { [[ "${live[$1]:-0}" == 1 ]]; }
kill() {
  local pid="${*: -1}"
  pid="${pid#-}"
  printf '%s\n' "$1 $pid" >> "$AUDIT_EVENT_LOG"
  if [[ "$pid" == 301 || "$pid" == 302 ]]; then
    if [[ "$failure" == term_recovery && "$1" == -TERM ]]; then
      live[$pid]=0
    fi
  else
    live[$pid]=0
  fi
}
sleep() {
  SECONDS=$((SECONDS + 1))
  if [[ "$failure" != stalled_pipeline* && "$failure" != term_recovery ]]; then
    live[301]=0
  fi
  if [[ "$failure" != stalled_action && "$failure" != term_recovery ]]; then
    live[302]=0
  fi
  if ! execution_owners_alive; then
    printf '%s\n' OWNERS_FINISHED >> "$AUDIT_EVENT_LOG"
  fi
}
stop_runtime
'''
    result = subprocess.run(['bash', '-c', 'source "$1"; ' + transport,
        'stop-runtime-test', str(shell), failure, str(tmp_path)],
        capture_output=True, text=True, timeout=5.)
    assert result.returncode == (1 if failure.startswith('stalled') else 0), result.stderr
    events = log.read_text().splitlines()
    if not failure:
        assert events.index('OWNERS_FINISHED') < events.index('-INT 303'), events
    else:
        assert 'Physical stop is unconfirmed' in result.stderr
    assert 'feetech-torque off' in events
    if failure.startswith('stalled'):
        assert (state / 'pipeline.pid').exists()
        if failure == 'stalled_pipeline_missing_pids':
            assert not pids.exists()
        else:
            assert pids.read_text()
        assert 'PID records retained' in result.stderr
        assert 'COLLECTION RUNTIME STOPPED' not in result.stdout
    else:
        assert not pids.read_text() and not (state / 'pipeline.pid').exists()


@pytest.mark.parametrize('command,missing', [
    ('start', 'CLEANY_MANIPULATION_COLLECTION_CONFIG'),
    ('start', 'CLEANY_FEETECH_CONTROL_CONFIG'),
    ('start', 'CLEANY_HARDWARE_MONITOR_CONFIG'),
    ('action-start', 'CLEANY_MANIPULATION_COLLECTION_CONFIG'),
    ('action-run', 'CLEANY_MANIPULATION_COLLECTION_CONFIG'),
    ('execute', 'CLEANY_MANIPULATION_COLLECTION_CONFIG'),
    ('run', 'CLEANY_MANIPULATION_COLLECTION_CONFIG'),
    ('recover-control', 'CLEANY_FEETECH_CONTROL_CONFIG')])
def test_missing_local_profile_is_rejected_before_start(command, monkeypatch, tmp_path, missing):
    monkeypatch.delenv(missing, raising=False)
    monkeypatch.setenv('CLEANY_COLLECTION_STATE_DIR', str(tmp_path / 'runtime'))
    result = subprocess.run([str(SCRIPT), command], capture_output=True, text=True, timeout=5)
    assert result.returncode == 2
    assert 'explicit local hardware profile' in result.stderr
    assert not (tmp_path / 'runtime').exists()


def test_real_control_owns_motion_and_moveit_only_plans(tmp_path):
    definitions, separator, _ = SCRIPT.read_text().partition('\ncommand="${1:-}"\n')
    assert separator
    shell = tmp_path / 'runtime-functions.sh'
    shell.write_text(definitions)
    result = subprocess.run(['bash', '-c',
        'source "$1"\nstart_group() { printf "%s\\n" "$@"; }\nstart_real_control /tmp/control.log',
        'control-test', str(shell)], text=True, capture_output=True, timeout=5)
    assert result.returncode == 0, result.stderr
    arguments = result.stdout.splitlines()
    assert {'read_only:=false', 'with_moveit:=true', 'allow_trajectory_execution:=false'} <= set(arguments)
