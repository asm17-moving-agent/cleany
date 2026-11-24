"""Durable command jobs. Robot processes run in a scope outside the monitor unit."""

import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from .operator_policy import capabilities, validate_action
from .prepare_progress import prepare_progress


RUNTIME_ENV_KEYS = (
    'CLEANY_DATA_ROOT', 'CLEANY_BUILD_ROOT', 'CLEANY_INSTALL_BASE',
    'GEMINI_API_KEY', 'COLLECTION_QUERY', 'CLEANY_COLLECTION_STATE_DIR',
    'CLEANY_COLLECTION_ROS_DOMAIN_ID', 'CLEANY_COLLECTION_START_TIMEOUT_SEC',
    'CLEANY_COLLECTION_DAEMON_STOP_TIMEOUT_SEC', 'CLEANY_COLLECTION_EXECUTION_STOP_TIMEOUT_SEC',
    'CLEANY_COLLECTION_GUI', 'CLEANY_COLLECTION_MOTION_BACKEND', 'CLEANY_COLLECTION_GRASP_PREDICTOR',
    'CLEANY_COLLECTION_MOTION_RUNTIME', 'CLEANY_CUROBO_VENV',
    'CLEANY_COLLECTION_RECORD_MOTOR_TELEMETRY',
    'CLEANY_COLLECTION_USE_EXISTING_PERCEPTION', 'CLEANY_COLLECTION_SHARED_RMW_IMPLEMENTATION',
    'CLEANY_ROBOT_STATE_PROFILE', 'CLEANY_FEETECH_CONTROL_CONFIG', 'CLEANY_HARDWARE_MONITOR_CONFIG',
    'CLEANY_MANIPULATION_COLLECTION_CONFIG', 'CLEANY_MANIPULATION_PROGRESS_FILE',
    'CLEANY_SCENE_RUNTIME_PREFIX', 'CLEANY_SCENE_DEPENDENCY_PREFIX', 'CLEANY_MOVEIT_PATCH_PREFIX',
    'CLEANY_MOVEIT_GEOMETRY_CACHE_DIR', 'ROS_LOCALHOST_ONLY', 'DISPLAY', 'XAUTHORITY',
)


def runtime_settings(root: Path) -> dict[str, str]:
    path = root / 'runtime-settings.json'
    if not path.exists():
        return {}
    if path.stat().st_mode & 0o077:
        raise ValueError('runtime-settings.json 파일 권한은 0600이어야 합니다.')
    values = json.loads(path.read_text())
    if not isinstance(values, dict):
        raise ValueError('런타임 설정은 JSON 객체여야 합니다.')
    return {key: values[key] for key in RUNTIME_ENV_KEYS if isinstance(values.get(key), str)}


def atomic_json(path: Path, value: dict):
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
        json.dump(value, stream)
        temporary = Path(stream.name)
    temporary.replace(path)


def web_collection_profile(repo: Path, root: Path, configured: str | None) -> Path | None:
    """Apply the web destination and saved arm home to the stock Jetson profiles."""
    robot = (repo / 'configs/robot').resolve()
    source = Path(configured).expanduser() if configured else robot / 'collection_cad.json'
    source = (repo / source).resolve()
    if not configured and not source.exists():
        return None
    if source not in (robot / 'collection_cad.json', robot / 'jetson/collection.json'):
        return None
    profile = json.loads(source.read_text())
    if profile.get('operation_mode') != 'pick_return_release':
        return None
    from cleany_skill_executor.core.collection import load_arm_home
    for arm, positions in load_arm_home(robot / 'arm_return_home.json').items():
        profile['arms'][arm]['home_joint_rad'] = list(positions)
    profile.update(operation_mode='pick_rear_bin_release', label_destinations={'*': 'rear_bin_a'},
                   return_to_pick_reverse_enabled=False, return_to_pick_collision_check_enabled=True,
                   held_object_octomap_collision_check_enabled=True,
                   rear_bin_a=json.loads((robot / 'dev_monitor_rear_bin_a.json').read_text()))
    trajectories = robot / 'jetson/rear_bin_trajectories.json'
    if trajectories.is_file():
        profile['rear_bin_trajectory_file'] = str(trajectories)
    output = root / 'rear-bin-profile.json'
    atomic_json(output, profile)
    return output


def alive(pid):
    if type(pid) is not int or pid <= 0:
        return False
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return fields[0] not in ('Z', 'X')
    except (OSError, IndexError):
        return False


def process_start(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19]
    except (OSError, IndexError):
        return ''


def cancel_external_client():
    """Interrupt only a verified CLI pipeline process group, never a runtime node."""
    root = Path(os.environ.get('CLEANY_COLLECTION_STATE_DIR', f'/tmp/cleany-collection-runtime-{os.getuid()}'))
    try:
        pid = int((root / 'pipeline.pid').read_text())
        command = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        if not alive(pid) or os.getpgid(pid) != pid:
            return False
        if not any(Path(arg.decode(errors='replace')).name in (
            'manipulation_test_client', 'collection_pipeline', 'arm_return',
        ) for arg in command):
            return False
        os.killpg(pid, signal.SIGINT)
        return True
    except (OSError, ValueError):
        return False


def runtime_info():
    root = Path(os.environ.get('CLEANY_COLLECTION_STATE_DIR', f'/tmp/cleany-collection-runtime-{os.getuid()}'))
    try:
        lines = (root / 'pids').read_text().splitlines()
        pids = {name: int(pid) for name, pid in (line.split() for line in lines)}
    except (OSError, ValueError):
        pids = {}
    try:
        client = alive(int((root / 'pipeline.pid').read_text()))
    except (OSError, ValueError):
        client = False
    pipeline_busy = client
    operator_root = Path(os.environ.get('CLEANY_WEB_CONTROL_DIRECTORY',
        str(Path(os.environ.get('XDG_STATE_HOME', str(Path.home()/'.local/state'))) / 'cleany/dev-monitor/operator')))
    if (operator_root / 'operation.lock').exists():
        with (operator_root / 'operation.lock').open('a') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                client = True
    return dict(managed=any(alive(pid) for pid in pids.values()), client_busy=client, pipeline_busy=pipeline_busy,
                manual_runtime=(root / 'manual-rviz-profile').is_file(),
                processes={name: alive(pid) for name, pid in pids.items()})


def command_vectors(repo: Path, action: str, args: dict):
    tool = str(repo / 'tools/collection-runtime')
    if action == 'prepare':
        return [[tool, 'start'], [tool, 'action-start']]
    return {
        'run': [['make', '--no-print-directory', '-C', str(repo), 'manipulation-run']],
        'return_arm': [[tool, 'arm-return', '--arm', args.get('arm', 'both')]],
        'shutdown': [[tool, 'stop']],
    }.get(action, [])


def require_web_home(state: dict, expected_home: dict) -> None:
    if state.get('operation_mode') != 'pick_rear_bin_release':
        raise RuntimeError('실행기가 다른 놓기 경로를 사용합니다. 웹에서 로봇을 다시 준비하세요.')
    if state.get('return_home_joint_rad') != expected_home:
        raise RuntimeError('실행기에 새 기본 자세가 반영되지 않았습니다. 웹에서 로봇을 다시 준비하세요.')


def prepare_executor_state(port, *, require_execution_available: bool = True) -> dict:
    state = {**port.snapshot(), 'available': True}
    info = runtime_info()
    checked = state if require_execution_available else {**state, 'inhibited': False}
    reason = capabilities(checked, busy=info['pipeline_busy'], managed=info['managed'],
                          manual_runtime=info.get('manual_runtime', False))['run']
    if reason:
        guard = state.get('guard') or {}
        if state.get('guard_fresh') is True and guard.get('state') == 'fault':
            raise RuntimeError('로봇 준비 중 제어기 보호 정지: ' + str(guard.get('reason') or reason))
        raise RuntimeError(reason)
    return state


def approved_command(repo: Path, job: dict) -> list[str]:
    """Only server-stored, approved fields become the explicit Action goal."""
    proposal = job['proposal']
    return ['ros2', 'run', 'cleany_skill_executor', 'manipulation_test_client',
        '--namespace', '/', '--execution-id', job['id'], '--mission-id', proposal['tracking_session_id'],
        '--task-id', proposal['id'], '--snapshot-id', proposal['snapshot_id'],
        '--object-id', str(proposal['target']['object_id']), '--skill-name', proposal['skill_name'],
        '--destination-id', proposal['destination_id'],
        '--timeout', str(job['proposal_policy']['proposal_execution_timeout_sec']),
        '--ros-args', '--params-file', str(repo / 'ros2_ws/src/cleany_manipulation_bt/config/manipulation_jetson.yaml')]


def selection_message(message: str) -> str:
    if message == 'No reachable commissioned arm':
        return '설정된 팔에서 자세, 충돌, 경로 조건을 모두 통과한 파지 후보를 찾지 못했습니다.'
    return message


def command_report(path: Path) -> dict:
    """Explain this job using its CLI's final record; never authorize motion from logs."""
    try:
        with path.open('rb') as stream:
            stream.seek(0, 2)
            stream.seek(max(0, stream.tell() - 65536))
            lines = stream.read().decode(errors='replace').splitlines()
    except OSError:
        return {}
    for line in reversed(lines):
        try:
            value = json.loads(line)
        except ValueError:
            continue
        if not isinstance(value, dict) or value.get('found') is not True:
            continue
        record = value.get('record')
        if (not isinstance(record, dict) or record.get('has_result') is not True
                or record.get('status') not in ('SUCCESS', 'FAILED', 'BLOCKED', 'CANCELED')
                or not isinstance(record.get('execution_id'), str)):
            continue
        result = {key: record[key] for key in ('execution_id', 'status', 'error_code', 'message',
            'failed_stage', 'failed_substage', 'object_id', 'object_state', 'placement_state',
            'arm_recovered', 'stop_confirmed') if key in record}
        message = str(record.get('message', ''))[:1000]
        return dict(result=result, message=selection_message(message))
    for line in reversed(lines):
        if line.startswith('Collection readiness failed:'):
            return dict(message='로봇 준비 확인 실패: ' + line.split(':', 1)[1].strip()[:1000])
    return {}


def previous_action_times(path: Path) -> dict:
    """Carry prior timings across the monitor's bounded recent-job window."""
    times = {}
    for other in path.parent.glob('*.json'):
        if other == path:
            continue
        try:
            job = json.loads(other.read_text())
            if not isinstance(job, dict):
                continue
            action, state = job.get('action'), job.get('state')
            start, end = job.get('created_at'), job.get('finished_at')
            if (isinstance(action, str) and isinstance(state, str)
                    and isinstance(start, (int, float)) and isinstance(end, (int, float))
                    and end >= start and start > times.get(action, {}).get('created_at', 0)):
                times[action] = dict(created_at=start, finished_at=end, state=state)
        except (OSError, ValueError):
            continue
    return times


def execute_job(path: Path, repo: Path):
    from .operator_ros import OperatorROS
    value = json.loads(path.read_text())
    os.environ['CLEANY_WEB_CONTROL_DIRECTORY'] = str(path.parent.parent)
    action = value['action']
    retired = action == 'manual_joint'
    args = {} if retired else validate_action(action, value['args'])
    lock = (path.parent.parent / 'operation.lock').open('a')
    port = None
    record_result = True
    profile_key = 'CLEANY_MANIPULATION_COLLECTION_CONFIG'
    original_profile = os.environ.get(profile_key)
    web_profile = None
    expected_home = None
    restart_executor = False
    cancelled = path.with_suffix('.cancel')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        os.environ['CLEANY_CONTROL_RUNTIME_LOCKED'] = '1'
        value = json.loads(path.read_text())
        if retired and value.get('state') != 'PENDING':
            record_result = False
            return
        if value.get('state') != 'PENDING' or not 0 <= time.time() - value.get('created_at', 0) < 15:
            raise RuntimeError('명령 시작 대기 시간이 지났습니다. 새 상태를 확인하고 다시 요청하세요.')
        if retired:
            raise RuntimeError('수동 관절 조작 명령은 제거되었습니다.')
        if not (path.parent.parent / 'enabled').is_file():
            raise RuntimeError('웹 제어가 비활성화되었습니다.')
        if cancelled.exists():
            raise InterruptedError('실행 요청이 취소되었습니다.')
        value['previous_action_times'] = previous_action_times(path)
        value.update(state='RUNNING', owner_pid=os.getpid(), owner_start=process_start(os.getpid()), started_at=time.time())
        atomic_json(path, value)
        if action in ('prepare', 'propose', 'skip', 'approve', 'run'):
            web_profile = web_collection_profile(repo, path.parent.parent, original_profile)
            if web_profile is not None:
                os.environ[profile_key] = str(web_profile)
                expected_home = {arm: settings['home_joint_rad'] for arm, settings in
                                 json.loads(web_profile.read_text())['arms'].items()}
        if action == 'prepare' and expected_home is not None:
            port = OperatorROS()
            if runtime_info().get('processes', {}).get('manipulation_action'):
                state = prepare_executor_state(port)
                restart_executor = (state.get('operation_mode') != 'pick_rear_bin_release'
                                    or state.get('return_home_joint_rad') != expected_home)
        if action != 'prepare':
            if action in ('inspect', 'propose', 'approve', 'skip'):
                from .proposals import policy_settings
                from .proposal_ros import ProposalROS
                port = ProposalROS(path.parent.parent, value.get('proposal_policy') or policy_settings({}))
            else:
                port = OperatorROS()
            info = runtime_info()
            manual_runtime = info.get('manual_runtime', False)
            try:
                state = {**port.snapshot(), 'available': True}
            except (RuntimeError, TimeoutError):
                if action != 'shutdown' or not manual_runtime:
                    raise
                state = {}
            if expected_home is not None:
                require_web_home(state, expected_home)
            # The worker owns the shared command lock; actual executor ownership is still checked.
            reason = capabilities(state, busy=info['pipeline_busy'], managed=info['managed'],
                                  manual_runtime=manual_runtime)[action]
            if reason:
                raise RuntimeError(reason)
        if cancelled.exists():
            raise InterruptedError('실행 요청이 취소되었습니다.')
        if action == 'recover':
            result = port.recover(args['execution_id'], args['revision'], args['note'])
            value['result'] = result
            if not result['success']:
                raise RuntimeError(result['message'])
        elif action == 'inspect':
            value['result'] = port.inspect_for_monitor(value['id'], cancelled.is_file)
        elif action in ('propose', 'skip'):
            value['phase'] = 'PROPOSING'
            atomic_json(path, value)
            value['result'] = port.propose(cancelled.is_file, previous=value.get('proposal'), skip=action == 'skip')
        else:
            if action == 'approve':
                value['phase'] = 'REVALIDATING'
                atomic_json(path, value)
                port.revalidate(value['proposal'], cancelled.is_file)
                state = {**port.snapshot(), 'available': True}
                reason = capabilities(state, busy=runtime_info()['pipeline_busy'], managed=True)['approve']
                if reason:
                    raise RuntimeError(reason)
                value['phase'] = 'EXECUTING'
                atomic_json(path, value)
            commands = [approved_command(repo, value)] if action == 'approve' else command_vectors(repo, action, args)
            if restart_executor:
                commands[1][1] = 'action-restart'
            for command in commands:
                if cancelled.exists():
                    raise InterruptedError('실행 요청이 취소되었습니다.')
                process = subprocess.Popen(command, start_new_session=True)
                value['command_pid'] = process.pid
                atomic_json(path, value)
                signalled = False
                next_progress = 0.0
                while process.poll() is None:
                    if action == 'prepare' and time.monotonic() >= next_progress:
                        next_progress = time.monotonic() + 1.0
                        observed = prepare_progress(path.with_suffix('.log'), repo)
                        if observed != value.get('prepare_progress'):
                            value['prepare_progress'] = observed
                            atomic_json(path, value)
                    if cancelled.exists() and not signalled:
                        os.killpg(process.pid, signal.SIGINT)
                        signalled = True
                    time.sleep(.2)
                if action in ('run', 'approve'):
                    value.update(command_report(path.with_suffix('.log')))
                if action == 'approve' and not cancelled.exists():
                    result = value.get('result', {})
                    if result.get('stop_confirmed') and result.get('arm_recovered'):
                        try:
                            state = {**port.snapshot(), 'available': True}
                            reason = capabilities(state, busy=runtime_info()['pipeline_busy'], managed=True)['propose']
                            if reason:
                                raise RuntimeError(reason)
                            value['phase'] = 'PROPOSING_NEXT'
                            atomic_json(path, value)
                            value['next_proposal_id'] = port.propose(cancelled.is_file,
                                previous=value['proposal'], last_result=result)['id']
                        except Exception as error:
                            value['next_proposal_error'] = str(error)
                if process.returncode:
                    raise RuntimeError(value.get('message') or f'명령 종료 코드 {process.returncode}; 상세 로그와 실물 상태를 확인하세요.')
            if action == 'prepare' and expected_home is not None:
                prepared = prepare_executor_state(port, require_execution_available=False)
                require_web_home(prepared, expected_home)
                if prepared.get('inhibited'):
                    value['message'] = '로봇 준비가 완료됐습니다. 수거 실행 차단이 남아 있습니다.'
        value['state'] = 'SUCCEEDED'
    except BaseException as error:
        value.update(state='INTERRUPTED' if cancelled.exists() else 'FAILED', message=str(error))
    finally:
        if port is not None:
            try:
                port.close()
            except Exception:
                pass
        if record_result:
            if value.get('action') == 'prepare':
                value['prepare_progress'] = prepare_progress(path.with_suffix('.log'), repo)
            value.update(finished_at=time.time(), command_pid=0)
            atomic_json(path, value)
        lock.close()
        if web_profile is not None:
            if original_profile is None:
                os.environ.pop(profile_key, None)
            else:
                os.environ[profile_key] = original_profile


def main():
    import sys
    import rclpy
    path, repo = map(Path, sys.argv[1:3])
    os.environ.update(runtime_settings(path.parent.parent))
    os.environ['ROS_DOMAIN_ID'] = os.environ.get('CLEANY_COLLECTION_ROS_DOMAIN_ID', '0')
    with path.with_suffix('.log').open('a', buffering=1) as log:
        os.dup2(log.fileno(), 1)
        os.dup2(log.fileno(), 2)
        rclpy.init()
        try:
            execute_job(path, repo)
        finally:
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
