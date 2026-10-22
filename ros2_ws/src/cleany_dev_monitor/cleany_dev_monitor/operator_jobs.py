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


RUNTIME_ENV_KEYS = (
    'CLEANY_DATA_ROOT', 'CLEANY_BUILD_ROOT', 'CLEANY_INSTALL_BASE',
    'GEMINI_API_KEY', 'COLLECTION_QUERY', 'CLEANY_COLLECTION_STATE_DIR',
    'CLEANY_COLLECTION_ROS_DOMAIN_ID', 'CLEANY_COLLECTION_START_TIMEOUT_SEC',
    'CLEANY_COLLECTION_DAEMON_STOP_TIMEOUT_SEC', 'CLEANY_COLLECTION_EXECUTION_STOP_TIMEOUT_SEC',
    'CLEANY_COLLECTION_GUI', 'CLEANY_COLLECTION_MOTION_BACKEND', 'CLEANY_COLLECTION_GRASP_PREDICTOR',
    'CLEANY_FEETECH_CONTROL_CONFIG', 'CLEANY_HARDWARE_MONITOR_CONFIG',
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
                processes={name: alive(pid) for name, pid in pids.items()})


def command_vectors(repo: Path, action: str, args: dict):
    tool = str(repo / 'tools/collection-runtime')
    return {
        'prepare': [[tool, 'start'], [tool, 'action-start']],
        'run': [['make', '--no-print-directory', '-C', str(repo), 'manipulation-run']],
        'return_arm': [[tool, 'arm-return', '--arm', args.get('arm', 'both')]],
        'shutdown': [[tool, 'stop']],
    }.get(action, [])


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
        if message == 'No reachable commissioned arm':
            message = '선택한 물체를 잡을 팔 자세(IK)를 양팔 모두에서 찾지 못했습니다. 물체 위치와 접근 방향을 확인하세요.'
        return dict(result=result, message=message)
    for line in reversed(lines):
        if line.startswith('Collection readiness failed:'):
            return dict(message='로봇 준비 확인 실패: ' + line.split(':', 1)[1].strip()[:1000])
    return {}


def execute_job(path: Path, repo: Path):
    from .operator_ros import OperatorROS
    value = json.loads(path.read_text())
    os.environ['CLEANY_WEB_CONTROL_DIRECTORY'] = str(path.parent.parent)
    action, args = value['action'], validate_action(value['action'], value['args'])
    lock = (path.parent.parent / 'operation.lock').open('a')
    port = None
    cancelled = path.with_suffix('.cancel')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        os.environ['CLEANY_CONTROL_RUNTIME_LOCKED'] = '1'
        value = json.loads(path.read_text())
        if value.get('state') != 'PENDING' or not 0 <= time.time() - value.get('created_at', 0) < 15:
            raise RuntimeError('명령 시작 대기 시간이 지났습니다. 새 상태를 확인하고 다시 요청하세요.')
        if not (path.parent.parent / 'enabled').is_file():
            raise RuntimeError('웹 제어가 비활성화되었습니다.')
        if cancelled.exists():
            raise InterruptedError('실행 요청이 취소되었습니다.')
        value.update(state='RUNNING', owner_pid=os.getpid(), owner_start=process_start(os.getpid()), started_at=time.time())
        atomic_json(path, value)
        if action != 'prepare':
            if action in ('propose', 'approve', 'skip'):
                from .proposal_ros import ProposalROS
                port = ProposalROS(path.parent.parent, value['proposal_policy'])
            else:
                port = OperatorROS()
            state = {**port.snapshot(), 'available': True}
            info = runtime_info()
            # The worker owns the shared command lock; actual executor ownership is still checked.
            reason = capabilities(state, busy=info['pipeline_busy'], managed=info['managed'])[action]
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
            value['result'] = port.inspect(cancelled.is_file)
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
            for command in commands:
                if cancelled.exists():
                    raise InterruptedError('실행 요청이 취소되었습니다.')
                process = subprocess.Popen(command, start_new_session=True)
                value['command_pid'] = process.pid
                atomic_json(path, value)
                signalled = False
                while process.poll() is None:
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
        value['state'] = 'SUCCEEDED'
    except BaseException as error:
        value.update(state='INTERRUPTED' if cancelled.exists() else 'FAILED', message=str(error))
    finally:
        if port is not None:
            try:
                port.close()
            except Exception:
                pass
        value.update(finished_at=time.time(), command_pid=0)
        atomic_json(path, value)
        lock.close()


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
