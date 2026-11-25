"""Persistent operator worker with explicit, single-job Unix socket admission."""

import asyncio
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import sys
import time
from typing import TYPE_CHECKING, Callable
from uuid import UUID

from .operator_jobs import atomic_json, execute_job, process_start, runtime_settings
from .proposals import policy_settings

if TYPE_CHECKING:
    from .proposal_ros import ProposalROS


class DispatchUnavailable(RuntimeError):
    """The worker definitely did not accept the submitted job."""


class DispatchUncertain(RuntimeError):
    """The connection failed after submission; the durable job must be queried."""


async def submit_job(path: Path, repo: Path) -> None:
    """Notify the worker without replaying or overwriting an ambiguous request."""
    try:
        reader, writer = await asyncio.wait_for(
            asyncio.open_unix_connection(str(path.parent.parent / 'worker.sock')), 2.)
    except (OSError, asyncio.TimeoutError) as error:
        raise DispatchUnavailable('작업 서비스에 연결할 수 없습니다. dev-monitor operator-start를 확인하세요.') from error
    try:
        writer.write((json.dumps({'id': path.stem}) + '\n').encode())
        await writer.drain()
        response = json.loads(await asyncio.wait_for(reader.readline(), 2.))
        if not isinstance(response, dict):
            raise ValueError('Invalid worker acknowledgement')
        if response.get('accepted') is not True:
            raise DispatchUnavailable(response.get('message', '작업 서비스가 요청을 거절했습니다.'))
    except DispatchUnavailable:
        raise
    except (OSError, ValueError, asyncio.TimeoutError) as error:
        raise DispatchUncertain('작업 접수 응답을 확인하지 못했습니다. 같은 요청의 상태를 조회하세요.') from error
    finally:
        writer.close()
        try:
            await writer.wait_closed()
        except OSError:
            pass


class OperatorPorts:
    """Reuse one ROS adapter; replace it only when its proposal policy changes."""

    def __init__(self, root: Path):
        self.root = root
        self.port = None
        self.policy = None

    def get(self, policy: dict | None = None) -> 'ProposalROS':
        from .proposal_ros import ProposalROS
        selected = policy_settings(policy) if policy is not None else self.policy or policy_settings({})
        if self.port is None or selected != self.policy:
            self.close()
            self.port = ProposalROS(self.root, selected)
            self.policy = selected
        return self.port

    def close(self) -> None:
        if self.port is not None:
            self.port.close()
            self.port = None


@contextmanager
def job_output(path: Path):
    """Keep child processes and ROS output in the existing per-request log."""
    sys.stdout.flush()
    sys.stderr.flush()
    saved = [os.dup(fd) for fd in (1, 2)]
    try:
        with path.with_suffix('.log').open('a', buffering=1) as log:
            for fd in (1, 2):
                os.dup2(log.fileno(), fd)
            yield
    finally:
        sys.stdout.flush()
        sys.stderr.flush()
        for fd, original in zip((1, 2), saved):
            os.dup2(original, fd)
            os.close(original)


class OperatorService:
    """Accept new requests only; never scan durable files for work to resume."""

    def __init__(self, root: Path, repo: Path, runner: Callable[[Path, Path], None]):
        self.root, self.repo, self.runner = root, repo, runner
        self.started_at = time.time()
        self.active_id = ''
        self.task = None
        self.stopping = False

    async def handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        try:
            request = json.loads(await asyncio.wait_for(reader.readline(), 2.))
            job_id = str(UUID(request['id']))
            path = self.root / 'jobs' / f'{job_id}.json'
            if self.stopping or not (self.root / 'enabled').is_file():
                raise ValueError('작업 서비스가 종료 중이거나 웹 제어가 꺼져 있습니다.')
            if self.active_id:
                if self.active_id != job_id:
                    raise ValueError('다른 웹 명령이 진행 중입니다.')
            else:
                job = json.loads(path.read_text())
                if (job.get('id') != job_id or job.get('state') != 'PENDING'
                        or not self.started_at <= job.get('created_at', 0) <= time.time()
                        or time.time() - job['created_at'] >= 15):
                    raise ValueError('새 작업 요청만 접수할 수 있습니다. 이전 작업은 재실행하지 않습니다.')
                self.active_id = job_id
                self.task = asyncio.create_task(self.run(path))
            response = {'accepted': True}
        except (OSError, ValueError, KeyError, TypeError, asyncio.TimeoutError) as error:
            response = {'accepted': False, 'message': str(error)}
        try:
            writer.write((json.dumps(response, ensure_ascii=False) + '\n').encode())
            await writer.drain()
        except (ConnectionError, OSError):
            pass  # A lost acknowledgement must not cancel or replay accepted work.
        finally:
            writer.close()
            try:
                await writer.wait_closed()
            except OSError:
                pass

    async def run(self, path: Path) -> None:
        try:
            await asyncio.to_thread(self.runner, path, self.repo)
        except Exception as error:
            # If execution escaped its normal finalization, do not admit another
            # job in this process or claim that an unknown command was stopped.
            self.stopping = True
            print(f'Operator job finalization failed: {error}', file=sys.stderr, flush=True)
        finally:
            self.active_id = ''

    async def serve(self, stop: asyncio.Event, ready: Callable[[], None] | None = None) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        with (self.root / 'worker.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            address = self.root / 'worker.sock'
            address.unlink(missing_ok=True)
            server = await asyncio.start_unix_server(self.handle, str(address), limit=4096)
            address.chmod(0o600)
            atomic_json(self.root / 'worker.json', dict(pid=os.getpid(),
                process_start=process_start(os.getpid()), started_at=self.started_at))
            try:
                if ready:
                    ready()
                async with server:
                    await stop.wait()
                    self.stopping = True
                    server.close()
                    await server.wait_closed()
                    if self.task:
                        await self.task
            finally:
                address.unlink(missing_ok=True)
                (self.root / 'worker.json').unlink(missing_ok=True)


def notify_ready() -> None:
    address = os.environ.get('NOTIFY_SOCKET')
    if address:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as client:
            client.connect('\0' + address[1:] if address.startswith('@') else address)
            client.sendall(b'READY=1')


def main() -> None:
    import rclpy
    root, repo = (Path(value).resolve() for value in sys.argv[1:3])
    os.environ.update(runtime_settings(root))
    os.environ['CLEANY_WEB_CONTROL_DIRECTORY'] = str(root)
    os.environ['ROS_DOMAIN_ID'] = os.environ.get('CLEANY_COLLECTION_ROS_DOMAIN_ID', '0')
    os.environ.pop('CLEANY_CONTROL_RUNTIME_LOCKED', None)
    rclpy.init()
    ports = OperatorPorts(root)
    try:
        ports.get()
        def run(path, repository):
            with job_output(path):
                execute_job(path, repository, ports=ports)

        async def serve():
            stop = asyncio.Event()
            loop = asyncio.get_running_loop()
            for sig in (signal.SIGTERM, signal.SIGINT):
                loop.add_signal_handler(sig, stop.set)
            await OperatorService(root, repo, run).serve(stop, notify_ready)

        asyncio.run(serve())
    finally:
        ports.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
