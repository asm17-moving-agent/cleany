"""Docker boundary for the selector; never source a host ROS install in Docker."""
from __future__ import annotations

from cleany_paths import install_base

import argparse
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
from tempfile import TemporaryDirectory
from typing import Any
from uuid import uuid4

import yaml


CONTAINER_ROOT = '/workspace/cleany'
WORKSPACE = '/opt/cleany-motion/workspace'
PACKAGES = ('cleany_interfaces', 'cleany_description', 'cleany_moveit_config', 'cleany_skill_executor')
DDS_ENVIRONMENT = (
    'ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_STATIC_PEERS',
    'RMW_IMPLEMENTATION', 'CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE',
    'FASTDDS_DEFAULT_PROFILES_FILE', 'RMW_FASTRTPS_USE_QOS_FROM_XML',
)


def default_host_rmw() -> str:
    try:
        from rclpy.utilities import get_rmw_implementation_identifier
    except ImportError:
        # Standalone exec outside a sourced ROS environment: Humble's default.
        return 'rmw_fastrtps_cpp'
    return get_rmw_implementation_identifier()


@dataclass(frozen=True)
class Runtime:
    root: Path
    image: str = 'cleany-motion:jp6.2'

    @property
    def identity(self) -> str:
        return hashlib.sha256(str(self.root).encode()).hexdigest()[:12]

    @property
    def workspace_volume(self) -> str:
        return f'cleany-motion-workspace-{self.identity}'

    @property
    def cache_volume(self) -> str:
        return f'cleany-motion-cache-{self.identity}'

    def docker(self, *args: str, capture: bool = False,
               timeout: float | None = 30) -> subprocess.CompletedProcess[str]:
        return subprocess.run(['docker', *args], check=True, text=True,
                              capture_output=capture, timeout=timeout)

    def image_id(self) -> str:
        return self.docker('image', 'inspect', '--format', '{{.Id}}', self.image,
                           capture=True).stdout.strip()

    def fingerprint(self) -> str:
        """Invalidate preparation when installed code, model or interface inputs change."""
        digest = hashlib.sha256()
        extensions = {'.py', '.yaml', '.xml', '.xacro', '.srdf', '.urdf', '.msg', '.srv', '.action', '.cfg'}
        for package in PACKAGES:
            directory = self.root / 'ros2_ws/src' / package
            for path in sorted(directory.rglob('*')):
                relative = path.relative_to(self.root)
                if any(part in ('__pycache__', 'test', '.pytest_cache') for part in relative.parts):
                    continue
                model_asset = package == 'cleany_description' and any(
                    part in ('meshes', 'mjcf', 'urdf') for part in relative.parts)
                if path.is_file() and (path.suffix in extensions or path.name == 'CMakeLists.txt' or model_asset):
                    digest.update(str(relative).encode())
                    digest.update(path.read_bytes())
        digest.update((self.root / 'containers/motion/runtime-entrypoint').read_bytes())
        return digest.hexdigest()

    def receipt(self) -> dict[str, str]:
        return {'image_id': self.image_id(), 'source_sha256': self.fingerprint()}

    def base(self, *, gpu: bool = True) -> list[str]:
        options = ['--network', 'host', '--ipc', 'host', '--pid', 'host', '--init',
                   '--label', 'cleany.runtime=motion', '--label', f'cleany.repository={self.identity}',
                   '--volume', f'{self.root}:{CONTAINER_ROOT}:ro',
                   '--volume', f'{self.workspace_volume}:{WORKSPACE}',
                   '--volume', f'{self.cache_volume}:/opt/cleany-motion/cache',
                   '--workdir', CONTAINER_ROOT]
        if gpu:
            options += ['--runtime', 'nvidia']
        return options

    def check(self) -> None:
        try:
            expected = self.receipt()
            self.docker('volume', 'inspect', self.workspace_volume, capture=True)
            self.docker('volume', 'inspect', self.cache_volume, capture=True)
            script = ('import json,pathlib; p=pathlib.Path("' + WORKSPACE + '"); '
                      'assert (p/"install/local_setup.bash").is_file(); '
                      'import os; assert os.access("/opt/cleany-motion/cache", os.W_OK); '
                      'print((p/"ready.json").read_text())')
            result = self.docker('run', '--rm', '--network', 'none',
                '--user', f'{os.getuid()}:{os.getgid()}',
                '--volume', f'{self.workspace_volume}:{WORKSPACE}:ro',
                '--volume', f'{self.cache_volume}:/opt/cleany-motion/cache',
                '--entrypoint', 'python3', self.image, '-c', script, capture=True)
            if json.loads(result.stdout) != expected:
                raise ValueError('Prepared workspace is stale for this image or source')
        except (OSError, subprocess.SubprocessError, ValueError) as error:
            raise RuntimeError('Motion container is not prepared. Run: '
                               'make motion-container-image motion-container-prepare\n' + str(error)) from error

    def translate(self, value: Any) -> Any:
        if isinstance(value, dict):
            return {key: self.translate(item) for key, item in value.items()}
        if isinstance(value, list):
            return [self.translate(item) for item in value]
        if isinstance(value, str):
            for source, destination in ((str(install_base(self.root)), WORKSPACE + '/install'),
                                        (str(self.root / 'ros2_ws/install'), WORKSPACE + '/install'),
                                        (str(self.root), CONTAINER_ROOT)):
                if value == source or value.startswith(source + '/'):
                    return destination + value[len(source):]
        return value

    def parameters(self, args: list[str], directory: Path) -> list[str]:
        converted = []
        index = 0
        while index < len(args):
            value = args[index]
            if value == '--params-file' or value.startswith('--params-file='):
                inline = value != '--params-file'
                if not inline and index + 1 == len(args):
                    raise ValueError('Missing ROS parameter file')
                source = Path(value.split('=', 1)[1] if inline else args[index + 1])
                destination = directory / f'parameters-{index}.yaml'
                destination.write_text(yaml.safe_dump(self.translate(yaml.safe_load(source.read_text()))))
                converted += ['--params-file', '/run/cleany-motion/' + destination.name]
                index += 1 if inline else 2
            else:
                # Also preserve/translate command-line ROS parameter overrides.
                if ':=' in value:
                    key, item = value.split(':=', 1)
                    value = key + ':=' + self.translate(item)
                converted.append(value)
                index += 1
        return converted

    def environment(self, environment: dict[str, str]) -> list[str]:
        blas_threads = int(environment.get('CLEANY_MOTION_BLAS_THREADS', '1'))
        if blas_threads < 1:
            raise ValueError('CLEANY_MOTION_BLAS_THREADS must be a positive integer')
        # Limit BLAS before importing NumPy/SciPy without overriding ROS or OpenMP.
        options = ['--env', f'OPENBLAS_NUM_THREADS={blas_threads}']
        mounts = set()
        for key in DDS_ENVIRONMENT:
            value = environment.get(key, '0' if key == 'ROS_DOMAIN_ID' else '')
            if key == 'RMW_IMPLEMENTATION' and not value:
                value = default_host_rmw()
            if not value:
                continue
            # Keep host DDS paths valid without mounting an entire system directory.
            if key.endswith('PROFILES_FILE') or key == 'CYCLONEDDS_URI':
                path = value.removeprefix('file://')
                if not path.lstrip().startswith('<'):
                    source = Path(path).expanduser().resolve()
                    if not source.is_file():
                        raise ValueError(f'{key} file does not exist: {source}')
                    mounts.add(source)
                    value = ('file://' if value.startswith('file://') else '') + str(source)
            options += ['--env', f'{key}={value}']
        for directory in sorted(mounts):
            options += ['--volume', f'{directory}:{directory}:ro']
        return options

    def owned(self, owner: str) -> list[str]:
        if not owner:
            raise ValueError('An explicit --owner is required')
        return self.docker('ps', '-a', '--filter', 'label=cleany.runtime=motion',
            '--filter', f'label=cleany.repository={self.identity}',
            '--filter', f'label=cleany.owner={owner}', '--format', '{{.Names}}', capture=True).stdout.split()

    def execution_user(self) -> list[str]:
        # Fast DDS shared memory requires the same user on both sides.
        return ['--user', f'{os.getuid()}:{os.getgid()}', '--env', 'HOME=/tmp',
                '--env', 'TINI_SUBREAPER=1']

    def cleanup(self, name: str) -> None:
        # Finish inside ROS launch's default 5s TERM + 5s KILL escalation window.
        # A timed-out stop must still attempt removal of this container.
        for args in (('stop', '--time', '3', name), ('rm', '--force', name)):
            try:
                subprocess.run(['docker', *args], stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL, timeout=4, check=False)
            except (OSError, subprocess.TimeoutExpired) as error:
                print(f'Motion cleanup failed for {name}: {error}', file=sys.stderr)


def runtime() -> Runtime:
    return Runtime(Path(__file__).resolve().parents[1], os.environ.get('CLEANY_MOTION_IMAGE', 'cleany-motion:jp6.2'))


def launch_selector(args: list[str] | None = None) -> int:
    args = sys.argv[1:] if args is None else args
    if not args or Path(args[0]).name != 'physical_grasp_selection_server':
        raise ValueError('This launch prefix accepts only physical_grasp_selection_server')
    settings = runtime()
    name = 'cleany-motion-selector-' + uuid4().hex[:12]
    owner = os.environ.get('CLEANY_MOTION_OWNER', name)
    process = None
    create_attempted = False
    interrupted = 0

    def interrupt(signum, frame):
        nonlocal interrupted
        interrupted = signum

    previous = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        settings.check()
        with TemporaryDirectory(prefix='cleany-motion-') as directory:
            directory = Path(directory)
            converted = settings.parameters(args[1:], directory)
            options = settings.base() + settings.execution_user() + settings.environment(dict(os.environ))
            options += ['--label', f'cleany.owner={owner}', '--name', name,
                        '--volume', f'{directory}:/run/cleany-motion:ro']
            if interrupted:
                return 128 + interrupted
            create_attempted = True
            settings.docker('create', *options, settings.image, 'python3', '-m',
                'cleany_skill_executor.physical_grasp_selection_node', *converted, capture=True)
            print(f'MOTION_CONTAINER name={name} owner={owner} image={settings.image} '
                  f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")} '
                  f'RMW_IMPLEMENTATION={os.environ.get("RMW_IMPLEMENTATION") or default_host_rmw()}', flush=True)
            if interrupted:
                return 128 + interrupted
            process = subprocess.Popen(['docker', 'start', '--attach', name])
            while not interrupted:
                try:
                    return process.wait(timeout=0.2)
                except subprocess.TimeoutExpired:
                    pass
            return 128 + interrupted
    except (OSError, subprocess.SubprocessError, RuntimeError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 1
    finally:
        # Ignore repeated signals while removing only this invocation's container.
        for sig in previous:
            signal.signal(sig, signal.SIG_IGN)
        try:
            if create_attempted:
                settings.cleanup(name)
            if process is not None and process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)


def cli() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('image', 'prepare', 'check', 'exec', 'status', 'stop'))
    parser.add_argument('--owner', default=os.environ.get('CLEANY_MOTION_OWNER', ''))
    options, remainder = parser.parse_known_args()
    settings = runtime()
    try:
        if options.command == 'image':
            # Send only image inputs, not host installs, private runtime files or meshes.
            with TemporaryDirectory(prefix='cleany-motion-build-') as directory:
                context = Path(directory)
                shutil.copytree(settings.root / 'containers/motion', context / 'containers/motion')
                settings.docker('build', '--network', 'host', '-f', str(context / 'containers/motion/Dockerfile'),
                                '-t', settings.image, str(context), timeout=None)
        elif options.command == 'prepare':
            receipt = json.dumps(settings.receipt())
            settings.docker('run', '--rm', *settings.base(), settings.image, 'prepare', receipt, timeout=None)
            settings.docker('run', '--rm', '--network', 'none',
                '--volume', f'{settings.cache_volume}:/opt/cleany-motion/cache',
                '--entrypoint', 'chown', settings.image, '-R', f'{os.getuid()}:{os.getgid()}',
                '/opt/cleany-motion/cache', timeout=None)
            settings.check()
        elif options.command == 'check':
            settings.check()
            print(f'Motion container ready: {settings.image} ({settings.workspace_volume})')
        elif options.command == 'exec':
            settings.check()
            if remainder and remainder[0] == '--':
                remainder = remainder[1:]
            if not remainder:
                raise ValueError('exec requires a command after --')
            settings.docker('run', '--rm', *settings.base(), *settings.execution_user(), *settings.environment(dict(os.environ)),
                            settings.image, *remainder, timeout=None)
        else:
            for name in settings.owned(options.owner):
                if options.command == 'stop':
                    settings.cleanup(name)
                else:
                    print(settings.docker('inspect', '--format', '{{.Name}} {{.State.Status}}',
                                          name, capture=True).stdout.strip())
        return 0
    except (OSError, subprocess.SubprocessError, RuntimeError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 1
