#!/usr/bin/env python3
"""Collect a reproducible Jetson runtime preflight report.

This script intentionally uses only the Python standard library so it can run
before ROS 2 or AI packages are installed.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import platform
import re
import shutil
import socket
import subprocess
import sys
from typing import Any, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]


def _read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding='utf-8').strip().rstrip('\x00')
    except (OSError, UnicodeError):
        return None


def resolve_command(
    command: str,
    fallback_paths: Sequence[Path] = (),
) -> str | None:
    """Resolve a command from PATH or known installation locations."""
    discovered = shutil.which(command)
    if discovered is not None:
        return discovered

    for candidate in fallback_paths:
        if candidate.is_file() and candidate.stat().st_mode & 0o111:
            return str(candidate)
    return None


def _run(command: Sequence[str], *, cwd: Path | None = None) -> dict[str, Any]:
    executable = shutil.which(command[0])
    if executable is None:
        return {
            'available': False,
            'returncode': None,
            'stdout': '',
            'stderr': '',
        }

    try:
        result = subprocess.run(
            [executable, *command[1:]],
            cwd=cwd,
            capture_output=True,
            check=False,
            text=True,
            timeout=15,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return {
            'available': True,
            'returncode': None,
            'stdout': '',
            'stderr': str(error),
        }

    return {
        'available': True,
        'returncode': result.returncode,
        'stdout': result.stdout.strip(),
        'stderr': result.stderr.strip(),
    }


def parse_os_release(text: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in text.splitlines():
        if '=' not in line or line.lstrip().startswith('#'):
            continue
        key, value = line.split('=', 1)
        values[key] = value.strip().strip('"')
    return values


def parse_l4t_release(text: str) -> dict[str, str | None]:
    release_match = re.search(r'# R(\d+) \(release\)', text)
    revision_match = re.search(r'REVISION:\s*([\d.]+)', text)
    board_match = re.search(r'GCID:\s*(\d+)', text)
    release = release_match.group(1) if release_match else None
    revision = revision_match.group(1) if revision_match else None
    return {
        'release': release,
        'revision': revision,
        'version': f'{release}.{revision}' if release and revision else None,
        'gcid': board_match.group(1) if board_match else None,
        'raw': text.strip(),
    }


def parse_nvpmodel(text: str) -> dict[str, str | int | None]:
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    mode_id: int | None = None
    mode_name: str | None = None

    for index, line in enumerate(lines):
        id_match = re.fullmatch(r'(\d+)', line)
        if id_match:
            mode_id = int(id_match.group(1))
            if index > 0:
                mode_name = lines[index - 1]
            break

    if mode_name is None and lines:
        mode_name = lines[0]

    return {'name': mode_name, 'id': mode_id, 'raw': text.strip()}


def parse_dpkg_packages(text: str) -> list[dict[str, str]]:
    packages: list[dict[str, str]] = []
    for line in text.splitlines():
        parts = line.split(None, 4)
        if len(parts) < 4 or parts[0].strip() != 'ii':
            continue
        packages.append(
            {
                'name': parts[1],
                'version': parts[2],
                'architecture': parts[3],
            }
        )
    return packages


def _dpkg_packages(pattern: str) -> list[dict[str, str]]:
    result = _run(['dpkg-query', '-l', pattern])
    return parse_dpkg_packages(result['stdout']) if result['available'] else []


def _cuda_report() -> dict[str, Any]:
    result = _run(['nvcc', '--version'])
    version_match = re.search(r'release\s+([\d.]+)', result['stdout'])
    return {
        'available': result['available'] and result['returncode'] == 0,
        'version': version_match.group(1) if version_match else None,
        'raw': result['stdout'] or result['stderr'],
    }


def _torch_report() -> dict[str, Any]:
    if importlib.util.find_spec('torch') is None:
        return {'installed': False, 'cuda_smoke': False}

    probe = (
        'import json, torch; '
        'available=torch.cuda.is_available(); '
        'result={"version":torch.__version__, '
        '"cuda_available":available, '
        '"torch_cuda_version":torch.version.cuda, '
        '"device_name":torch.cuda.get_device_name(0) if available else None, '
        '"cuda_smoke":bool((torch.ones(2, device="cuda") + 1).sum().item() == 4) '
        'if available else False}; '
        'print(json.dumps(result))'
    )
    result = _run([sys.executable, '-c', probe])
    if result['returncode'] != 0:
        return {
            'installed': True,
            'cuda_smoke': False,
            'error': result['stderr'] or result['stdout'],
        }

    try:
        report = json.loads(result['stdout'])
    except json.JSONDecodeError as error:
        return {'installed': True, 'cuda_smoke': False, 'error': str(error)}
    return {'installed': True, **report}


def _thermal_report() -> list[dict[str, Any]]:
    zones: list[dict[str, Any]] = []
    for path in sorted(Path('/sys/class/thermal').glob('thermal_zone*')):
        raw_temperature = _read_text(path / 'temp')
        zone_type = _read_text(path / 'type')
        if raw_temperature is None:
            continue
        try:
            celsius = float(raw_temperature) / 1000.0
        except ValueError:
            continue
        zones.append({'type': zone_type, 'celsius': celsius})
    return zones


def _memory_report() -> dict[str, int | None]:
    values: dict[str, int] = {}
    memory_text = _read_text(Path('/proc/meminfo')) or ''
    for line in memory_text.splitlines():
        match = re.fullmatch(r'(\w+):\s+(\d+)\s+kB', line)
        if match:
            values[match.group(1)] = int(match.group(2)) * 1024
    return {
        'total_bytes': values.get('MemTotal'),
        'available_bytes': values.get('MemAvailable'),
    }


def _git_report() -> dict[str, Any]:
    branch = _run(['git', 'branch', '--show-current'], cwd=REPO_ROOT)
    commit = _run(['git', 'rev-parse', 'HEAD'], cwd=REPO_ROOT)
    status = _run(['git', 'status', '--porcelain'], cwd=REPO_ROOT)
    return {
        'root': str(REPO_ROOT),
        'branch': branch['stdout'] or None,
        'commit': commit['stdout'] or None,
        'dirty': bool(status['stdout']),
    }


def base_checks(report: dict[str, Any]) -> dict[str, bool]:
    os_release = report['system']['os_release']
    l4t = report['jetson']['l4t']
    return {
        'architecture_is_aarch64': report['system']['architecture'] == 'aarch64',
        'ubuntu_is_22_04': os_release.get('ID') == 'ubuntu'
        and os_release.get('VERSION_ID') == '22.04',
        'python_is_3_10': report['python']['major_minor'] == '3.10',
        'l4t_detected': bool(l4t.get('version')),
        'jetpack_package_installed': bool(report['jetson']['jetpack_packages']),
        'cuda_detected': bool(report['cuda']['version']),
        'cudnn_detected': bool(report['cudnn_packages']),
        'tensorrt_detected': bool(report['tensorrt_packages']),
        'nvpmodel_detected': report['jetson']['nvpmodel']['id'] is not None,
    }


def collect_report() -> dict[str, Any]:
    os_release = parse_os_release(_read_text(Path('/etc/os-release')) or '')
    l4t_text = _read_text(Path('/etc/nv_tegra_release')) or ''
    nvpmodel_result = _run(['nvpmodel', '-q'])
    disk = shutil.disk_usage(REPO_ROOT)
    ros2_fallbacks = sorted(Path('/opt/ros').glob('*/bin/ros2'))
    report: dict[str, Any] = {
        'schema_version': 1,
        'collected_at': datetime.now(timezone.utc).isoformat(),
        'system': {
            'hostname': socket.gethostname(),
            'architecture': platform.machine(),
            'kernel': platform.release(),
            'device_model': _read_text(Path('/proc/device-tree/model')),
            'os_release': os_release,
        },
        'python': {
            'version': platform.python_version(),
            'major_minor': f'{sys.version_info.major}.{sys.version_info.minor}',
            'executable': sys.executable,
        },
        'jetson': {
            'l4t': parse_l4t_release(l4t_text),
            'jetpack_packages': _dpkg_packages('nvidia-jetpack'),
            'nvpmodel': {
                **parse_nvpmodel(nvpmodel_result['stdout']),
                'available': nvpmodel_result['available'],
                'returncode': nvpmodel_result['returncode'],
                'error': nvpmodel_result['stderr'] or None,
            },
        },
        'cuda': _cuda_report(),
        'cudnn_packages': _dpkg_packages('libcudnn*'),
        'tensorrt_packages': _dpkg_packages('libnvinfer*'),
        'torch': _torch_report(),
        'thermal_zones': _thermal_report(),
        'memory': _memory_report(),
        'disk': {
            'path': str(REPO_ROOT),
            'total_bytes': disk.total,
            'free_bytes': disk.free,
        },
        'commands': {
            'tegrastats': resolve_command('tegrastats'),
            'jetson_clocks': resolve_command('jetson_clocks'),
            'ros2': resolve_command('ros2', ros2_fallbacks),
            'rs-enumerate-devices': resolve_command('rs-enumerate-devices'),
        },
        'repository': _git_report(),
    }
    report['checks'] = base_checks(report)
    report['base_ready'] = all(report['checks'].values())
    return report


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--check',
        action='store_true',
        help='exit with status 2 when a base runtime check fails',
    )
    parser.add_argument(
        '--output',
        type=Path,
        help='write the JSON report to this path instead of stdout',
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    options = _parse_args(argv)
    report = collect_report()
    rendered = json.dumps(report, indent=2, ensure_ascii=False) + '\n'

    if options.output is None:
        print(rendered, end='')
    else:
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(rendered, encoding='utf-8')

    return 2 if options.check and not report['base_ready'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
