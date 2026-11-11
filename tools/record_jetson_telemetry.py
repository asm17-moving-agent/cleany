#!/usr/bin/env python3
"""Run a command while recording Jetson and container memory telemetry."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import shutil
import statistics
import subprocess
import threading
import time
from typing import Any


_RAM = re.compile(r'RAM (\d+)/(\d+)MB')
_SWAP = re.compile(r'SWAP (\d+)/(\d+)MB')
_GPU = re.compile(r'GR3D_FREQ (\d+)%')
_POWER = re.compile(r'VDD_IN (\d+)mW')


def _statistics(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {'count': 0, 'mean': None, 'p95': None, 'peak': None}
    ordered = sorted(values)
    position = (len(ordered) - 1) * 0.95
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    percentile = ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)
    return {
        'count': len(values), 'mean': statistics.fmean(values),
        'p95': percentile, 'peak': ordered[-1],
    }


def _parse_tegrastats(raw: str) -> dict[str, float]:
    values = {}
    for key, pattern in (
        ('ram_used_mb', _RAM), ('swap_used_mb', _SWAP),
        ('gpu_percent', _GPU), ('power_mw', _POWER),
    ):
        if match := pattern.search(raw):
            values[key] = float(match.group(1))
    if cpu := re.search(r'CPU \[([^]]+)\]', raw):
        cores = cpu.group(1).split(',')
        # Normalize to all advertised cores, including offline cores as idle.
        values['cpu_percent'] = sum(
            float(match.group(1)) if (match := re.search(r'(\d+)%', core)) else 0.0
            for core in cores
        ) / len(cores)
    for sensor in ('cpu', 'gpu', 'tj'):
        if match := re.search(rf'\b{sensor}@([\d.]+)C', raw):
            values[f'{sensor}_temperature_c'] = float(match.group(1))
    return values


def _read_int(path: Path) -> int | None:
    try:
        return int(path.read_text(encoding='ascii').strip())
    except (OSError, ValueError):
        return None


def _container_cgroup(name: str) -> Path | None:
    result = subprocess.run(
        ['docker', 'inspect', '--format', '{{.State.Pid}}', name],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        return None
    try:
        pid = int(result.stdout.strip())
        cgroup_lines = Path(f'/proc/{pid}/cgroup').read_text(
            encoding='ascii'
        ).splitlines()
    except (OSError, ValueError):
        return None
    for line in cgroup_lines:
        fields = line.split(':', 2)
        if len(fields) == 3 and fields[0] == '0':
            return Path('/sys/fs/cgroup') / fields[2].lstrip('/')
    return None


def _memory_sample(cgroups: dict[str, Path]) -> dict[str, Any]:
    meminfo = {}
    try:
        for line in Path('/proc/meminfo').read_text(
            encoding='ascii'
        ).splitlines():
            key, value = line.split(':', 1)
            if key in ('MemTotal', 'MemAvailable', 'SwapTotal', 'SwapFree'):
                meminfo[key] = int(value.strip().split()[0]) * 1024
    except (OSError, ValueError):
        pass
    containers = {}
    for name, root in cgroups.items():
        containers[name] = {
            'memory_current_bytes': _read_int(root / 'memory.current'),
            'memory_peak_bytes': _read_int(root / 'memory.peak'),
            'memory_swap_current_bytes': _read_int(
                root / 'memory.swap.current'
            ),
        }
    return {'meminfo': meminfo, 'containers': containers}


def _memory_events(root: Path) -> dict[str, int]:
    values = {}
    try:
        for line in (root / 'memory.events').read_text(
            encoding='ascii'
        ).splitlines():
            key, value = line.split()
            values[key] = int(value)
    except (OSError, ValueError):
        pass
    return values


def _metadata(command: list[str], containers: list[str]) -> dict[str, Any]:
    def output(*args: str) -> str:
        result = subprocess.run(
            list(args), capture_output=True, text=True, check=False
        )
        return (result.stdout or result.stderr).strip()

    release = ''
    try:
        release = Path('/etc/nv_tegra_release').read_text(
            encoding='utf-8'
        ).strip()
    except OSError:
        pass
    return {
        'schema': 'cleany.jetson_telemetry/v1',
        'started_at': datetime.now(timezone.utc).isoformat(),
        'command': command,
        'containers': containers,
        'uname': output('uname', '-a'),
        'jetson_release': release,
        'nvpmodel': output('/usr/sbin/nvpmodel', '-q'),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--interval-ms', type=int, default=200)
    parser.add_argument('--container', action='append', default=[])
    parser.add_argument('command', nargs=argparse.REMAINDER)
    arguments = parser.parse_args()
    command = arguments.command
    if command and command[0] == '--':
        command = command[1:]
    if not command:
        parser.error('a command is required after --')
    if arguments.interval_ms < 50:
        parser.error('--interval-ms must be at least 50')
    arguments.output_root.mkdir(parents=True, exist_ok=True)
    events_path = arguments.output_root / 'telemetry.jsonl'
    summary_path = arguments.output_root / 'telemetry_summary.json'
    if any(path.exists() for path in (
        events_path, summary_path, arguments.output_root / 'run_metadata.json',
    )):
        parser.error('output directory already contains telemetry; choose a new path')
    metadata = _metadata(command, arguments.container)
    gpu_frequency_path = next(Path('/sys/class/devfreq').glob('*gpu*/cur_freq'), None)
    metadata['gpu_clock_path'] = str(gpu_frequency_path) if gpu_frequency_path else None
    (arguments.output_root / 'run_metadata.json').write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )

    cgroups = {
        name: root
        for name in arguments.container
        if (root := _container_cgroup(name)) is not None
    }
    lock = threading.Lock()
    started = time.monotonic()
    tegra_values: list[dict[str, float]] = []
    memory_values: list[dict[str, Any]] = []
    with events_path.open('w', encoding='utf-8', buffering=1) as stream:
        def write(event: dict[str, Any]) -> None:
            event['elapsed_seconds'] = time.monotonic() - started
            event['timestamp'] = datetime.now(timezone.utc).isoformat()
            with lock:
                stream.write(json.dumps(event, separators=(',', ':')) + '\n')

        tegrastats = shutil.which('tegrastats')
        if tegrastats is None:
            raise RuntimeError('tegrastats is unavailable')
        tegra = subprocess.Popen(
            [tegrastats, '--interval', str(arguments.interval_ms)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

        def read_tegrastats() -> None:
            assert tegra.stdout is not None
            for line in tegra.stdout:
                raw = line.strip()
                values = _parse_tegrastats(raw)
                tegra_values.append(values)
                write({'kind': 'tegrastats', 'raw': raw, **values})

        reader = threading.Thread(target=read_tegrastats, daemon=True)
        reader.start()
        process = subprocess.Popen(command)
        try:
            while process.poll() is None:
                sample = _memory_sample(cgroups)
                sample['gpu_clock_hz'] = (
                    _read_int(gpu_frequency_path) if gpu_frequency_path else None
                )
                memory_values.append(sample)
                write({'kind': 'memory', **sample})
                time.sleep(arguments.interval_ms / 1000.0)
            return_code = int(process.returncode)
        finally:
            tegra.terminate()
            try:
                tegra.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                tegra.kill()
            reader.join(timeout=3.0)

    container_peaks: dict[str, int] = {}
    system_used_peak = 0
    system_used_min: int | None = None
    system_swap_peak = 0
    for sample in memory_values:
        meminfo = sample['meminfo']
        if 'MemTotal' in meminfo and 'MemAvailable' in meminfo:
            used = meminfo['MemTotal'] - meminfo['MemAvailable']
            system_used_peak = max(system_used_peak, used)
            system_used_min = (
                used if system_used_min is None else min(system_used_min, used)
            )
        if 'SwapTotal' in meminfo and 'SwapFree' in meminfo:
            system_swap_peak = max(
                system_swap_peak,
                meminfo['SwapTotal'] - meminfo['SwapFree'],
            )
        for name, values in sample['containers'].items():
            current = values.get('memory_current_bytes')
            if current is not None:
                container_peaks[name] = max(
                    container_peaks.get(name, 0), current
                )
    summary = {
        'schema': 'cleany.jetson_telemetry_summary/v1',
        'duration_seconds': time.monotonic() - started,
        'command_return_code': return_code,
        'sample_interval_ms': arguments.interval_ms,
        'sample_count': len(memory_values),
        'system_used_min_bytes': system_used_min or 0,
        'system_used_peak_bytes': system_used_peak,
        'system_used_peak_delta_bytes': (
            system_used_peak - (system_used_min or 0)
        ),
        'system_swap_used_peak_bytes': system_swap_peak,
        'container_memory_current_peak_bytes': container_peaks,
        'container_memory_events': {
            name: _memory_events(root) for name, root in cgroups.items()
        },
        'tegrastats_ram_used_peak_mb': max(
            (item.get('ram_used_mb', 0) for item in tegra_values),
            default=0,
        ),
        'tegrastats_swap_used_peak_mb': max(
            (item.get('swap_used_mb', 0) for item in tegra_values),
            default=0,
        ),
        'tegrastats_gpu_peak_percent': max(
            (item.get('gpu_percent', 0) for item in tegra_values),
            default=0,
        ),
        'tegrastats_power_peak_mw': max(
            (item.get('power_mw', 0) for item in tegra_values),
            default=0,
        ),
        'tegrastats_statistics': {
            key: _statistics([item[key] for item in tegra_values if key in item])
            for key in sorted({key for item in tegra_values for key in item})
        },
        'system_used_bytes_statistics': _statistics([
            item['meminfo']['MemTotal'] - item['meminfo']['MemAvailable']
            for item in memory_values
            if {'MemTotal', 'MemAvailable'} <= item['meminfo'].keys()
        ]),
        'gpu_clock_mhz_statistics': _statistics([
            item['gpu_clock_hz'] / 1_000_000 for item in memory_values
            if item['gpu_clock_hz'] is not None
        ]),
        'container_memory_current_bytes_statistics': {
            name: _statistics([
                item['containers'][name]['memory_current_bytes']
                for item in memory_values
                if item['containers'][name]['memory_current_bytes'] is not None
            ])
            for name in cgroups
        },
    }
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    raise SystemExit(return_code)


if __name__ == '__main__':
    main()
