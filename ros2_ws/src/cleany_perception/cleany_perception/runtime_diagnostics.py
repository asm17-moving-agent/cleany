from __future__ import annotations

from collections.abc import Callable
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import resource
import time
from typing import Any

import numpy as np
from PIL import Image as PilImage


_SNAPSHOT_ID_PATTERN = re.compile(r'[A-Za-z0-9][A-Za-z0-9._-]*')
MemoryValue = int | float | None
MemoryReader = Callable[[bool, bool], dict[str, MemoryValue]]


def _current_rss_bytes() -> int | None:
    try:
        fields = Path('/proc/self/statm').read_text(encoding='utf-8').split()
        return int(fields[1]) * os.sysconf('SC_PAGE_SIZE')
    except (OSError, ValueError, IndexError):
        return None


def _process_peak_rss_bytes() -> int:
    # Linux reports ru_maxrss in KiB.
    return int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) * 1024


def _system_memory_bytes() -> tuple[int | None, int | None]:
    try:
        values = {}
        for line in Path('/proc/meminfo').read_text(
            encoding='utf-8'
        ).splitlines():
            name, raw_value = line.split(':', maxsplit=1)
            if name in ('MemTotal', 'MemAvailable'):
                values[name] = int(raw_value.split()[0]) * 1024
        return values.get('MemTotal'), values.get('MemAvailable')
    except (OSError, ValueError, IndexError):
        return None, None


def read_memory(
    include_cuda: bool,
    reset_cuda_peak: bool = False,
) -> dict[str, MemoryValue]:
    system_total, system_available = _system_memory_bytes()
    values: dict[str, MemoryValue] = {
        'rss_bytes': _current_rss_bytes(),
        'process_peak_rss_bytes': _process_peak_rss_bytes(),
        'system_total_bytes': system_total,
        'system_available_bytes': system_available,
        'cuda_allocated_bytes': None,
        'cuda_reserved_bytes': None,
        'cuda_peak_allocated_bytes': None,
        'cuda_peak_reserved_bytes': None,
        'cuda_total_bytes': None,
    }
    if not include_cuda:
        return values
    try:
        import torch

        if not torch.cuda.is_available():
            return values
        torch.cuda.synchronize()
        if reset_cuda_peak:
            torch.cuda.reset_peak_memory_stats()
        values.update(
            {
                'cuda_allocated_bytes': int(torch.cuda.memory_allocated()),
                'cuda_reserved_bytes': int(torch.cuda.memory_reserved()),
                'cuda_peak_allocated_bytes': int(
                    torch.cuda.max_memory_allocated()
                ),
                'cuda_peak_reserved_bytes': int(
                    torch.cuda.max_memory_reserved()
                ),
                'cuda_total_bytes': int(
                    torch.cuda.get_device_properties(
                        torch.cuda.current_device()
                    ).total_memory
                ),
            }
        )
    except Exception:
        # Runtime diagnostics must never turn a valid perception request into
        # a failure. CUDA can become unavailable independently of inference.
        pass
    return values


class RuntimeMonitor:
    def __init__(
        self,
        enabled: bool,
        request_kind: str,
        include_cuda: bool,
        *,
        clock: Callable[[], float] = time.monotonic,
        memory_reader: MemoryReader = read_memory,
    ) -> None:
        self._enabled = enabled
        self._request_kind = request_kind
        self._include_cuda = include_cuda
        self._clock = clock
        self._memory_reader = memory_reader
        self._started = clock()
        self._stage_name: str | None = None
        self._stage_started = self._started
        self._durations: dict[str, float] = {}
        self._cuda_metrics_started = False
        self._memory_start = self._read_memory(
            include_cuda=False,
            reset_cuda_peak=False,
        )

    def begin_stage(self, name: str) -> None:
        if not self._enabled:
            return
        if not name:
            raise ValueError('runtime stage name must not be empty')
        now = self._clock()
        self._finish_active_stage(now)
        self._stage_name = name
        self._stage_started = now
        if (
            self._include_cuda
            and not self._cuda_metrics_started
            and name == 'sam2'
        ):
            self._read_memory(
                include_cuda=True,
                reset_cuda_peak=True,
            )
            self._cuda_metrics_started = True

    def finish(
        self,
        *,
        success: bool,
        error_code: int,
        message: str,
        snapshot_id: str,
        selected_object_id: int,
    ) -> dict[str, Any] | None:
        if not self._enabled:
            return None
        finished = self._clock()
        self._finish_active_stage(finished)
        memory_end = self._read_memory(
            include_cuda=self._cuda_metrics_started,
            reset_cuda_peak=False,
        )
        rss_start = self._memory_start.get('rss_bytes')
        rss_end = memory_end.get('rss_bytes')
        system_total = memory_end.get('system_total_bytes')
        system_available = memory_end.get('system_available_bytes')
        system_used = (
            system_total - system_available
            if isinstance(system_total, int)
            and isinstance(system_available, int)
            else None
        )
        cuda_peak = memory_end.get('cuda_peak_allocated_bytes')
        cuda_total = memory_end.get('cuda_total_bytes')
        rss_delta = (
            rss_end - rss_start
            if isinstance(rss_start, int) and isinstance(rss_end, int)
            else None
        )
        return {
            'schema_version': 1,
            'recorded_at': datetime.now(timezone.utc).isoformat(),
            'request_kind': self._request_kind,
            'snapshot_id': snapshot_id,
            'selected_object_id': selected_object_id,
            'success': success,
            'error_code': error_code,
            'message': message,
            'timing_seconds': {
                **self._durations,
                'total': finished - self._started,
            },
            'memory': {
                'rss_start_bytes': rss_start,
                'rss_end_bytes': rss_end,
                'rss_end_mib': _mib(rss_end),
                'rss_delta_bytes': rss_delta,
                'process_peak_rss_bytes': memory_end.get(
                    'process_peak_rss_bytes'
                ),
                'process_peak_rss_mib': _mib(
                    memory_end.get('process_peak_rss_bytes')
                ),
                'system_total_bytes': system_total,
                'system_total_gib': _gib(system_total),
                'system_available_end_bytes': system_available,
                'system_used_end_bytes': system_used,
                'system_used_end_gib': _gib(system_used),
                'system_used_end_percent': _percent(
                    system_used,
                    system_total,
                ),
                'rss_end_percent_of_system': _percent(
                    rss_end,
                    system_total,
                ),
                'cuda_allocated_end_bytes': memory_end.get(
                    'cuda_allocated_bytes'
                ),
                'cuda_reserved_end_bytes': memory_end.get(
                    'cuda_reserved_bytes'
                ),
                'cuda_peak_allocated_bytes': memory_end.get(
                    'cuda_peak_allocated_bytes'
                ),
                'cuda_peak_allocated_mib': _mib(cuda_peak),
                'cuda_peak_reserved_bytes': memory_end.get(
                    'cuda_peak_reserved_bytes'
                ),
                'cuda_total_bytes': cuda_total,
                'cuda_total_gib': _gib(cuda_total),
                'cuda_peak_allocated_percent_of_total': _percent(
                    cuda_peak,
                    cuda_total,
                ),
            },
        }

    def _finish_active_stage(self, finished: float) -> None:
        if self._stage_name is None:
            return
        elapsed = finished - self._stage_started
        self._durations[self._stage_name] = (
            self._durations.get(self._stage_name, 0.0) + elapsed
        )
        self._stage_name = None

    def _read_memory(
        self,
        *,
        include_cuda: bool,
        reset_cuda_peak: bool,
    ) -> dict[str, MemoryValue]:
        if not self._enabled:
            return {}
        try:
            return self._memory_reader(
                include_cuda,
                reset_cuda_peak,
            )
        except Exception:
            return {}


def _percent(value: MemoryValue, total: MemoryValue) -> float | None:
    if not isinstance(value, (int, float)):
        return None
    if not isinstance(total, (int, float)) or total <= 0:
        return None
    return round(float(value) / float(total) * 100.0, 2)


def _mib(value: MemoryValue) -> float | None:
    if not isinstance(value, (int, float)):
        return None
    return round(float(value) / (1024**2), 2)


def _gib(value: MemoryValue) -> float | None:
    if not isinstance(value, (int, float)):
        return None
    return round(float(value) / (1024**3), 2)


class SnapshotArtifactStore:
    def __init__(self, root: Path) -> None:
        self._root = root.expanduser()

    @property
    def root(self) -> Path:
        return self._root

    def save_rgb(
        self,
        snapshot_id: str,
        filename: str,
        rgb: np.ndarray,
    ) -> Path:
        path = self._snapshot_path(snapshot_id, filename, '.png')
        image = np.asarray(rgb, dtype=np.uint8)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError('debug RGB image must have shape HxWx3')
        PilImage.fromarray(image).save(path)
        return path

    def save_json(
        self,
        snapshot_id: str,
        filename: str,
        value: dict[str, Any],
    ) -> Path:
        path = self._snapshot_path(snapshot_id, filename, '.json')
        path.write_text(
            json.dumps(value, indent=2, ensure_ascii=False) + '\n',
            encoding='utf-8',
        )
        return path

    def _snapshot_path(
        self,
        snapshot_id: str,
        filename: str,
        required_suffix: str,
    ) -> Path:
        if _SNAPSHOT_ID_PATTERN.fullmatch(snapshot_id) is None:
            raise ValueError(f'invalid snapshot ID: {snapshot_id!r}')
        if Path(filename).name != filename or not filename.endswith(
            required_suffix
        ):
            raise ValueError(f'invalid artifact filename: {filename!r}')
        directory = self._root / snapshot_id
        directory.mkdir(parents=True, exist_ok=True)
        return directory / filename
