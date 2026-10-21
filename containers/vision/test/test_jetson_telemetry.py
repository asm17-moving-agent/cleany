from __future__ import annotations

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from record_jetson_telemetry import _parse_tegrastats, _statistics  # noqa: E402


def test_tegrastats_cpu_normalization_and_temperature() -> None:
    result = _parse_tegrastats(
        'RAM 7319/15656MB SWAP 0/7828MB CPU [100%@1497,off,0%@1497,0%@1497] '
        'GR3D_FREQ 99% cpu@64C gpu@61.625C tj@64C VDD_IN 6146mW/7055mW'
    )
    assert result['cpu_percent'] == 25.0
    assert result['gpu_percent'] == 99.0
    assert result['gpu_temperature_c'] == 61.625
    assert result['power_mw'] == 6146
    assert result['ram_used_mb'] == 7319
    assert _parse_tegrastats('unavailable') == {}


def test_statistics_distinguishes_unmeasured_from_zero() -> None:
    assert _statistics([])['mean'] is None
    assert _statistics([0]) == {'count': 1, 'mean': 0.0, 'p95': 0.0, 'peak': 0}
    result = _statistics([0, 10, 20])
    assert result['mean'] == 10
    assert result['p95'] == pytest.approx(19)
    assert result['peak'] == 20
