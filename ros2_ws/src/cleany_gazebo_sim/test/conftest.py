from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import pytest


@dataclass(frozen=True)
class RuntimeTestOptions:
    profile: str
    sensor_profile: str
    warmup_sec: float
    measure_sec: float
    startup_timeout_sec: float
    min_rtf: float | None
    min_camera_sim_hz: float | None
    min_lidar_sim_hz: float | None
    min_imu_sim_hz: float | None


def pytest_addoption(parser: pytest.Parser) -> None:
    group = parser.getgroup('Gazebo runtime and evaluation tests')
    group.addoption(
        '--run-sim-runtime',
        action='store_true',
        default=False,
        help='Run opt-in Gazebo runtime tests.',
    )
    group.addoption(
        '--sim-profile',
        choices=('fortress', 'harmonic'),
        default='fortress',
        help='Gazebo profile used by the runtime test (default: fortress).',
    )
    group.addoption(
        '--sensor-profile',
        choices=(
            'lidar_nav',
            'head_rgbd',
            'left_wrist',
            'right_wrist',
            'all_cameras',
        ),
        default='all_cameras',
        help='Sensor workload used by the runtime test (default: all_cameras).',
    )
    group.addoption('--warmup-sec', type=float, default=10.0)
    group.addoption('--measure-sec', type=float, default=30.0)
    group.addoption('--startup-timeout-sec', type=float, default=60.0)
    group.addoption('--min-rtf', type=float, default=None)
    group.addoption('--min-camera-sim-hz', type=float, default=None)
    group.addoption('--min-lidar-sim-hz', type=float, default=None)
    group.addoption('--min-imu-sim-hz', type=float, default=None)
    group.addoption(
        '--run-evaluation-tests',
        action='store_true',
        default=False,
        help='Run temporary SLAM and study-cafe evaluation checks.',
    )


def pytest_ignore_collect(
    collection_path: Path,
    config: pytest.Config,
) -> bool | None:
    if (
        'evaluation' in collection_path.parts
        and not config.getoption('--run-evaluation-tests')
    ):
        return True
    return None


@pytest.fixture
def runtime_test_options(request: pytest.FixtureRequest) -> RuntimeTestOptions:
    if not request.config.getoption('--run-sim-runtime'):
        pytest.skip('pass --run-sim-runtime to run the Gazebo runtime test')

    options = RuntimeTestOptions(
        profile=request.config.getoption('--sim-profile'),
        sensor_profile=request.config.getoption('--sensor-profile'),
        warmup_sec=request.config.getoption('--warmup-sec'),
        measure_sec=request.config.getoption('--measure-sec'),
        startup_timeout_sec=request.config.getoption('--startup-timeout-sec'),
        min_rtf=request.config.getoption('--min-rtf'),
        min_camera_sim_hz=request.config.getoption('--min-camera-sim-hz'),
        min_lidar_sim_hz=request.config.getoption('--min-lidar-sim-hz'),
        min_imu_sim_hz=request.config.getoption('--min-imu-sim-hz'),
    )
    durations = {
        '--warmup-sec': options.warmup_sec,
        '--measure-sec': options.measure_sec,
        '--startup-timeout-sec': options.startup_timeout_sec,
    }
    for name, value in durations.items():
        if value <= 0.0:
            pytest.fail(f'{name} must be greater than zero')

    thresholds = {
        '--min-rtf': options.min_rtf,
        '--min-camera-sim-hz': options.min_camera_sim_hz,
        '--min-lidar-sim-hz': options.min_lidar_sim_hz,
        '--min-imu-sim-hz': options.min_imu_sim_hz,
    }
    for name, value in thresholds.items():
        if value is not None and value < 0.0:
            pytest.fail(f'{name} must be zero or greater')

    return options
