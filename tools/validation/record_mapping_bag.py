#!/usr/bin/env python3
"""Logger 가공과 독립적으로 native rosbag2로 mapping 입력을 기록합니다."""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import signal
import subprocess

TOPICS = ('/scan_raw', '/odom', '/tf', '/tf_static', '/base/wheel_state', '/joint_states')


def command(output: Path, cache_mib: int = 64) -> list[str]:
    if output.exists():
        raise ValueError(f'Refusing to overwrite an existing bag: {output}')
    if cache_mib <= 0:
        raise ValueError('cache_mib must be positive')
    return ['ros2', 'bag', 'record', '--storage', 'sqlite3',
            '--storage-preset-profile', 'resilient', '--max-cache-size',
            str(cache_mib * 1024**2), '--output', str(output), *TOPICS]


def record(output: Path, duration_s: float, cache_mib: int = 64) -> int:
    """시간 초과 또는 Ctrl-C 시 소유한 recorder에 SIGINT를 보내 flush합니다."""
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError('duration_s must be positive')
    arguments = command(output, cache_mib)
    output.parent.mkdir(parents=True, exist_ok=True)
    process = subprocess.Popen(arguments, stdin=subprocess.DEVNULL, start_new_session=True)
    interrupted = False
    try:
        process.wait(timeout=duration_s)
    except (subprocess.TimeoutExpired, KeyboardInterrupt):
        interrupted = True
        process.send_signal(signal.SIGINT)
        process.wait(timeout=30)
    return 0 if interrupted and process.returncode in (0, -signal.SIGINT, 130) else process.returncode


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration-s', type=float, default=180.)
    parser.add_argument('--cache-mib', type=int, default=64)
    args = parser.parse_args()
    return record(args.output.expanduser().resolve(), args.duration_s, args.cache_mib)


if __name__ == '__main__':
    raise SystemExit(main())
