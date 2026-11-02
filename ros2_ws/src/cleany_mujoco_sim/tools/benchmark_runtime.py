"""Measure a ROS simulation command and stop all owned processes afterward.

Requires ROS 2, sensor/clock messages and psutil. A launch exit code of zero
does not establish pick/sort/place success; inspect its log and stage artifacts.
"""

from __future__ import annotations

import argparse
from collections.abc import Iterable
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time

import psutil
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image


def _alive_processes(processes: Iterable[psutil.Process]) -> list[psutil.Process]:
    alive = []
    for process in processes:
        try:
            if process.is_running() and process.status() != psutil.STATUS_ZOMBIE:
                alive.append(process)
        except psutil.NoSuchProcess:
            pass
    return alive


def _stop_owned_processes(
    process: subprocess.Popen, tracked: Iterable[psutil.Process],
) -> list[int]:
    for termination_signal, timeout in (
        (signal.SIGINT, 12), (signal.SIGTERM, 5), (signal.SIGKILL, 5),
    ):
        if process.poll() is not None:
            break
        try:
            os.killpg(process.pid, termination_signal)
            process.wait(timeout=timeout)
        except ProcessLookupError:
            break
        except subprocess.TimeoutExpired:
            continue
    tracked = list(tracked)
    alive = _alive_processes(tracked)
    for child in alive:
        try:
            child.terminate()
        except psutil.NoSuchProcess:
            pass
    _, alive = psutil.wait_procs(alive, timeout=4)
    for child in alive:
        try:
            child.kill()
        except psutil.NoSuchProcess:
            pass
    psutil.wait_procs(alive, timeout=4)
    return [child.pid for child in _alive_processes(tracked)]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=35)
    parser.add_argument('--warmup', type=float, default=5)
    parser.add_argument('--output', type=Path, default=Path('/tmp/cleany-runtime.json'))
    parser.add_argument('--startup-timeout', type=float, default=120)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if (not all(math.isfinite(value) for value in (
            args.duration, args.warmup, args.startup_timeout))
            or args.duration <= 0 or args.warmup < 0 or args.startup_timeout <= 0):
        parser.error('duration/startup-timeout must be positive; warmup must be nonnegative')
    command = args.command
    if command and command[0] == '--':
        command = command[1:]
    if not command:
        parser.error('provide a command after --')
    profile = Path(__file__).parents[1] / 'config' / 'fastdds_rgbd.xml'
    # The probe must share the launch transport profile to avoid dropped large
    # RGB-D messages from an undersized default SHM segment.
    if not profile.is_file():
        profile = Path(__file__).parents[2] / 'cleany_perception/config/fastdds_rgbd.xml'
    if profile.is_file():
        os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE', str(profile))
    rclpy.init(args=[])
    node = rclpy.create_node('cleany_runtime_probe')
    clocks = []
    images = {}
    measuring = False
    first_clock = None
    latest_clock = None
    def clock_cb(msg: Clock) -> None:
        nonlocal first_clock, latest_clock
        latest_clock = msg.clock.sec + msg.clock.nanosec/1e9
        if first_clock is None:
            first_clock = time.monotonic()
        if measuring:
            clocks.append((time.monotonic(), latest_clock))
    subscriptions = [node.create_subscription(Clock, '/clock', clock_cb, qos_profile_sensor_data)]
    for topic in ('/camera/color/image_raw', '/camera/aligned_depth_to_color/image_raw', '/left_wrist_camera/image_raw', '/right_wrist_camera/image_raw'):
        images[topic] = []
        def callback(msg: Image, name: str = topic) -> None:
            if measuring:
                images[name].append((time.monotonic(), msg.header.stamp.sec + msg.header.stamp.nanosec/1e9))
        subscriptions.append(node.create_subscription(Image, topic, callback, qos_profile_sensor_data))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log_path = args.output.with_suffix('.log')
    tracked = {}
    cpu_deltas = {}
    cpu_last = {}
    process = None
    started = time.monotonic()
    report = {'command': command, 'scope': 'ROS runtime; clock, RGB-D delivery and process CPU', 'log': str(log_path)}
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            root = psutil.Process(process.pid)
            last_sample = 0
            measure_start = None
            while time.monotonic()-started < args.startup_timeout + args.warmup + args.duration:
                rclpy.spin_once(node, timeout_sec=0.02)
                now = time.monotonic()
                if process.poll() is not None:
                    report['launch_returncode'] = process.returncode
                    break
                if first_clock is not None and now-first_clock >= args.warmup and measure_start is None:
                    measure_start = now
                    measuring = True
                if now-last_sample >= 0.5:
                    last_sample = now
                    try:
                        processes = [root, *root.children(recursive=True)]
                    except psutil.NoSuchProcess:
                        processes = []
                    for child in processes:
                        try:
                            tracked[child.pid] = child
                            cpu = child.cpu_times()
                            total = cpu.user + cpu.system
                            if measuring and child.pid in cpu_last:
                                cpu_deltas[child.pid] = cpu_deltas.get(child.pid, 0) + total-cpu_last[child.pid]
                            cpu_last[child.pid] = total
                        except (psutil.NoSuchProcess, psutil.AccessDenied):
                            pass
                if measure_start is not None and now-measure_start >= args.duration:
                    break
            report['measurement_completed'] = (measure_start is not None and
                                               time.monotonic()-measure_start >= args.duration)
            if not clocks:
                raise RuntimeError(f'No simulation clock; see {log_path}')
            wall = clocks[-1][0]-clocks[0][0]
            if wall <= 0:
                raise RuntimeError('Insufficient clock samples to measure runtime')
            report.update(first_clock_seconds=first_clock-started, wall_seconds=wall,
                          realtime_factor=(clocks[-1][1]-clocks[0][1])/wall,
                          cpu_core_equivalents=sum(cpu_deltas.values())/wall)
            report['images'] = {}
            for topic, values in images.items():
                intervals = [b[0]-a[0] for a,b in zip(values, values[1:])]
                report['images'][topic] = {'frames':len(values), 'wall_hz':len(values)/wall,
                    'median_interval_ms':statistics.median(intervals)*1000 if intervals else None,
                    'max_interval_ms':max(intervals)*1000 if intervals else None}
    except (KeyboardInterrupt, RuntimeError, OSError, psutil.Error) as error:
        report['error'] = str(error) or 'Interrupted'
    finally:
        if process is not None:
            report['remaining_test_processes'] = _stop_owned_processes(
                process, tracked.values(),
            )
        node.destroy_node()
        rclpy.try_shutdown()
        args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return int(bool(report.get('error') or report.get('remaining_test_processes')
                    or report.get('launch_returncode', 0)))


if __name__ == '__main__':
    raise SystemExit(main())
