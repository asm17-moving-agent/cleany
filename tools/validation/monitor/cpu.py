#!/usr/bin/env python3
"""Compare monitor CPU with synthetic ROS input in an isolated DDS domain.

Source the built ROS overlay first. This tool publishes observation messages only;
it never starts the robot runtime or calls manipulation control services.
"""

import argparse
import asyncio
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import threading
import time


def worker(args):
    sys.path.insert(0, str(args.source.resolve()))
    from cleany_dev_monitor.node import main

    main([
        '--ros-args', '-p', f'port:={args.port}', '-p', 'host:=127.0.0.1',
        '-p', f'state_directory:={args.output / "recordings"}',
    ])


async def measure(args, process):
    import psutil
    from aiohttp import ClientSession

    target = psutil.Process(process.pid)
    base = f'http://127.0.0.1:{args.port}'
    packets, rows, gaps, latencies = 0, 0, 0, []
    async with ClientSession() as client:
        for _ in range(150):
            if process.poll() is not None:
                raise RuntimeError('Monitor exited; inspect monitor.log')
            try:
                async with client.get(base + '/api/snapshot') as response:
                    await response.json()
                break
            except OSError:
                await asyncio.sleep(.1)
        async with client.ws_connect(base + '/ws') as ws:
            await ws.send_json({'op': 'select', 'topics': [f'/cpu_benchmark/topic_{i}' for i in range(16)]})
            warmup = time.monotonic() + 5
            while time.monotonic() < warmup:
                await asyncio.wait_for(ws.receive_json(), timeout=3)
            cpu_start = target.cpu_times()
            started = time.monotonic()

            async def poll():
                while time.monotonic() - started < args.seconds:
                    before = time.monotonic()
                    async with client.get(base + '/api/control/status') as response:
                        assert response.status == 200
                        await response.read()
                    latencies.append(time.monotonic() - before)
                    async with client.get(base + '/api/recordings') as response:
                        assert response.status == 200
                        await response.read()
                    await asyncio.sleep(2)

            polling = asyncio.create_task(poll())
            try:
                while time.monotonic() - started < args.seconds:
                    payload = await asyncio.wait_for(ws.receive_json(), timeout=3)
                    if 'sequence' in payload:
                        packets += 1
                        rows += len(payload['rows'])
                        gaps += bool(payload.get('gap'))
            finally:
                polling.cancel()
                await asyncio.gather(polling, return_exceptions=True)
            elapsed = time.monotonic() - started
            cpu_end = target.cpu_times()
            return dict(
                seconds=elapsed,
                cpu_percent_one_core=100 * (cpu_end.user + cpu_end.system - cpu_start.user - cpu_start.system) / elapsed,
                rss_mib=target.memory_info().rss / 1024**2,
                websocket_packets=packets, websocket_rows=rows, websocket_gaps=gaps,
                status_max_ms=1000 * max(latencies),
            )


def benchmark(args):
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    source = rclpy.create_node('monitor_cpu_benchmark_source')
    topics = [source.create_publisher(String, f'/cpu_benchmark/topic_{i}', 5) for i in range(48)]
    scan = source.create_publisher(LaserScan, '/scan', 5)
    odometry = source.create_publisher(Odometry, '/wheel/odom', 5)
    transforms = source.create_publisher(TFMessage, '/tf', 5)
    counter = [0]

    def publish():
        counter[0] += 1
        stamp = source.get_clock().now().to_msg()
        position = math.sin(counter[0] / 30)
        message = Odometry()
        message.header.stamp, message.header.frame_id, message.child_frame_id = stamp, 'map', 'base_link'
        message.pose.pose.position.x, message.pose.pose.orientation.w = position, 1.
        odometry.publish(message)
        transform = TransformStamped()
        transform.header, transform.child_frame_id = message.header, 'base_link'
        transform.transform.translation.x, transform.transform.rotation.w = position, 1.
        transforms.publish(TFMessage(transforms=[transform]))
        laser = LaserScan(angle_min=-1.57, angle_max=1.57, angle_increment=3.14 / 511,
                          range_min=.05, range_max=10., ranges=[2. + position * .1] * 512)
        laser.header.stamp, laser.header.frame_id = stamp, 'base_link'
        scan.publish(laser)
        if counter[0] % 4 == 0:
            for publisher in topics:
                publisher.publish(String(data=json.dumps({'tick': counter[0], 'payload': 'x' * 256})))

    source.create_timer(.05, publish)
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    environment = dict(os.environ)
    with (args.output / 'monitor.log').open('w') as log:
        process = subprocess.Popen([
            sys.executable, str(Path(__file__).resolve()), '--worker',
            '--source', str(args.source.resolve()), '--output', str(args.output.resolve()),
            '--port', str(args.port),
        ], env=environment, stdout=log, stderr=subprocess.STDOUT)
        try:
            report = asyncio.run(measure(args, process))
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            executor.shutdown()
            thread.join(timeout=5)
            source.destroy_node()
            rclpy.shutdown()
    archives = list((args.output / 'recordings').glob('*.sqlite3'))
    recorded, raw, bag_messages = 0, 0, 0
    finalized, gaps = True, []
    for archive in archives:
        with sqlite3.connect(archive) as db:
            recorded += db.execute('SELECT count(*) FROM records').fetchone()[0]
            raw += db.execute('SELECT count(*) FROM raw_index').fetchone()[0]
            metadata = dict(db.execute('SELECT key,value FROM metadata'))
            finalized = finalized and metadata.get('bag_state') == 'finalized'
            gaps.extend(metadata[key] for key in ('gap', 'raw_gap') if key in metadata)
    for bag in (args.output / 'recordings/bags').glob('*/*.db3'):
        with sqlite3.connect(bag) as db:
            bag_messages += db.execute('SELECT count(*) FROM messages').fetchone()[0]
    report.update(source=str(args.source.resolve()), monitor_exit_code=process.returncode,
                  processed_records=recorded, raw_records=raw,
                  bag_messages=bag_messages, bag_finalized=finalized, recording_gaps=gaps,
                  ros_domain_id=os.environ['ROS_DOMAIN_ID'], synthetic_publishers=51)
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    assert process.returncode == 0 and recorded > 0 and raw == bag_messages > 0
    assert finalized and not gaps and report['websocket_gaps'] == 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path('ros2_ws/src/cleany_dev_monitor'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--port', type=int, default=8779)
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    arguments = parser.parse_args()
    # Isolate all test topics from the real robot, regardless of caller defaults.
    os.environ.update(ROS_DOMAIN_ID='180', ROS_LOCALHOST_ONLY='1')
    os.environ.pop('CYCLONEDDS_URI', None)
    worker(arguments) if arguments.worker else benchmark(arguments)
