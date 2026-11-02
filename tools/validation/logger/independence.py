#!/usr/bin/env python3
"""Verify installed Logger and Monitor as separate processes using synthetic ROS input.

Run in a sourced overlay, with an isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1.
No motion command, action or robot control endpoint is used.
"""
import argparse
import json
import os
import signal
import sqlite3
import subprocess
import time
import urllib.error
import urllib.request
from pathlib import Path

import rclpy
import rosbag2_py
from ament_index_python.packages import get_package_prefix
from cleany_interfaces.srv import RecordingControl
from cleany_logger.recording import defaults
from cleany_logger.wire import config_message
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, PointCloud2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, default=18878)
    parser.add_argument('--base-commit', required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('Use an isolated nonzero ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
    args.output.mkdir(parents=True, exist_ok=False)
    storage = args.output / 'recordings'
    processes, streams = [], []
    report = dict(environment=dict(domain=os.environ['ROS_DOMAIN_ID'], distro=os.environ.get('ROS_DISTRO'),
                  localhost_only=True), input='Synthetic stamped Path, numbered RGB Image and PointCloud2 at approximately 20 Hz',
                  base_commit=args.base_commit,
                  output=str(args.output), failures=[])
    rclpy.init()
    source = rclpy.create_node('logger_independence_probe')
    path_pub = source.create_publisher(RosPath, '/plan', 10)
    image_pub = source.create_publisher(Image, '/camera/head/color/image_raw', 10)
    clients = {op: source.create_client(RecordingControl, '/cleany_logger/' + op)
               for op in ('start', 'stop', 'configure', 'get_config')}
    cloud_pub = source.create_publisher(PointCloud2, '/camera/head/depth/points', 10)
    sequence = 0
    published = {}
    def pump(seconds):
        nonlocal sequence
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            sequence += 1
            stamp = source.get_clock().now().to_msg()
            published[sequence] = stamp.sec * 10**9 + stamp.nanosec
            point = PoseStamped()
            point.header.frame_id = 'map'
            point.pose.position.x = float(sequence)
            message = RosPath(poses=[point])
            message.header.frame_id = 'map'
            message.header.stamp = stamp
            path_pub.publish(message)
            image = Image(height=1, width=1, encoding='rgb8', step=3,
                          data=sequence.to_bytes(3, 'little'))
            image.header.frame_id = 'camera'
            image.header.stamp = stamp
            image_pub.publish(image)
            cloud = PointCloud2(height=1, width=1, point_step=4, row_step=4,
                                data=sequence.to_bytes(4, 'little'))
            cloud.header.frame_id = 'camera'
            cloud.header.stamp = stamp
            cloud_pub.publish(cloud)
            rclpy.spin_once(source, timeout_sec=.03)
            time.sleep(.02)
    def spawn(package, executable, name, *parameters):
        stream = (args.output / (name + '.log')).open('w')
        streams.append(stream)
        command = [str(Path(get_package_prefix(package)) / 'lib' / package / executable), '--ros-args']
        for parameter in parameters:
            command += ['-p', parameter]
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process
    def terminate(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(3)
    def control(op, **fields):
        deadline = time.monotonic() + 12
        while not clients[op].service_is_ready() and time.monotonic() < deadline:
            pump(.1)
        assert clients[op].service_is_ready(), f'{op} service not ready'
        pending = clients[op].call_async(RecordingControl.Request(**fields))
        while not pending.done() and time.monotonic() < deadline:
            pump(.05)
        assert pending.done(), f'{op} timed out'
        return pending.result()
    def http(endpoint, data=None, method=None):
        request = urllib.request.Request(f'http://127.0.0.1:{args.port}{endpoint}',
            data=json.dumps(data).encode() if data is not None else None, method=method,
            headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=5) as response:
            return json.load(response)
    def await_http():
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            try:
                return http('/api/recordings')
            except (urllib.error.URLError, TimeoutError):
                pump(.1)
        raise AssertionError('Monitor HTTP did not become ready')
    def counts(session):
        with sqlite3.connect(f'file:{storage / (session + ".sqlite3")}?mode=ro', uri=True) as db:
            return dict(processed=db.execute('SELECT count(*) FROM records').fetchone()[0],
                        raw=db.execute('SELECT count(*) FROM raw_index').fetchone()[0])
    phases = {}
    try:
        logger = spawn('cleany_logger', 'logger_node', 'logger', f'state_directory:={storage}', 'record_on_start:=false')
        settings = defaults()
        settings['enabled']['camera'] = True
        settings['enabled']['point_cloud'] = True
        assert control('configure', config=config_message(settings)).success
        started = control('start')
        assert started.success, started.reason
        session = started.status.session_id
        assert control('start').status.session_id == session
        assert not control('configure', config=config_message(settings)).success
        assert not control('stop', session_id='wrong-session').success
        phases['before_monitor'] = dict(start=sequence)
        pump(3)
        phases['before_monitor'].update(end=sequence, **counts(session))
        assert phases['before_monitor']['raw'] > 0
        monitor = spawn('cleany_dev_monitor', 'dev_monitor', 'monitor', f'state_directory:={storage}', f'port:={args.port}')
        await_http()
        pump(2)
        current = http('/api/recording/settings')
        assert current['session_id'] == session
        try:
            http('/api/recording/settings', current['settings'], 'PUT')
            raise AssertionError('Active reconfiguration was accepted')
        except urllib.error.HTTPError as exc:
            assert exc.code == 409
        phases['with_monitor'] = dict(start=phases['before_monitor']['end'], end=sequence, **counts(session))
        terminate(monitor)
        assert monitor.poll() is not None and logger.poll() is None
        phases['after_monitor_exit'] = dict(start=sequence, monitor_pid=monitor.pid, logger_pid=logger.pid)
        pump(3)
        assert control('get_config').status.session_id == session
        phases['after_monitor_exit'].update(end=sequence, **counts(session))
        assert phases['after_monitor_exit']['processed'] > phases['with_monitor']['processed']
        assert phases['after_monitor_exit']['raw'] > phases['with_monitor']['raw']
        restarted = spawn('cleany_dev_monitor', 'dev_monitor', 'monitor-restarted', f'state_directory:={storage}', f'port:={args.port}')
        await_http()
        pump(1)
        assert http('/api/recording/settings')['session_id'] == session
        current = http('/api/recording/settings')
        acknowledgement = http('/api/recording/settings', {**current['settings'], 'active': False}, 'PUT')
        stopped = control('get_config')
        assert acknowledgement['state'] == stopped.status.state == 'IDLE'
        assert acknowledgement['session_id'] == session
        bag = storage / 'bags' / session
        assert (bag / 'metadata.yaml').is_file()
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
        actual = {'/plan': [], '/camera/head/color/image_raw': [], '/camera/head/depth/points': []}
        stamps = []
        while reader.has_next():
            topic, payload, timestamp = reader.read_next()
            if topic not in actual:
                continue
            message = deserialize_message(payload, RosPath if topic == '/plan' else Image if topic.endswith('image_raw') else PointCloud2)
            number = int(message.poses[0].pose.position.x) if topic == '/plan' else int.from_bytes(message.data, 'little')
            assert message.header.stamp.sec * 10**9 + message.header.stamp.nanosec == published[number]
            actual[topic].append(number)
            stamps.append(timestamp)
        for phase in phases.values():
            phase['bag_samples'] = {topic: sum(phase['start'] < number <= phase['end'] for number in numbers)
                                    for topic, numbers in actual.items()}
            assert all(phase['bag_samples'].values()), phase
        assert len(set(stamps)) > 3
        assert http('/api/recordings')['active'] == ''
        another = http('/api/recording/session', current['settings'], 'POST')
        assert another['session_id'] != session and another['state'] == 'RECORDING'
        http('/api/recording/settings', {**another['settings'], 'active': False}, 'PUT')
        terminate(logger)
        pump(3.5)
        assert http('/api/recordings')['state'] == 'UNAVAILABLE'
        try:
            http('/api/recording/settings')
            raise AssertionError('Offline logger returned a live configuration')
        except urllib.error.HTTPError as exc:
            assert exc.code == 503
        terminate(restarted)
        autostart = spawn('cleany_logger', 'logger_node', 'logger-autostart', f'state_directory:={storage}', 'record_on_start:=true')
        automatic = control('get_config')
        assert automatic.success and automatic.status.state == 'RECORDING'
        assert automatic.status.session_id not in (session, another['session_id'])
        pump(2)
        terminate(autostart)
        with sqlite3.connect(storage / f'{automatic.status.session_id}.sqlite3') as db:
            metadata = dict(db.execute('SELECT key,value FROM metadata'))
            assert metadata['bag_state'] == 'finalized'
        assert autostart.returncode == 0, f'Logger SIGINT exit: {autostart.returncode}'
        # A dashboard alone must not create an archive directory or recording file.
        absent = args.output / 'must-remain-absent'
        offline = spawn('cleany_dev_monitor', 'dev_monitor', 'monitor-offline', f'state_directory:={absent}', f'port:={args.port}')
        await_http()
        assert http('/api/recordings')['sessions'] == []
        assert not absent.exists()
        terminate(offline)
        report.update(success=True, session_id=session, phases=phases,
                      bag_totals={topic: len(numbers) for topic, numbers in actual.items()},
                      stop_state=stopped.status.state, metadata_finalized=True,
                      monitor_offline_creates_storage=False, logger_offline_state='UNAVAILABLE',
                      http_start_stop=True, original_header_stamps_verified=True,
                      autostart_and_active_sigint_finalization=True,
                      exit_codes={f"process_{process.pid}": process.returncode for process in processes})
    except Exception as exc:
        report.update(success=False, phases=phases)
        report['failures'].append(repr(exc))
        raise
    finally:
        for process in reversed(processes):
            terminate(process)
        for stream in streams:
            stream.close()
        source.destroy_node()
        rclpy.shutdown()
        (args.output / 'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
