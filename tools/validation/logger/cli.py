#!/usr/bin/env python3
"""Exercise the installed CLI against an independent logger with synthetic input."""
import argparse
import json
import os
import signal
import sqlite3
import subprocess
import time
from pathlib import Path

import rclpy
import rosbag2_py
from ament_index_python.packages import get_package_prefix
from cleany_interfaces.srv import RecordingControl
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--base-commit', required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('Use an isolated nonzero ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
    args.output.mkdir(parents=True, exist_ok=False)
    storage = args.output / 'recordings'
    executable = Path(get_package_prefix('cleany_logger')) / 'lib/cleany_logger'
    report = dict(base_commit=args.base_commit, domain=os.environ['ROS_DOMAIN_ID'],
                  input='Numbered Path and RGB Image on /cli/plan and /cli/image; no dashboard',
                  commands=[], failures=[], success=False)
    processes = []
    rclpy.init()
    source = rclpy.create_node('logger_cli_probe', enable_rosout=False)
    path_pub = source.create_publisher(RosPath, '/cli/plan', 10)
    image_pub = source.create_publisher(Image, '/cli/image', 10)
    def slow_response(request, response):
        time.sleep(1.5)  # Deliberately exceed the CLI response deadline.
        return response
    source.create_service(RecordingControl, '/cli/slow/get_config', slow_response)
    sequence = 0
    def pump(seconds):
        nonlocal sequence
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            sequence += 1
            stamp = source.get_clock().now().to_msg()
            point = PoseStamped()
            point.pose.position.x = float(sequence)
            message = RosPath(poses=[point])
            message.header.frame_id = 'map'
            message.header.stamp = stamp
            path_pub.publish(message)
            image = Image(height=1, width=1, encoding='rgb8', step=3, data=sequence.to_bytes(3, 'little'))
            image.header.frame_id = 'camera'
            image.header.stamp = stamp
            image_pub.publish(image)
            rclpy.spin_once(source, timeout_sec=.02)
            time.sleep(.02)
    def terminate(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(3)
    def cli(*arguments, expected=0, timeout=3.):
        number = len(report['commands'])
        prefix = args.output / f'command-{number:02}'
        command = [str(executable / 'logger'), '--node', '/cli/recorder', '--timeout', str(timeout), '--json', *arguments]
        started = time.monotonic()
        with prefix.with_suffix('.stdout').open('w+') as stdout, prefix.with_suffix('.stderr').open('w+') as stderr:
            process = subprocess.Popen(command, stdout=stdout, stderr=stderr, start_new_session=True)
            processes.append(process)
            while process.poll() is None and time.monotonic() - started < 15:
                pump(.05)
            if process.poll() is None:
                terminate(process)
                raise AssertionError(f'CLI did not finish: {arguments}')
            stdout.seek(0); stderr.seek(0)
            out, err = stdout.read(), stderr.read()
        report['commands'].append(dict(arguments=list(arguments), exit_code=process.returncode,
                                       elapsed_s=round(time.monotonic()-started, 3), stdout=prefix.with_suffix('.stdout').name,
                                       stderr=prefix.with_suffix('.stderr').name))
        assert process.returncode == expected, (arguments, process.returncode, out, err)
        if expected:
            assert not out and json.loads(err)['exit_code'] == expected
            return json.loads(err)
        return json.loads(out)
    logger_log = (args.output / 'logger.log').open('w')
    try:
        logger = subprocess.Popen([str(executable / 'logger_node'), '--ros-args',
            '-r', '__node:=recorder', '-r', '__ns:=/cli',
            '-p', f'state_directory:={storage}', '-p', 'record_on_start:=false'],
            stdout=logger_log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(logger)
        assert cli('status')['state'] == 'IDLE'
        assert list(storage.glob('*.sqlite3')) == []
        cli('--node', '/invalid-node', 'status', expected=2)
        original = cli('topics')
        assert any(row['id'] == 'camera' and not row['enabled'] for row in original['topics'])
        cli('topics', 'set', 'path_topic', '--topic', '/cli/plan', '--hz', '3', '--enable')
        cli('topics', 'set', 'camera', '--topic', '/cli/image', '--hz', '2', '--enable')
        cli('topics', 'set', 'local_path_topic', '--disable')
        changed = cli('topics', 'set', 'velocity_topics', '--topic', '/cli/v1', '--topic', '/cli/v2')
        assert changed['settings']['sources']['velocity_topics'] == ['/cli/v1', '/cli/v2']
        assert changed['settings']['sources']['path_topic'] == '/cli/plan'
        assert not changed['settings']['source_enabled']['local_path_topic']
        revision = changed['revision']
        cli('topics', 'set', 'camera', '--topic', '/bad//image', expected=2)
        assert cli('status')['revision'] == revision
        active = cli('start')
        assert active['state'] == 'RECORDING'
        session = active['session_id']
        assert cli('start')['session_id'] == session
        cli('topics', 'set', 'camera', '--disable', expected=4)
        cli('stop', '--session', 'old-session', expected=4)
        pump(3)
        assert cli('status')['state'] == 'RECORDING'
        with sqlite3.connect(f'file:{storage / (session + ".sqlite3")}?mode=ro', uri=True) as db:
            processed = db.execute('SELECT count(*) FROM records').fetchone()[0]
            raw = db.execute('SELECT count(*) FROM raw_index').fetchone()[0]
        assert processed > 0 and raw > 0
        stopped = cli('stop')
        assert stopped['state'] == 'IDLE' and stopped['session_id'] == session
        with sqlite3.connect(storage / f'{session}.sqlite3') as db:
            assert dict(db.execute('SELECT key,value FROM metadata'))['bag_state'] == 'finalized'
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(storage / 'bags' / session), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        actual = {'/cli/plan': [], '/cli/image': []}
        while reader.has_next():
            topic, payload, _ = reader.read_next()
            if topic in actual:
                message = deserialize_message(payload, RosPath if topic.endswith('plan') else Image)
                number = int(message.poses[0].pose.position.x) if topic.endswith('plan') else int.from_bytes(message.data, 'little')
                assert 0 < number <= sequence
                actual[topic].append(number)
        assert all(actual.values())
        # Closing each one-shot CLI above did not stop the independent writer.
        second = cli('start')
        assert second['session_id'] != session
        cli('stop')
        delayed = cli('--node', '/cli/slow', 'status', expected=3, timeout=.8)
        assert 'timed out' in delayed['error'] and 'unknown' in delayed['error']
        terminate(logger)
        assert logger.returncode == 0
        offline = cli('status', expected=3, timeout=.4)
        assert 'unavailable' in offline['error'].lower() or 'timed out' in offline['error'].lower()
        assert report['commands'][-1]['elapsed_s'] < 3
        report.update(success=True, session_id=session, processed_rows=processed, raw_rows=raw,
                      bag_samples={topic: len(numbers) for topic, numbers in actual.items()},
                      logger_exit=logger.returncode, cli_exit_does_not_stop_recording=True,
                      active_configuration_rejected=True, stale_session_stop_rejected=True,
                      invalid_topic_does_not_change_revision=True, response_timeout_reports_unknown=True)
    except Exception as exc:
        report['failures'].append(repr(exc))
        raise
    finally:
        for process in reversed(processes):
            terminate(process)
        source.destroy_node()
        rclpy.shutdown()
        logger_log.close()
        (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
