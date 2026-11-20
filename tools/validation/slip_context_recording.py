#!/usr/bin/env python3
"""Verify native diagnostic CDR recording on a non-hardware ROS domain."""
import argparse
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) == 0:
        parser.error('Synthetic messages are forbidden on hardware domain 0')
    args.output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    from cleany_interfaces.srv import RecordingControl
    rclpy.init()
    node = rclpy.create_node('slip_recording_probe')
    messages = {
        '/navigation/slip': dict(state='confirmed', t0=100., t1=101., context=dict(
            at=101., frame='map', pose=[1., 2., .3], map_revision='synthetic',
            command_time_basis='ros_receive_time', commands=dict(requested=dict(
                velocity=[.05, .1, 0.], received_at=100.95)))),
        '/navigation/traction_recovery': dict(phase='braking', reason='slip', retry=False),
    }
    publishers = {name: node.create_publisher(String, name, 10) for name in messages}
    log = (args.output/'logger.log').open('w')
    process = subprocess.Popen(['ros2', 'run', 'cleany_logger', 'logger_node', '--ros-args',
        '-p', f'state_directory:={args.output}/recordings', '-p', 'record_on_start:=true'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        client = node.create_client(RecordingControl, '/cleany_logger/get_config')
        assert client.wait_for_service(timeout_sec=12), 'Logger unavailable'
        future = client.call_async(RecordingControl.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=8)
        response = future.result()
        assert response and response.success and response.status.state == 'RECORDING'
        deadline = time.monotonic()+12
        sent = 0
        while time.monotonic() < deadline:
            for name, publisher in publishers.items():
                publisher.publish(String(data=json.dumps(messages[name])))
            sent += 1
            rclpy.spin_once(node, timeout_sec=.1)
        client = node.create_client(RecordingControl, '/cleany_logger/stop')
        assert client.wait_for_service(timeout_sec=3)
        future = client.call_async(RecordingControl.Request(session_id=response.status.session_id))
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        assert future.result() and future.result().success
    finally:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=12)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
    counts = {name: 0 for name in messages}
    for path in (args.output/'recordings/bags').rglob('*.db3'):
        with sqlite3.connect(path) as db:
            for name, data in db.execute('SELECT topics.name, messages.data FROM messages JOIN topics ON topics.id=messages.topic_id'):
                if name in messages:
                    assert json.loads(deserialize_message(data, String).data) == messages[name]
                    counts[name] += 1
    assert all(n > 10 for n in counts.values()), counts
    result = dict(domain=os.environ['ROS_DOMAIN_ID'], sent_each=sent, recorded=counts,
                  exact_payload_roundtrip=True, synthetic=True)
    (args.output/'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result))


if __name__ == '__main__':
    main()
