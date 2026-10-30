#!/usr/bin/env python3
"""Exercise the display on isolated topics without starting robot controllers."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from urllib.request import urlopen

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from cleany_interfaces.msg import MissionStatus, MissionResult, ManipulationExecutionRecord
from sensor_msgs.msg import BatteryState


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', required=True)
    options = parser.parse_args()
    config = json.loads(Path(options.config).read_text())
    config['port'] = 8771
    config['topics'] = {key: '/cleany_display_validation/' + key for key in config['topics']}
    assert os.environ.get('ROS_DOMAIN_ID') == '173', 'Validation requires isolated ROS_DOMAIN_ID=173'
    output = Path(options.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    results = []
    with tempfile.TemporaryDirectory(prefix='cleany-display-') as temp, output.with_suffix('.log').open('w') as log:
        config_file = Path(temp)/'display.json'
        config_file.write_text(json.dumps(config))
        process = subprocess.Popen([sys.executable, '-m', 'cleany_display.node', '--config', str(config_file)],
                                   stdout=log, stderr=subprocess.STDOUT)
        node = None
        try:
            rclpy.init()
            node = rclpy.create_node('cleany_display_validation')
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            mission = node.create_publisher(MissionStatus, config['topics']['mission'], qos)
            battery = node.create_publisher(BatteryState, config['topics']['battery'], 10)
            manip = node.create_publisher(ManipulationExecutionRecord, config['topics']['manipulation'], qos)

            def expect(mode, emit=lambda: None, seconds=12):
                deadline = time.monotonic()+seconds
                last = None
                while time.monotonic() < deadline:
                    emit()
                    rclpy.spin_once(node, timeout_sec=.1)
                    try:
                        with urlopen('http://127.0.0.1:8771/api/state', timeout=1) as response:
                            last = json.load(response)
                        if last['mode'] == mode:
                            results.append({'expected': mode, 'actual': last})
                            return last
                    except OSError:
                        pass
                raise AssertionError((mode, last, process.poll()))

            expect('idle')
            expect('moving', lambda: mission.publish(MissionStatus(state='NAVIGATE_TO_TARGET', mission_id='validation')))
            expect('checking', lambda: mission.publish(MissionStatus(state='WORKING', bt_stage='ObserveBefore', mission_id='validation')))
            expect('collecting', lambda: mission.publish(MissionStatus(state='WORKING', bt_stage='ExecuteOne', mission_id='validation')))
            charge = expect('charging', lambda: battery.publish(BatteryState(power_supply_status=1, percentage=.72)))
            assert charge['battery_percent'] == 72
            expect('help', lambda: manip.publish(ManipulationExecutionRecord(human_confirmation_required=True, updated_at_ns=time.time_ns())))
            battery.publish(BatteryState(power_supply_status=2))
            mission.publish(MissionStatus(state='IDLE'))
            completed = ManipulationExecutionRecord(record_state='FINISHED', has_result=True, status='SUCCESS', updated_at_ns=time.time_ns())
            expect('complete', lambda: manip.publish(completed))
            # Remove all observed publishers, retaining the display itself.
            node.destroy_publisher(mission); node.destroy_publisher(battery); node.destroy_publisher(manip)
            lost = expect('help', seconds=18)
            assert lost['source'] == 'disconnected'
            mission = node.create_publisher(MissionStatus, config['topics']['mission'], qos)
            expect('idle', lambda: mission.publish(MissionStatus(state='IDLE')))
            expect('moving', lambda: mission.publish(MissionStatus(state='RETURN_HOME', mission_id='second')))
            mission.publish(MissionStatus(state='IDLE'))
            result = node.create_publisher(MissionResult, config['topics']['result'], qos)
            completed = expect('complete', lambda: result.publish(MissionResult(
                mission_id='second', outcome='SUCCESS', completed_tasks=['first', 'second'])))
            assert completed['collected_count'] == 2
            output.write_text(json.dumps({'domain': 173, 'results': results}, ensure_ascii=False, indent=2))
            print(f'PASS: {len(results)} isolated ROS/HTTP scenarios')
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            process.terminate()
            try: process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()


if __name__ == '__main__':
    main()
