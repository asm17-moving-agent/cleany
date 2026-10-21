"""Isolated ROS synthetic IO/stage exercise. Never starts robot nodes or sends Goals."""
from datetime import datetime
import argparse
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, JointState
from control_msgs.msg import DynamicJointState, InterfaceValue
from std_msgs.msg import Bool, String
from cleany_interfaces.msg import ManipulationExecutionRecord

from cleany_dev_monitor.io_probe import load_config
from cleany_dev_monitor.io_probe_node import IOProbe, json_file, udp_counters
from cleany_dev_monitor.io_probe_report import render, read_recording


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    options = parser.parse_args()
    if (os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID', '0') in ('', '0')
            or os.environ.get('ROS_DOMAIN_ID') == os.environ.get('CLEANY_COLLECTION_ROS_DOMAIN_ID', '0')):
        raise SystemExit('Requires nonzero isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
    options.output.mkdir(parents=True, exist_ok=False)
    config = load_config(Path(__file__).parents[1]/'config/manipulation_io_probe.json')
    rclpy.init()
    source = rclpy.create_node('synthetic_io_source', enable_rosout=False)
    probe = IOProbe(config, options.output)
    executor = SingleThreadedExecutor()
    executor.add_node(source); executor.add_node(probe)
    latched = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    event_pub = source.create_publisher(ManipulationExecutionRecord, config['execution_topic'], latched)
    bt = source.create_publisher(String, config['bt_topic'], latched)
    state_pubs = {a: source.create_publisher(Bool, f'/{a}_wrist_camera/enabled', latched) for a in ('left', 'right')}
    types = dict(joint=JointState, motor=DynamicJointState, image=Image, info=CameraInfo)
    pubs = {s['topic']: source.create_publisher(types[s['kind']], s['topic'], qos_profile_sensor_data) for s in config['streams']}
    names = [f'{side}_{part}_joint' for side in ('left', 'right') for part in
             ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll', 'gripper')]
    names += ['head_pan_joint', 'head_tilt_joint']
    motor_stamp = source.get_clock().now().nanoseconds
    next_motor = next_camera = next_event = 0.
    accepted_ns = time.time_ns()
    revision = 0
    manifest = dict(config=config, label='격리 ROS 합성 시험 — 실제 파지 아님', synthetic=True,
                    shared_process_fixture=True,
                    finished=False, start=datetime.now().astimezone().isoformat(),
                    observation_start_wall_ns=probe.wall_ns, udp_before=udp_counters(), commands_sent=False)
    json_file(options.output/'manifest.json', manifest)
    probe.start_resources(True)
    try:
        while probe.elapsed() < 12.5:
            elapsed = probe.elapsed()
            stamp = source.get_clock().now().to_msg()
            left_on = 3 <= elapsed < 8
            if elapsed >= next_motor:
                next_motor = elapsed + .02
                if not 5 <= elapsed < 6.5:
                    motor_stamp = source.get_clock().now().nanoseconds
                joints = JointState(name=names, position=[0.]*14); joints.header.stamp = stamp
                pubs['/joint_states'].publish(joints)
                values = [InterfaceValue(interface_names=['position', 'feedback_stamp_ns', 'protocol_error_code',
                           'motor_status', 'torque_enabled', 'communication_recovery_count'],
                           values=[0., float(motor_stamp), 0., 0., 0., 0.]) for _ in range(12)]
                values += [InterfaceValue(interface_names=['position'], values=[0.]) for _ in range(2)]
                dynamic = DynamicJointState(joint_names=names, interface_values=values); dynamic.header.stamp = stamp
                pubs['/joint_state_broadcaster/dynamic_joint_states'].publish(dynamic)
            if elapsed >= next_camera:
                next_camera = elapsed + 1/15
                for spec in config['streams']:
                    if spec['kind'] not in ('image', 'info'): continue
                    if spec['mode'] == 'right' or (spec['mode'] == 'left' and not left_on): continue
                    if spec['topic'] == '/camera/color/image_raw' and 4.8 <= elapsed < 5.35: continue
                    if spec['kind'] == 'image':
                        depth = '/depth/' in spec['topic']; stride = 640*(2 if depth else 3)
                        message = Image(width=640, height=480, step=stride,
                                        encoding='16UC1' if depth else 'rgb8', data=bytes(480*stride))
                    else:
                        message = CameraInfo(width=640, height=480)
                    message.header.stamp = stamp
                    if spec['topic'] == '/camera/depth/image_raw' and 6 <= elapsed < 7:
                        message.header.stamp.sec -= 1  # Fresh delivery, deliberately stale source stamp.
                    pubs[spec['topic']].publish(message)
            if elapsed >= next_event:
                next_event = elapsed + .5; revision += 1
                stage = ('VALIDATING' if elapsed < 2 else 'APPROACHING' if elapsed < 4 else
                         'GRASPING' if elapsed < 6 else 'LIFTING' if elapsed < 8 else 'RETURNING_ARM' if elapsed < 10 else 'FINALIZING')
                record = ManipulationExecutionRecord(execution_id='synthetic-io-only', execution_profile='synthetic',
                    record_state='FINISHED' if elapsed >= 11 else 'ACTIVE', stage=stage, substage=stage,
                    selected_arm='left', revision=revision, accepted_at_ns=accepted_ns,
                    updated_at_ns=time.time_ns(), has_result=elapsed >= 11,
                    status='SUCCESS' if elapsed >= 11 else '', placement_state='ASSUMED' if elapsed >= 11 else '')
                event_pub.publish(record); bt.publish(String(data=json.dumps(dict(execution_id=record.execution_id, stage=stage))))
                for arm, pub in state_pubs.items(): pub.publish(Bool(data=left_on if arm == 'left' else False))
            executor.spin_once(timeout_sec=.002)
        published = probe.get_publisher_names_and_types_by_node(probe.get_name(), probe.get_namespace())
        assert all(name == '/parameter_events' for name, _ in published), published
        assert not probe.get_client_names_and_types_by_node(probe.get_name(), probe.get_namespace())
        manifest['finished'] = True
    finally:
        manifest.update(probe.finish()); manifest['udp_after'] = udp_counters()
        json_file(options.output/'manifest.json', manifest)
        executor.shutdown(); probe.destroy_node(); source.destroy_node(); rclpy.try_shutdown()
    _, rows = read_recording(options.output)
    arrivals = rows['arrival']
    assert any(r.get('feedback_max_age_ms', 0) and r['feedback_max_age_ms'] > 1000 for r in arrivals)
    assert any(r['topic'] == '/camera/depth/image_raw' and r['stamp_age_ms'] > 900 for r in arrivals)
    assert any(r['topic'] == '/camera/color/image_raw' and (r['active_gap_ms'] or 0) > 450 for r in arrivals)
    assert any(r.get('feedback_changed') is False for r in arrivals)
    assert any(s['topic'] == '/left_wrist_camera/image_raw' and s['status'] == 'OFF' for r in rows['window'] for s in r['streams'])
    assert any(r['stage'] == 'GRASPING' for r in rows['window'])
    assert manifest['writer']['complete']
    summary = render(options.output)
    assert summary['status'] == 'SYNTHETIC_TEST'
    assert summary['terminal_results']['synthetic-io-only']['placement_state'] == 'ASSUMED'
    json_file(options.output/'smoke-checks.json', dict(
        synthetic=True, commands_sent=False, real_hardware_used=False, passed=True,
        injected_image_gap_detected=True, stale_source_stamp_detected=True,
        repeated_motor_feedback_detected=True, intentional_camera_off_detected=True,
        action_stages_and_terminal_retained=True, writer_loss=manifest['writer']['dropped']))


if __name__ == '__main__': main()
