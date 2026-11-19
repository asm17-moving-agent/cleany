#!/usr/bin/env python3
"""Publish encoder-driven CAD TF; optionally run a bounded head-only wall probe."""

import argparse
import json
import math
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
import serial
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from read_head_servos import read_register


def angles(raw: list[int], config: dict) -> list[float]:
    period = config['encoder_period']
    return [((value-zero+period/2) % period-period/2)*direction*2*math.pi/period
            for value, zero, direction in zip(raw, config['reference_ticks'], config['encoder_directions'])]


def write_ram(port, motor: int, address: int, size: int, value: int) -> None:
    limits = {40: (1, 1), 41: (1, 5), 42: (2, 4095), 44: (2, 0), 46: (2, 80), 48: (2, 300)}
    if motor not in (13, 14) or address not in limits or size != limits[address][0] or not 0 <= value <= limits[address][1]:
        raise ValueError('Write outside bounded head RAM contract')
    data = value.to_bytes(size, 'little')
    payload = bytes([motor, size+3, 3, address])+data
    port.reset_input_buffer()
    port.write(b'\xff\xff'+payload+bytes([~sum(payload) & 255]))
    port.flush()
    time.sleep(.005)
    if read_register(port, motor, address, size) != value:
        raise RuntimeError(f'ID {motor}: RAM write verification failed')


class HeadProbe:
    """Read actual encoders and publish fresh dynamic TF without guessed hold values."""

    def __init__(self, node, port, config: dict, log):
        self.node, self.port, self.config, self.log = node, port, config, log
        self.tf = TransformBroadcaster(node)
        self.joints = node.create_publisher(JointState, '/head/joint_states', 10)
        self.last_feedback = 0.0

    def poll(self) -> list[int]:
        start = time.monotonic()
        raw = [read_register(self.port, motor, 56, 2) for motor in self.config['ids']]
        if any(not 0 <= value <= 4095 for value in raw) or time.monotonic()-start > .1:
            raise RuntimeError('Invalid or delayed head feedback')
        pan, tilt = angles(raw, self.config)
        stamp = self.node.get_clock().now().to_msg()
        entries = [
            (self.config['parent'], self.config['pan_frame'], self.config['pan_pivot_m'], [0, 0, math.sin(pan/2), math.cos(pan/2)]),
            (self.config['pan_frame'], self.config['tilt_frame'], self.config['tilt_from_pan_m'], [0, math.sin(tilt/2), 0, math.cos(tilt/2)]),
            (self.config['tilt_frame'], self.config['camera_frame'], self.config['camera_from_tilt_m'], [0, 0, 0, 1]),
        ]
        transforms = []
        for parent, child, xyz, xyzw in entries:
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, xyzw)
            transforms.append(t)
        self.tf.sendTransform(transforms)
        msg = JointState()
        msg.header.stamp, msg.name, msg.position = stamp, ['head_pan_joint', 'head_tilt_joint'], [pan, tilt]
        self.joints.publish(msg)
        if self.log and time.monotonic()-self.last_feedback > .1:
            self.log.write(json.dumps({'stamp': stamp.sec+stamp.nanosec*1e-9, 'raw': raw,
                                       'angles_rad': [pan, tilt]})+'\n')
            self.log.flush()
            self.last_feedback = time.monotonic()
        rclpy.spin_once(self.node, timeout_sec=0)
        return raw

    def hold_for(self, seconds: float) -> None:
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            self.poll()
            time.sleep(1/self.config['poll_hz'])

    def move(self, target: list[int], origin: list[int]) -> None:
        if any(abs(t-o) > 60 for t, o in zip(target, origin)):
            raise ValueError('Exercise may not exceed 60 ticks from original pose')
        for motor, goal in zip(self.config['ids'], target):
            write_ram(self.port, motor, 42, 2, goal)
        deadline, stable = time.monotonic()+4, 0
        while time.monotonic() < deadline:
            raw = self.poll()
            if any(abs(r-o) > 75 for r, o in zip(raw, origin)):
                raise RuntimeError('Head left bounded exercise region')
            stable = stable+1 if all(abs(a-b) <= self.config['settle_tolerance_ticks'] for a, b in zip(raw, target)) else 0
            if stable >= 10:
                return
            time.sleep(.05)
        raise RuntimeError('Head failed to settle')

    def exercise(self, output: Path, selected: list[str] | None = None) -> None:
        saved = {}
        for motor in self.config['ids']:
            state = {name: read_register(self.port, motor, *loc) for name, loc in
                     {'model': (3, 2), 'mode': (33, 1), 'torque': (40, 1), 'position': (56, 2),
                      'temperature': (63, 1), 'fault': (65, 1)}.items()}
            if state['model'] != 777 or state['mode'] != 0 or state['torque'] != 0 or state['fault'] or state['temperature'] >= self.config['exercise_max_temperature_c']:
                raise RuntimeError(f'Head preflight rejected ID {motor}: {state}')
            saved[motor] = state
        origin = [saved[m]['position'] for m in self.config['ids']]
        if not (1900 <= origin[0] <= 2100 and 2000 <= origin[1] <= 2250):
            raise RuntimeError('Exercise requires the verified near-neutral starting region')
        (output/'before.json').write_text(json.dumps(saved, indent=2))
        try:
            for motor, raw in zip(self.config['ids'], origin):
                for address, size, value in [(41, 1, self.config['exercise_acceleration']), (44, 2, 0),
                                             (46, 2, self.config['exercise_speed_ticks_s']),
                                             (48, 2, self.config['exercise_torque_limit']), (42, 2, raw)]:
                    write_ram(self.port, motor, address, size, value)
            if any(abs(a-b) > 3 for a, b in zip(self.poll(), origin)):
                raise RuntimeError('Head moved before torque enable')
            for motor in self.config['ids']:
                write_ram(self.port, motor, 40, 1, 1)
            step = round(self.config['exercise_degrees']/360*self.config['encoder_period'])
            poses = [('start', [0, 0]), ('pan_plus', [-step, 0]), ('pan_minus', [step, 0]),
                     ('tilt_plus', [0, step]), ('tilt_minus', [0, -step]), ('return', [0, 0])]
            if selected:
                poses = [pose for pose in poses if pose[0] in selected]
            for name, delta in poses:
                for motor in self.config['ids']:
                    if read_register(self.port, motor, 65, 1) or read_register(self.port, motor, 63, 1) >= self.config['exercise_max_temperature_c']:
                        raise RuntimeError(f'ID {motor}: fault or temperature during exercise')
                self.move([a+b for a, b in zip(origin, delta)], origin)
                self.hold_for(.5)
                print(f'Capturing {name}', flush=True)
                command = [sys.executable, str(Path(__file__).with_name('capture_depth_lidar.py')),
                           '--output', str(output/name), '--seconds', '4']
                child = subprocess.Popen(command)
                try:
                    while child.poll() is None:
                        self.hold_for(.1)
                    if child.returncode:
                        raise RuntimeError(f'Capture failed: {name}')
                finally:
                    if child.poll() is None:
                        child.terminate()
                        child.wait(timeout=5)
                self.move(origin, origin)
        finally:
            # Preserve original torque-off state. Never blindly drive home on a fault.
            for motor in self.config['ids']:
                try:
                    write_ram(self.port, motor, 40, 1, 0)
                except Exception as exc:
                    print(f'ID {motor}: TORQUE-OFF NOT CONFIRMED: {exc}', file=sys.stderr)
            (output/'after.json').write_text(json.dumps({m: {'position': read_register(self.port, m, 56, 2),
                                                            'torque': read_register(self.port, m, 40, 1)} for m in self.config['ids']}, indent=2))

    def floor_view(self, degrees: float, output: Path) -> None:
        """Aim tilt downward in bounded steps; restore its initial torque-off state."""
        if not 15 <= degrees <= 30:
            raise ValueError('Floor view requires 15 to 30 degrees down')
        raw = self.poll()
        if not 1900 <= raw[0] <= 2100 or not 1950 <= raw[1] <= 2420:
            raise RuntimeError('Floor view requires the verified near-neutral head region')
        for motor in self.config['ids']:
            for address, size, expected in [(3, 2, 777), (33, 1, 0), (40, 1, 0), (65, 1, 0)]:
                if read_register(self.port, motor, address, size) != expected:
                    raise RuntimeError(f'Floor-view preflight failed: {motor}/{address}')
            if read_register(self.port, motor, 63, 1) >= self.config['exercise_max_temperature_c']:
                raise RuntimeError('Head temperature too high')
        target = self.config['reference_ticks'][1] + round(degrees/360*self.config['encoder_period'])
        (output/'before.json').write_text(json.dumps({'raw': raw, 'target_tilt_tick': target}))
        try:
            for address, size, value in [(41, 1, 5), (44, 2, 0), (46, 2, 80), (48, 2, 300), (42, 2, raw[1])]:
                write_ram(self.port, 14, address, size, value)
            if abs(self.poll()[1]-raw[1]) > 3:
                raise RuntimeError('Head moved before tilt torque enable')
            write_ram(self.port, 14, 40, 1, 1)
            while abs(raw[1]-target) > 3:
                next_raw = [raw[0], raw[1] + max(-57, min(57, target-raw[1]))]
                self.move(next_raw, raw)
                raw = self.poll()
            self.hold_for(.5)
        finally:
            write_ram(self.port, 14, 40, 1, 0)
            self.hold_for(.5)
            (output/'after.json').write_text(json.dumps({'raw': self.poll(), 'tilt_torque': read_register(self.port, 14, 40, 1)}))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--exercise', action='store_true')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--poses', nargs='+', choices=['start', 'pan_plus', 'pan_minus', 'tilt_plus', 'tilt_minus', 'return'])
    parser.add_argument('--floor-view-deg', type=float)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    if args.exercise and args.output is None:
        parser.error('--exercise requires a new --output directory')
    if args.floor_view_deg is not None and (args.exercise or args.output is None):
        parser.error('--floor-view-deg requires --output and excludes --exercise')
    if args.output:
        args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('d435_head_encoder_tf')
    log = (args.output/'encoders.jsonl').open('w') if args.output else None
    try:
        with serial.Serial(config['serial_port'], config['baudrate'], timeout=.02, exclusive=True) as port:
            probe = HeadProbe(node, port, config, log)
            if args.floor_view_deg is not None:
                probe.floor_view(args.floor_view_deg, args.output)
            elif args.exercise:
                probe.exercise(args.output, args.poses)
            else:
                while rclpy.ok():
                    probe.hold_for(.1)
    finally:
        if log:
            log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
