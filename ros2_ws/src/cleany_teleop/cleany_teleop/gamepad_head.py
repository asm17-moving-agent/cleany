"""Tilt the head camera using vertical right-stick input."""

import json
import math
import os
import signal
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState, Joy

from cleany_teleop.gamepad import CONDITIONED_AXIS_COUNT, StickSnapbackFilter, physical_axes
from cleany_teleop.gamepad_input import FRAME


MOTORS = (13, 14)
MAX_AGE = 0.15
TICK = 0.05
# Floor the speed limit to whole ticks/s for both target generation and servo RAM.
SPEED = 30 * 4096 // 360
RAM_SETTINGS = ((41, 1, 5), (44, 2, 0), (46, 2, SPEED), (48, 2, 300))


class HeadStick:
    """Hold position across receiver loss and resume from a fresh right-stick report."""

    def __init__(self):
        self.stamp = -1
        self.filter = StickSnapbackFilter()
        self.armed = False
        self.clear_input(-math.inf)

    def clear_input(self, cutoff):
        self.filter.reset()
        self.resume_after = cutoff
        self.expires = 0.0
        self.axes = (0.0, 0.0)

    def current(self, now):
        if self.expires and now >= self.expires:
            self.clear_input(self.expires)
        return self.axes

    def sample(self, axes, stamp, age, now):
        self.current(now)
        pair = axes[3:5]
        raw = physical_axes(axes)
        raw_pair = raw[3:5]
        if (len(raw_pair) != 2 or not all(math.isfinite(v) and abs(v) <= 1
                                          for v in (*pair, *raw_pair))
                or not math.isfinite(age) or age < 0):
            self.clear_input(now)
            return
        if age >= MAX_AGE or stamp <= self.stamp or now - age <= self.resume_after:
            return
        self.stamp, self.expires = stamp, now + MAX_AGE - age
        tilt = pair[1] if abs(pair[1]) > 0.1 else 0.0
        if abs(raw_pair[1]) <= 0.1:
            self.armed = True
        if self.armed:
            self.axes = (0.0, tilt)
            if len(axes) != CONDITIONED_AXIS_COUNT:
                self.axes = self.filter.apply(self.axes, stamp / 1e9)


class HeadServos:
    """Start at measured positions and respect EEPROM limits and bounded target lead."""

    def __init__(self, bus, mapping):
        self.bus = bus
        self.bounds = {}
        self.goals = {}
        self.owned = set()
        self.saved = {}
        self.moving = set()
        self.faulted = False
        self.healthy_since = None
        directions = mapping['direction_verification']
        if not (directions['pan_left_raw_decreases'] and directions['tilt_up_raw_decreases']):
            raise ValueError('헤드 방향 보정값을 확인하세요')
        for motor in MOTORS:
            lo, hi = mapping['motor_ranges_ticks'][str(motor)]
            if not (0 <= lo < hi <= 4095 and
                    (lo, hi) == (bus.read(motor, 9, 2), bus.read(motor, 11, 2))):
                raise ValueError(f'ID {motor}: 헤드 보정 범위 불일치')
            self.bounds[motor] = (lo, hi)
            if (bus.read(motor, 3, 2) != 777 or bus.read(motor, 33, 1) != 0
                    or bus.read(motor, 40, 1) != 0):
                raise ValueError(f'ID {motor}: 서보 모델, 위치 모드 또는 토크 소유권 확인 필요')
            self.position(motor)

    def position(self, motor):
        pos = self.bus.read(motor, 56, 2)
        lo, hi = self.bounds[motor]
        status = self.bus.read(motor, 65, 1)
        temperature = self.bus.read(motor, 63, 1)
        if not lo <= pos <= hi or status or temperature >= 50:
            raise ServoHealthFault(
                f'ID {motor}: 위치 {pos} tick (범위 {lo}..{hi}), '
                f'상태 0x{status:02x}, 온도 {temperature} °C'
            )
        return pos

    def acquire(self, motor):
        bus = self.bus
        if bus.read(motor, 40, 1) != 0:
            raise RuntimeError(f'ID {motor}: 다른 토크 소유자')
        self.saved[motor] = [(a, n, bus.read(motor, a, n)) for a, n, _ in RAM_SETTINGS]
        for addr, size, value in RAM_SETTINGS:
            bus.write(motor, addr, size, value)
            if bus.read(motor, addr, size) != value:
                raise RuntimeError(f'ID {motor}: RAM 설정 확인 실패')
        pos = self.position(motor)
        bus.write(motor, 42, 2, pos)
        if bus.read(motor, 42, 2) != pos or abs(self.position(motor) - pos) > 3:
            raise RuntimeError(f'ID {motor}: 현재 위치 유지 목표 확인 실패')
        # Release torque on shutdown even if a successful write lost its reply.
        self.owned.add(motor)
        bus.write(motor, 40, 1, 1)
        if bus.read(motor, 40, 1) != 1:
            raise RuntimeError(f'ID {motor}: 토크 활성화 확인 실패')
        self.goals[motor] = float(pos)
        return pos

    def step(self, axes, dt, expires, clock=time.monotonic):
        positions = []
        for motor, axis in zip(MOTORS, axes):
            pos = self.position(motor)
            positions.append(pos)
            if motor not in self.owned:
                if axis and clock() < expires:
                    pos = self.acquire(motor)
                    positions[-1] = pos
                else:
                    continue
            if clock() >= expires:
                axis = 0.0
            if not axis:
                if motor in self.moving:
                    self.bus.write(motor, 42, 2, pos)
                    self.moving.remove(motor)
                self.goals[motor] = float(pos)
                continue
            lo, hi = self.bounds[motor]
            # Positive tilt moves up, which decreases the raw servo position.
            # Limit target lead to one input lifetime even if the servo lags.
            lead = SPEED * MAX_AGE
            goal = self.goals[motor] - axis * SPEED * min(max(dt, 0.0), TICK)
            goal = min(hi, pos + lead, max(lo, pos - lead, goal))
            if round(goal) != round(self.goals[motor]):
                self.bus.write(motor, 42, 2, round(goal))
            self.goals[motor] = goal
            self.moving.add(motor)
        return positions

    def update(self, stick, dt, log, clock=time.monotonic):
        now = clock()
        try:
            if not self.faulted:
                return self.step(stick.current(now), dt, stick.expires, clock)
            positions = [self.position(motor) for motor in MOTORS]
        except ServoHealthFault as exc:
            self.healthy_since = None
            if not self.faulted:
                log.error(f'헤드 조향 중지. {exc}')
                self.close()
                self.faulted = True
            return None

        if self.healthy_since is None:
            self.healthy_since = now
        elif now - self.healthy_since >= 1.0:
            stick.armed = False
            stick.clear_input(now)
            self.faulted = False
            log.info('두 서보 정상. RS 중립 입력 대기')
        return positions

    def close(self):
        errors = []
        for motor, settings in self.saved.items():
            try:
                if motor in self.owned:
                    self.bus.write(motor, 40, 1, 0)
                if self.bus.read(motor, 40, 1) != 0:
                    raise RuntimeError('토크 해제 확인 실패')
                for addr, size, value in settings:
                    self.bus.write(motor, addr, size, value)
            except (OSError, RuntimeError) as exc:
                errors.append(f'ID {motor}: {exc}')
        if errors:
            raise RuntimeError(', '.join(errors))
        self.saved.clear()
        self.owned.clear()
        self.goals.clear()
        self.moving.clear()


class ServoHealthFault(RuntimeError):
    """Position, hardware status or temperature prevents head movement."""


def main(args=None):
    from cleany_robot_interface.arm_torque import connect

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('gamepad_head')
    stopping = False

    def request_stop(signum, frame):
        # An SDK error can leave the port busy and prevent torque release.
        nonlocal stopping
        stopping = True

    previous_handlers = {sig: signal.signal(sig, request_stop)
                         for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        root = Path(os.getenv('CLEANY_FIELD_REPO', '/opt/cleany/current/field/repo'))
        profiles = Path(node.declare_parameter('profile_dir', str(root / 'configs/robot')).value)
        head = json.loads((profiles / 'head_board_view.json').read_text())
        mapping = json.loads((profiles / 'head_preview_joints.json').read_text())
        if head['motor_ids'] != {'pan': 13, 'tilt': 14}:
            raise ValueError('헤드 서보 ID 불일치')
        stick = HeadStick()
        zeros = mapping['reference_ticks']
        signs = mapping['encoder_directions']
        scale = mapping['encoder_scale_rad_per_tick']
        feedback = node.create_publisher(JointState, 'head/joint_states', 1)

        def sample(msg):
            if msg.header.frame_id == FRAME:
                stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                stick.sample(msg.axes, stamp, (time.time_ns() - stamp) / 1e9, time.monotonic())

        with connect(head['serial_port'], head['baudrate']) as bus:
            servos = HeadServos(bus, mapping)
            previous = time.monotonic()

            def tick():
                nonlocal previous
                now = time.monotonic()
                stamp = node.get_clock().now().to_msg()
                positions = servos.update(stick, now - previous, node.get_logger())
                previous = now
                if positions is None:
                    return
                msg = JointState()
                msg.header.stamp = stamp
                msg.name = ['head_pan_joint', 'head_tilt_joint']
                msg.position = [(p - z) * sign * scale for p, z, sign in zip(positions, zeros, signs)]
                feedback.publish(msg)

            try:
                joy_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
                node.create_subscription(Joy, 'joy', sample, joy_qos)
                node.create_timer(TICK, tick)
                node.get_logger().info(
                    f'RS 상하 헤드 틸트 준비 (ID 14), '
                    f'최대 {math.degrees(SPEED * scale):.2f}°/s ({SPEED} tick/s)'
                )
                while rclpy.ok() and not stopping:
                    rclpy.spin_once(node, timeout_sec=TICK)
            finally:
                servos.close()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
