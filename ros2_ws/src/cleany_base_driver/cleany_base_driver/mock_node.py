"""Faithful device-free peer for the fixed-size MCU gate contract."""
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from cleany_base_interfaces.msg import WheelCommand, WheelState
from std_srvs.srv import SetBool, Trigger

from .core import newer


class MockMcu(Node):
    """MCU gate with a deliberately simple perfect-velocity plant."""

    def __init__(self):
        super().__init__('mock_base_mcu')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(WheelState, 'base/wheel_state', qos)
        self.create_subscription(WheelCommand, 'base/wheel_command', self.command, qos)
        self.create_service(Trigger, 'mock/reboot', self.reboot)
        self.create_service(SetBool, 'mock/transport', self.transport)
        self.create_service(Trigger, 'mock/drop_next_begin', self.drop_begin)
        self.create_service(Trigger, 'mock/drop_next_arm', self.drop_arm)
        self.create_service(Trigger, 'mock/force_disarm', self.force_disarm)
        self.create_service(Trigger, 'mock/change_session', self.change_session)
        self.boot, self.session, self.sequence = 1234, 0, 0
        self.armed, self.target = False, [0.] * 4
        self.last_command = None
        self.last_seq = None
        self.last_accepted_seq = 0
        self.valid_until_us = 0
        self.retired_session = 0
        self.retired_sessions = {}
        self.session_holdoff_until_us = 0
        self.faults = 0
        self.connected = True
        self.drop_begin_count = self.drop_arm_count = 0
        self.encoder_counts = [0.] * 4
        self.encoder_integers = [0] * 4
        self.last_update = time.monotonic()
        self.create_timer(.02, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    @staticmethod
    def _deadline_ok(command: WheelCommand, now_us: int) -> bool:
        return now_us < command.valid_until_us <= now_us + 250000

    def command(self, command: WheelCommand) -> None:
        now = time.monotonic()
        now_us = int(now * 1e6)
        # STOP is unconditional and always retires the active session.
        if command.mode == WheelCommand.STOP:
            self._retire(watchdog=False, now=now)
            return
        if not self.connected:
            return
        if command.protocol_version != 1 or command.boot_id != self.boot:
            return
        if command.mode == WheelCommand.BEGIN_SESSION:
            if self.drop_begin_count:
                self.drop_begin_count -= 1
                return
            if (not command.session_id or any(command.velocity_rad_s) or
                    not self._deadline_ok(command, now_us)):
                return
            if (now_us < self.session_holdoff_until_us or
                    command.session_id in (self.session, self.retired_session) or
                    now_us < self.retired_sessions.get(command.session_id, 0)):
                return
            if not self._remember_session(now_us):
                return
            self.session = command.session_id
            self.last_seq = command.sequence
            self.last_accepted_seq = command.sequence
            self.armed, self.target = False, [0.] * 4
            self.last_command, self.faults, self.valid_until_us = None, 0, 0
            return
        if command.session_id != self.session or self.last_seq is None:
            return
        if not newer(command.sequence, self.last_seq):
            return
        if command.mode == WheelCommand.ARM:
            if self.drop_arm_count:
                self.drop_arm_count -= 1
                return
            if any(command.velocity_rad_s) or not self._deadline_ok(command, now_us):
                return
            self.armed, self.target = True, [0.] * 4
            self.last_seq, self.last_command = command.sequence, now
            self.last_accepted_seq, self.valid_until_us = command.sequence, command.valid_until_us
            return
        if command.mode != WheelCommand.VELOCITY or not self.armed:
            return
        values = tuple(float(v) for v in command.velocity_rad_s)
        if (not self._deadline_ok(command, now_us) or
                not all(math.isfinite(v) and abs(v) <= 10 for v in values)):
            return
        self.target = list(values)
        self.last_seq, self.last_command = command.sequence, now
        self.last_accepted_seq, self.valid_until_us = command.sequence, command.valid_until_us

    def _remember_session(self, now_us: int) -> bool:
        if not self.session:
            return True
        self.retired_session = self.session
        self.retired_sessions = {
            session: until for session, until in self.retired_sessions.items()
            if now_us < until
        }
        if self.session in self.retired_sessions or len(self.retired_sessions) < 8:
            self.retired_sessions[self.session] = now_us + 250000
            return True
        self.session_holdoff_until_us = now_us + 250000
        return False

    def _retire(self, *, watchdog: bool, now=None) -> None:
        self._remember_session(int((time.monotonic() if now is None else now) * 1e6))
        self.armed, self.target = False, [0.] * 4
        self.session, self.last_seq, self.last_command = 0, None, None
        self.valid_until_us = 0
        if watchdog:
            self.faults |= 1

    def publish(self) -> None:
        now = time.monotonic()
        dt = min(.1, max(0., now - self.last_update))
        self.last_update = now
        # Feedback transport loss does not suspend the controller watchdog.
        if (self.armed and self.last_command is not None and
                (now - self.last_command >= .25 or int(now * 1e6) >= self.valid_until_us)):
            self._retire(watchdog=True, now=now)
        if self.armed:
            for index, speed in enumerate(self.target):
                self.encoder_counts[index] += speed * dt * 3172 / (2 * math.pi)
                wrapped = int(round(self.encoder_counts[index])) & 0xffffffff
                self.encoder_integers[index] = (
                    wrapped - 0x100000000 if wrapped >= 0x80000000 else wrapped
                )
        if not self.connected:
            return
        message = WheelState()
        message.protocol_version, message.boot_id = 1, self.boot
        message.session_id, message.sequence = self.session, self.sequence
        message.last_command_sequence = self.last_accepted_seq
        message.timestamp_us = int(now * 1e6)
        self.sequence = (self.sequence + 1) & 0xffffffff
        message.counts_per_revolution = 3172
        message.max_velocity_rad_s, message.watchdog_ms = 10., 250
        message.armed, message.fault_bits = self.armed, self.faults
        message.control_mode = (
            WheelState.VELOCITY if self.armed else
            WheelState.WATCHDOG_STOP if self.faults else WheelState.STOPPED
        )
        message.command_age_ms = (
            65535 if self.last_command is None else
            min(65534, int((now - self.last_command) * 1000))
        )
        message.encoder_counts = list(self.encoder_integers)
        message.velocity_rad_s = [float(v) for v in self.target]
        message.target_rad_s = [float(v) for v in self.target]
        message.commanded_rad_s = [float(v) for v in self.target]
        message.pwm_percent = [0] * 4
        self.pub.publish(message)

    def reboot(self, request, response):
        self.boot = (self.boot + 1) & 0xffffffff or 1
        self.session, self.sequence, self.last_seq = 0, 0, None
        self.armed, self.target, self.faults = False, [0.] * 4, 0
        self.last_command = None
        self.valid_until_us = self.last_accepted_seq = self.retired_session = 0
        self.retired_sessions = {}
        self.session_holdoff_until_us = 0
        self.encoder_counts, self.encoder_integers = [0.] * 4, [0] * 4
        response.success, response.message = True, 'mock MCU rebooted'
        return response

    def transport(self, request, response):
        self.connected = request.data
        if not self.connected:
            self._retire(watchdog=False)
        response.success = True
        response.message = 'feedback connected' if request.data else 'feedback disconnected'
        return response

    def drop_begin(self, request, response):
        self.drop_begin_count += 1
        response.success, response.message = True, 'next BEGIN_SESSION will be dropped'
        return response

    def drop_arm(self, request, response):
        self.drop_arm_count += 1
        response.success, response.message = True, 'next ARM will be dropped'
        return response

    def force_disarm(self, request, response):
        self._retire(watchdog=False)
        response.success, response.message = True, 'mock MCU disarmed'
        return response

    def change_session(self, request, response):
        self.session = (self.session + 1) & 0xffffffff or 1
        self.armed, self.target = False, [0.] * 4
        self.last_seq, self.last_command = 0, None
        response.success, response.message = True, 'mock session changed unexpectedly'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockMcu()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
