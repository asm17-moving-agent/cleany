"""Device-free peer for the fixed-size MCU permission contract."""
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
    def __init__(self):
        super().__init__('mock_base_mcu')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(WheelState, 'base/wheel_state', qos)
        self.create_subscription(WheelCommand, 'base/wheel_command', self.command, qos)
        self.create_service(Trigger, 'mock/reboot', self.reboot)
        self.create_service(SetBool, 'mock/transport', self.transport)
        self.create_service(Trigger, 'mock/drop_next_enable', self.drop_enable)
        self.create_service(Trigger, 'mock/force_disarm', self.force_disarm)
        self.boot, self.sequence, self.last_accepted_seq = 1234, 0, 0
        self.have_command_sequence = False
        self.enabled, self.target, self.valid_until_us = False, [0.] * 4, 0
        self.last_command = None
        self.faults, self.connected, self.drop_enable_count = 0, True, 0
        self.encoder_counts, self.encoder_integers = [0.] * 4, [0] * 4
        self.last_update = time.monotonic()
        self.create_timer(.02, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    @staticmethod
    def _deadline_ok(command, now_us):
        return now_us < command.valid_until_us <= now_us + 250000

    def command(self, command):
        now = time.monotonic()
        now_us = int(now * 1e6)
        self._expire(now_us)
        if command.mode == WheelCommand.STOP:
            if not self.have_command_sequence or newer(command.sequence, self.last_accepted_seq):
                self.last_accepted_seq = command.sequence
                self.have_command_sequence = True
            self.enabled, self.target, self.last_command, self.valid_until_us = False, [0.] * 4, None, 0
            return
        if not self.connected or command.protocol_version != 2:
            return
        if command.mode == WheelCommand.ENABLE and self.drop_enable_count:
            self.drop_enable_count -= 1
            return
        if self.have_command_sequence and not newer(command.sequence, self.last_accepted_seq):
            return
        values = tuple(float(v) for v in command.velocity_rad_s)
        if not self._deadline_ok(command, now_us) or not all(math.isfinite(v) and abs(v) <= 10 for v in values):
            return
        if command.mode == WheelCommand.ENABLE:
            if self.enabled or any(values):
                return
            self.enabled, self.target = True, [0.] * 4
            self.last_command, self.valid_until_us = now, command.valid_until_us
            self.last_accepted_seq = command.sequence
            self.have_command_sequence = True
            self.faults = 0
            return
        if command.mode != WheelCommand.VELOCITY or not self.enabled:
            return
        self.target = list(values)
        self.last_command, self.valid_until_us = now, command.valid_until_us
        self.last_accepted_seq = command.sequence
        self.have_command_sequence = True

    def _expire(self, now_us):
        if self.enabled and self.valid_until_us and now_us >= self.valid_until_us:
            self.enabled, self.target, self.last_command, self.valid_until_us = False, [0.] * 4, None, 0
            self.faults |= WheelState.FAULT_COMMAND_TIMEOUT

    def publish(self):
        now = time.monotonic()
        dt = min(.1, max(0., now - self.last_update))
        self.last_update = now
        self._expire(int(now * 1e6))
        if self.enabled:
            for i, speed in enumerate(self.target):
                self.encoder_counts[i] += speed * dt * 3172 / (2 * math.pi)
                wrapped = int(round(self.encoder_counts[i])) & 0xffffffff
                self.encoder_integers[i] = wrapped - 0x100000000 if wrapped >= 0x80000000 else wrapped
        if not self.connected:
            return
        msg = WheelState()
        msg.protocol_version, msg.boot_id = 2, self.boot
        msg.sequence, msg.last_command_sequence = self.sequence, self.last_accepted_seq
        self.sequence = (self.sequence + 1) & 0xffffffff
        msg.timestamp_us = int(now * 1e6)
        msg.counts_per_revolution, msg.max_velocity_rad_s, msg.watchdog_ms = 3172, 10., 250
        msg.enabled, msg.fault_bits = self.enabled, self.faults
        msg.control_mode = WheelState.VELOCITY if self.enabled else (
            WheelState.TIMEOUT_STOP if self.faults else WheelState.STOPPED)
        msg.command_age_ms = 65535 if self.last_command is None else min(65534, int((now-self.last_command)*1000))
        msg.encoder_counts = list(self.encoder_integers)
        msg.velocity_rad_s = msg.target_rad_s = msg.commanded_rad_s = list(self.target)
        msg.pwm_percent = [0] * 4
        self.pub.publish(msg)

    def reboot(self, request, response):
        self.boot = (self.boot + 1) & 0xffffffff or 1
        self.sequence, self.last_accepted_seq = 0, 0
        self.have_command_sequence = False
        self.enabled, self.target, self.last_command, self.faults = False, [0.] * 4, None, 0
        self.valid_until_us = 0
        self.encoder_counts, self.encoder_integers = [0.] * 4, [0] * 4
        response.success, response.message = True, 'mock MCU rebooted'
        return response

    def transport(self, request, response):
        self.connected = request.data
        if not request.data:
            self.enabled, self.target, self.last_command, self.valid_until_us = False, [0.] * 4, None, 0
        response.success, response.message = True, 'feedback connected' if request.data else 'feedback disconnected'
        return response

    def drop_enable(self, request, response):
        self.drop_enable_count += 1
        response.success, response.message = True, 'next ENABLE will be dropped'
        return response

    def force_disarm(self, request, response):
        self.enabled, self.target, self.last_command, self.valid_until_us = False, [0.] * 4, None, 0
        response.success, response.message = True, 'mock MCU disabled'
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
