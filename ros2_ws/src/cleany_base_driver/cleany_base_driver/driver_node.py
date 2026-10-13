"""ROS adapter for the fixed-size micro-ROS wheel transport."""
from __future__ import annotations

import time
from math import isfinite

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from cleany_base_interfaces.msg import WheelCommand, WheelState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from .core import EncoderAdapter, Geometry, Limits, finite_twist_axes, newer, wheel_speeds


class BaseDriver(Node):
    def __init__(self, parameter_overrides=None) -> None:
        super().__init__('cleany_base_driver', parameter_overrides=parameter_overrides or [])
        self.declare_parameter('mock', False)
        for key, default in (('geometry.wheel_radius_m', 0.), ('geometry.wheelbase_m', 0.),
                             ('geometry.wheel_separation_m', 0.)):
            self.declare_parameter(key, default)
        for key in ('linear_x_mps', 'linear_y_mps', 'angular_z_rad_s', 'wheel_rad_s', 'command_timeout_s'):
            self.declare_parameter('limits.' + key, 0.)
        self.declare_parameter('counts_per_revolution', 3172)
        self.geometry = Geometry(*(float(self.get_parameter('geometry.' + p).value)
                                   for p in ('wheel_radius_m', 'wheelbase_m', 'wheel_separation_m')))
        self.limits = Limits(*(float(self.get_parameter('limits.' + p).value)
                               for p in ('linear_x_mps', 'linear_y_mps', 'angular_z_rad_s',
                                         'wheel_rad_s', 'command_timeout_s')))
        self.encoder = EncoderAdapter(int(self.get_parameter('counts_per_revolution').value))
        self.command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                       durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(WheelCommand, 'base/wheel_command', self.command_qos)
        self.joints = self.create_publisher(JointState, 'joint_states', 10)
        self.diags = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.create_subscription(WheelState, 'base/wheel_state', self.on_state, self.command_qos)
        self.create_subscription(Twist, 'cmd_vel', self.on_twist, 10)
        self.create_service(SetBool, 'base/enable', self.on_enable)
        self.enabled = False
        self.latest = self.latest_receipt = self.last_state_receipt = None
        self.last_cmd_receipt = self.last_state_sequence = self.last_mcu_timestamp = None
        self.latest_twist = None
        self.boot_id = None
        self.command_sequence = None
        self.pending_enable = None
        self.diagnostic_error = ''
        self._stop_sent = False
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(.1, self.publish_diagnostics, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _deadline(self) -> int:
        return self._mcu_now() + min(200000, int(self.limits.timeout_s * 1e6))

    def _mcu_now(self) -> int:
        return self.latest.timestamp_us + max(0, int((time.monotonic() - self.latest_receipt) * 1e6))

    def _send(self, mode: int, values=(0., 0., 0., 0.), sequence=None, deadline=0) -> WheelCommand:
        msg = WheelCommand()
        msg.protocol_version = 2
        if sequence is None:
            self.command_sequence = ((self.command_sequence or 0) + 1) & 0xffffffff
            sequence = self.command_sequence
        msg.sequence, msg.mode = sequence, mode
        msg.velocity_rad_s = list(values)
        msg.valid_until_us = deadline
        self.publisher.publish(msg)
        return msg

    def _disable(self, reason: str, send_stop=True) -> None:
        self.enabled = False
        self.pending_enable = None
        self.last_cmd_receipt = None
        self.latest_twist = None
        self.diagnostic_error = reason
        if send_stop and not self._stop_sent:
            self._send(WheelCommand.STOP)
            self._stop_sent = True

    def on_enable(self, request, response):
        if not request.data:
            self._disable('stopped by request')
            response.success, response.message = True, 'STOP sent'
            return response
        if self.enabled:
            response.success, response.message = True, 'already enabled'
            return response
        if self.latest is None or self.last_state_receipt is None or time.monotonic() - self.last_state_receipt > .25:
            response.success, response.message = False, 'no fresh wheel feedback'
            return response
        if self.command_sequence is None:
            self.command_sequence = self.latest.last_command_sequence
        elif newer(self.latest.last_command_sequence, self.command_sequence):
            self.command_sequence = self.latest.last_command_sequence
        self.command_sequence = (self.command_sequence + 1) & 0xffffffff
        deadline = self._deadline()
        self.pending_enable = (self.command_sequence, deadline)
        self.enabled = True
        self._stop_sent = False
        self.last_cmd_receipt = None
        self.latest_twist = None
        self._send(WheelCommand.ENABLE, sequence=self.command_sequence, deadline=deadline)
        response.success, response.message = True, 'ENABLE sent; awaiting MCU acknowledgement'
        return response

    def on_twist(self, msg: Twist) -> None:
        axes = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
        if not finite_twist_axes(*axes):
            if self.enabled:
                self._disable('non-finite cmd_vel')
            return
        if not self.enabled:
            return
        self.last_cmd_receipt = time.monotonic()
        self.latest_twist = (msg.linear.x, msg.linear.y, msg.angular.z)
        if any(v != 0 for v in (msg.linear.z, msg.angular.x, msg.angular.y)):
            self.get_logger().warning('unsupported cmd_vel axes ignored', throttle_duration_sec=2)

    def on_state(self, msg: WheelState) -> None:
        now = time.monotonic()
        if msg.protocol_version != 2 or msg.boot_id == 0 or msg.counts_per_revolution != self.encoder.scale or \
                msg.max_velocity_rad_s != 10.0 or msg.watchdog_ms != 250:
            self._disable('WheelState contract mismatch')
            return
        if self.boot_id == msg.boot_id and self.last_state_sequence is not None and \
                not newer(msg.sequence, self.last_state_sequence):
            return
        if self.boot_id == msg.boot_id and self.last_mcu_timestamp is not None and \
                msg.timestamp_us <= self.last_mcu_timestamp:
            return
        reboot = self.boot_id is not None and self.boot_id != msg.boot_id
        if reboot:
            self.command_sequence = msg.last_command_sequence
            self._stop_sent = False
            self._disable('MCU reboot detected')
            self.last_state_sequence = self.last_mcu_timestamp = None
            self.encoder.rebase()
        if not all(isfinite(float(v)) for v in (*msg.velocity_rad_s, *msg.target_rad_s, *msg.commanded_rad_s)):
            self._disable('non-finite wheel feedback')
            return
        self.boot_id = msg.boot_id
        self.last_state_sequence = msg.sequence
        self.last_mcu_timestamp = msg.timestamp_us
        self.latest, self.latest_receipt = msg, now
        self.last_state_receipt = now
        if self.command_sequence is None or newer(msg.last_command_sequence, self.command_sequence):
            self.command_sequence = msg.last_command_sequence
        if self.pending_enable is not None:
            seq, _ = self.pending_enable
            if msg.last_command_sequence == seq and msg.enabled:
                self.pending_enable = None
                self.diagnostic_error = ''
        if self.enabled and not msg.enabled and self.pending_enable is None:
            self._disable('MCU unexpectedly disabled')
        elif not self.enabled and msg.enabled:
            self._disable('MCU enabled without host permission')
        if msg.fault_bits:
            if self.pending_enable is None:
                self._disable(f'MCU fault bits: {msg.fault_bits:#x}')
        positions = self.encoder.update(msg.boot_id, tuple(msg.encoder_counts))
        if positions is not None:
            out = JointState()
            out.header.stamp = self.get_clock().now().to_msg()
            out.name = ['front_left_wheel_joint', 'front_right_wheel_joint',
                        'rear_left_wheel_joint', 'rear_right_wheel_joint']
            out.position, out.velocity = list(positions), [float(v) for v in msg.velocity_rad_s]
            self.joints.publish(out)

    def tick(self) -> None:
        now = time.monotonic()
        if self.last_state_receipt is None or now - self.last_state_receipt > .25:
            if self.enabled:
                self._disable('stale wheel feedback')
            if self.latest is not None:
                self.latest = self.latest_receipt = self.last_state_receipt = None
                self.encoder.rebase()
            return
        if not self.enabled:
            return
        if self.last_cmd_receipt is not None and now - self.last_cmd_receipt > self.limits.timeout_s:
            self._disable('cmd_vel timeout')
            return
        if self.pending_enable is not None:
            seq, deadline = self.pending_enable
            if self._mcu_now() >= deadline:
                self._disable('ENABLE acknowledgement timeout')
                return
            self._send(WheelCommand.ENABLE, sequence=seq, deadline=deadline)
            return
        if not self.latest.enabled:
            return
        twist = self.latest_twist
        wheels = (0.,) * 4 if twist is None else wheel_speeds(*twist, self.geometry, self.limits)
        self._send(WheelCommand.VELOCITY, wheels, deadline=self._deadline())

    def publish_diagnostics(self) -> None:
        status = DiagnosticStatus()
        age = float('inf') if self.last_state_receipt is None else time.monotonic() - self.last_state_receipt
        status.name, status.hardware_id = 'base_driver', 'micro_ros_base'
        status.level = DiagnosticStatus.ERROR if age > .25 or self.diagnostic_error else DiagnosticStatus.OK
        status.message = self.diagnostic_error or ('stale wheel feedback' if age > .25 else 'feedback current')
        status.values = [KeyValue(key='enabled', value=str(self.enabled)), KeyValue(key='feedback_age_s', value=str(age))]
        arr = DiagnosticArray()
        arr.header.stamp, arr.status = self.get_clock().now().to_msg(), [status]
        self.diags.publish(arr)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = BaseDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._disable('driver shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
