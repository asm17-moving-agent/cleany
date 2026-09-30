"""ROS adapter for the fixed-size micro-ROS wheel transport."""
from __future__ import annotations

import secrets
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

from .core import (
    EncoderAdapter, Geometry, Limits, SessionGate, finite_twist_axes,
    wheel_speeds,
)


class BaseDriver(Node):
    def __init__(self, parameter_overrides=None) -> None:
        super().__init__('cleany_base_driver', parameter_overrides=parameter_overrides or [])
        self.declare_parameter('mock', False)
        self.declare_parameter('geometry.wheel_radius_m', 0.0)
        self.declare_parameter('geometry.wheelbase_m', 0.0)
        self.declare_parameter('geometry.wheel_separation_m', 0.0)
        for key in ('linear_x_mps', 'linear_y_mps', 'angular_z_rad_s',
                    'wheel_rad_s', 'command_timeout_s'):
            self.declare_parameter('limits.' + key, 0.0)
        self.declare_parameter('counts_per_revolution', 3172)
        mock = bool(self.get_parameter('mock').value)
        self.geometry = Geometry(*(float(self.get_parameter('geometry.' + p).value)
            for p in ('wheel_radius_m', 'wheelbase_m', 'wheel_separation_m')))
        self.limits = Limits(*(float(self.get_parameter('limits.' + p).value)
            for p in ('linear_x_mps', 'linear_y_mps', 'angular_z_rad_s',
                      'wheel_rad_s', 'command_timeout_s')))
        self.gate = SessionGate()
        self.encoder = EncoderAdapter(int(self.get_parameter('counts_per_revolution').value))
        self.command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                       durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(WheelCommand, 'base/wheel_command', self.command_qos)
        self.joints = self.create_publisher(JointState, 'joint_states', 10)
        self.diags = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.create_subscription(WheelState, 'base/wheel_state', self.on_state, self.command_qos)
        self.create_subscription(Twist, 'cmd_vel', self.on_twist, 10)
        self.create_service(SetBool, 'base/enable', self.on_enable)
        self.latest = None
        self.latest_receipt = None
        self.last_cmd_receipt = None
        self.last_state_receipt = None
        self.last_state_sequence = None
        self.last_mcu_timestamp = None
        self.diagnostic_error = ''
        self.command_sequence = 0
        self.session = 0
        self._started = False
        self.session_confirmed = False
        self.begin_command = None
        self._stop_sent = False
        self.arm_sent = False
        self.arm_started = None
        self.armed_since = None
        self.mock = mock
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(.1, self.publish_diagnostics, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _send(self, mode: int, values=(0., 0., 0., 0.)) -> None:
        msg = WheelCommand()
        msg.protocol_version = 1
        msg.boot_id = self.gate.boot or 0
        msg.session_id = self.session
        msg.sequence = self.command_sequence
        self.command_sequence = (self.command_sequence + 1) & 0xffffffff
        msg.mode = mode
        msg.velocity_rad_s = list(values)
        if mode:
            stamp, receipt = self.latest.timestamp_us, self.latest_receipt
            elapsed_us = max(0, int((time.monotonic() - receipt) * 1e6))
            msg.valid_until_us = stamp + elapsed_us + min(200000, int(self.limits.timeout_s * 1e6))
        self.publisher.publish(msg)

    def _begin(self) -> None:
        if self.latest is None or not self.last_state_receipt or \
                time.monotonic() - self.last_state_receipt > .25:
            return
        self.session = secrets.randbits(32) or 1
        self.gate.begin(self.latest.boot_id, self.session)
        self.command_sequence = 0
        self._send(WheelCommand.BEGIN_SESSION)
        self.begin_command = WheelCommand()
        self.begin_command.protocol_version = 1
        self.begin_command.boot_id = self.latest.boot_id
        self.begin_command.session_id = self.session
        self.begin_command.sequence = 0
        self.begin_command.mode = WheelCommand.BEGIN_SESSION
        self.begin_command.velocity_rad_s = [0.] * 4
        self.begin_command.valid_until_us = (
            self.latest.timestamp_us
            + max(0, int((time.monotonic() - self.latest_receipt) * 1e6))
            + min(200000, int(self.limits.timeout_s * 1e6))
        )
        self._started = True
        self.session_confirmed = False
        self._stop_sent = False

    def _invalidate(self, reason: str, *, send_stop: bool = True) -> None:
        """Drop all command authority and feedback that could authorize ARM."""
        self.diagnostic_error = reason
        if send_stop and not self._stop_sent:
            self._send(WheelCommand.STOP)
            self._stop_sent = True
        boot = self.gate.boot
        self.gate.reset(boot)
        self.session = 0
        self._started = False
        self.session_confirmed = False
        self.begin_command = None
        self.arm_sent = False
        self.arm_started = None
        self.armed_since = None
        self.last_cmd_receipt = None
        self.latest = self.latest_receipt = None
        self.last_state_receipt = None
        self.encoder.rebase()
        self.publish_diagnostics()

    def on_enable(self, request, response):
        if request.data:
            if (self.latest is None or self.last_state_receipt is None or
                    time.monotonic() - self.last_state_receipt > .25 or
                    not self.session_confirmed or
                    self.latest.session_id != self.session or
                    self.latest.fault_bits):
                response.success, response.message = False, 'no fresh matching disarmed session feedback'
                return response
            self.gate.request_enable()
            self.last_cmd_receipt = None
            self.arm_sent = False
            self.arm_started = time.monotonic()
            if self.latest.session_id == self.session:
                self._send(WheelCommand.ARM)
                self.arm_sent = True
            response.success, response.message = True, 'ARM sent; awaiting matching armed feedback'
        else:
            self._stop_session()
            response.success, response.message = True, 'STOP sent'
        return response

    def _stop_session(self) -> None:
        self._invalidate('stopped by request')

    def on_twist(self, msg: Twist) -> None:
        axes = (msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z)
        if not finite_twist_axes(*axes):
            if self.gate.enabled or self.gate.armed:
                self._invalidate('non-finite cmd_vel')
            return
        if any(v != 0 for v in (msg.linear.z, msg.angular.x, msg.angular.y)):
            self.get_logger().warning('unsupported cmd_vel axes ignored', throttle_duration_sec=2)
        # Never retain input received before matching armed feedback. The
        # first command accepted here must therefore be a post-ARM cmd_vel.
        if not self.gate.enabled or not self.gate.armed:
            return
        self.last_cmd_receipt = time.monotonic()
        wheels = wheel_speeds(msg.linear.x, msg.linear.y, msg.angular.z, self.geometry, self.limits)
        self.gate.command(wheels)

    def on_state(self, msg: WheelState) -> None:
        now = time.monotonic()
        if msg.protocol_version != 1 or msg.boot_id == 0 or \
                msg.counts_per_revolution != self.encoder.scale or \
                msg.max_velocity_rad_s != 10.0 or msg.watchdog_ms != 250:
            self.get_logger().error('WheelState contract mismatch; disarming')
            self._invalidate('WheelState contract mismatch')
            return
        if self.gate.boot == msg.boot_id and self.last_mcu_timestamp is not None \
                and msg.timestamp_us <= self.last_mcu_timestamp:
            self._invalidate('non-increasing MCU timestamp')
            return
        float_fields = (*msg.velocity_rad_s, *msg.target_rad_s, *msg.commanded_rad_s)
        if not all(isfinite(float(v)) for v in float_fields):
            self._invalidate('non-finite wheel feedback')
            return
        if self.gate.boot is not None and msg.boot_id != self.gate.boot:
            self._invalidate('MCU reboot detected')
            self.gate.reset(msg.boot_id)
            self.last_state_sequence = None
            self.last_mcu_timestamp = None
            self.encoder.rebase()
        elif self.last_state_sequence is not None:
            delta = (msg.sequence - self.last_state_sequence) & 0xffffffff
            if delta == 0 or delta >= 0x80000000:
                return
            if delta > 4:
                self.last_state_sequence = msg.sequence
                self._invalidate('wheel feedback sequence gap')
                return
        if self.session_confirmed and msg.session_id != self.session:
            self._invalidate('MCU session changed unexpectedly')
            return
        if self.gate.armed and not msg.armed:
            self._invalidate('MCU unexpectedly disarmed')
            return
        if msg.armed and not self.gate.enabled:
            self._invalidate('MCU armed without host enable')
            return
        self.gate.boot = msg.boot_id
        if msg.fault_bits:
            if self.gate.enabled or self.gate.armed or self.session_confirmed:
                self._invalidate(f'MCU fault bits: {msg.fault_bits:#x}')
            self.gate.boot = msg.boot_id
            self.gate.last_state = msg.sequence
            self.last_state_sequence = msg.sequence
            self.latest, self.latest_receipt = msg, now
            self.last_state_receipt, self.last_mcu_timestamp = now, msg.timestamp_us
            if self.session == 0 and not self._started:
                self._begin()
            return
        was_armed = self.gate.armed
        fresh = self.gate.accept_state(msg.boot_id, msg.session_id, msg.sequence,
                                       msg.armed, 0)
        if not fresh:
            return
        self.latest, self.latest_receipt = msg, now
        self.last_state_receipt = now
        self.last_state_sequence = msg.sequence
        self.last_mcu_timestamp = msg.timestamp_us
        self._stop_sent = False
        if self.session != 0 and self._started and msg.session_id == self.session:
            self.session_confirmed = True
            self.begin_command = None
            self.diagnostic_error = ''
        if self.gate.armed and not was_armed:
            self.arm_sent = False
            self.arm_started = None
            self.armed_since = now
            self.last_cmd_receipt = None
            self.gate.pending_velocity = None
        if self.session_confirmed and self.gate.enabled and not self.gate.armed and not self.arm_sent:
            self._send(WheelCommand.ARM)
            self.arm_sent = True
            self.arm_started = now
        positions = self.encoder.update(msg.boot_id, msg.sequence, tuple(msg.encoder_counts))
        if positions is not None:
            out = JointState()
            out.header.stamp = self.get_clock().now().to_msg()
            out.name = ['front_left_wheel_joint', 'front_right_wheel_joint',
                        'rear_left_wheel_joint', 'rear_right_wheel_joint']
            out.position = list(positions)
            out.velocity = [float(v) for v in msg.velocity_rad_s]
            self.joints.publish(out)
        if self.session == 0 and not self._started:
            self._begin()

    def tick(self) -> None:
        now = time.monotonic()
        if self.last_state_receipt is None or now - self.last_state_receipt > .25:
            if self.latest is not None or self.session != 0 or not self._stop_sent:
                self._invalidate('stale wheel feedback')
            return
        if not self.session_confirmed:
            if self.begin_command is not None:
                self.begin_command.sequence = self.command_sequence
                self.command_sequence = (self.command_sequence + 1) & 0xffffffff
                self.begin_command.valid_until_us = (
                    self.latest.timestamp_us
                    + max(0, int((now - self.latest_receipt) * 1e6))
                    + min(200000, int(self.limits.timeout_s * 1e6))
                )
                self.publisher.publish(self.begin_command)
            elif self.session == 0:
                self._begin()
            return
        # Do not cancel an ARM in flight. The first command after its matching
        # armed feedback is the only command allowed to establish motion.
        if self.gate.enabled and not self.gate.armed:
            if self.arm_started is None:
                self.arm_started = now
            elif now - self.arm_started > .25:
                self._invalidate('ARM acknowledgement timeout')
                return
            self._send(WheelCommand.ARM)
            self.arm_sent = True
            return
        if self.gate.armed:
            if self.last_cmd_receipt is None:
                if self.armed_since is not None and now - self.armed_since > .25:
                    self._invalidate('no fresh cmd_vel after ARM')
                return
            if now - self.last_cmd_receipt > self.limits.timeout_s:
                self._stop_session()
                return
        if not self.gate.enabled:
            return
        wheels = self.gate.pending_velocity if self.gate.pending_velocity is not None else (0.,) * 4
        self._send(WheelCommand.VELOCITY, wheels)

    def publish_diagnostics(self) -> None:
        status = DiagnosticStatus()
        status.name, status.hardware_id = 'base_driver', 'micro_ros_base'
        age = float('inf') if self.last_state_receipt is None else time.monotonic()-self.last_state_receipt
        fault = self.diagnostic_error
        status.level = DiagnosticStatus.ERROR if age > .25 or fault else DiagnosticStatus.OK
        status.message = fault or ('stale wheel feedback' if age > .25 else 'feedback current')
        status.values = [KeyValue(key='armed', value=str(self.gate.armed)),
                         KeyValue(key='session_id', value=str(self.session)),
                         KeyValue(key='feedback_age_s', value=str(age))]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self.diags.publish(arr)


def main(args=None):
    # Keep the ROS context valid until the SIGINT/KeyboardInterrupt STOP has
    # been published. rclpy's default signal handler shuts it down first.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = BaseDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node._invalidate('driver shutdown')
        else:
            node.gate.reset(node.gate.boot)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
