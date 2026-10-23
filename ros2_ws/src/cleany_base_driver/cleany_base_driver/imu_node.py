"""Receive MCU power-on calibration and publish live gyro and status."""
from __future__ import annotations

import math
import time

import rclpy
from cleany_base_interfaces.msg import ImuState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu

from .imu_core import ImuAdapter, ImuValidationError


class ImuDriver(Node):
    """Relay MCU-calibrated gyro without starting or repeating calibration."""

    def __init__(self, parameter_overrides=None) -> None:
        super().__init__('cleany_imu', parameter_overrides=parameter_overrides or [])
        defaults = {
            'frame_id': 'imu_link',
            'expected_device_mac': '',
            'max_transport_delay_s': 0.1,
            'max_clock_jump_s': 0.05,
            'stale_timeout_s': 0.25,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        def value(name: str):
            return self.get_parameter(name).value

        self.frame_id = value('frame_id')
        if not self.frame_id or self.frame_id.startswith('/') or any(
                char.isspace() for char in self.frame_id):
            raise ValueError('frame_id must be a nonempty relative sensor-frame name')
        self.adapter = ImuAdapter(
            expected_device_mac=value('expected_device_mac'),
            max_transport_delay_s=value('max_transport_delay_s'),
            max_clock_jump_s=value('max_clock_jump_s'),
        )
        self.stale_timeout = value('stale_timeout_s')
        if not math.isfinite(self.stale_timeout) or self.stale_timeout <= 0:
            raise ValueError('stale_timeout_s must be finite and positive')
        self.started_at = time.monotonic()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.raw = self.create_publisher(Imu, 'imu/data_raw', qos)
        self.corrected = self.create_publisher(Imu, 'imu/data', qos)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(DiagnosticArray, 'imu/status', status_qos)
        self.diags = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.create_subscription(ImuState, 'base/imu_state', self.on_state, qos)
        self.received = self.rejected = 0
        self.last_sample: ImuState | None = None
        self.last_receipt: float | None = None
        self.last_error = self.last_rejection = ''
        self._last_status_key = None
        self._last_status_at = -math.inf
        self.create_timer(
            0.1, self.publish_diagnostics, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.publish_diagnostics()

    def on_state(self, message: ImuState) -> None:
        try:
            stamp = self.adapter.stamp(
                message, self.get_clock().now().nanoseconds, time.monotonic_ns()
            )
        except ImuValidationError as error:
            self.rejected += 1
            self.last_error = self.last_rejection = str(error)
            self.publish_diagnostics()
            return
        now = time.monotonic()
        self.last_sample, self.last_receipt = message, now
        self.last_error = ''
        self.received += 1
        self.raw.publish(self._message(stamp, message.angular_velocity_rad_s, (0.,) * 9))
        if message.calibration_state in (ImuState.RUNNING, ImuState.FALLBACK):
            self.corrected.publish(self._message(
                stamp, message.corrected_angular_velocity_rad_s,
                message.gyro_covariance_rad2_s2))
        self.publish_diagnostics()

    def _message(self, stamp_ns: int, angular_velocity, covariance) -> Imu:
        message = Imu()
        message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        message.header.frame_id = self.frame_id
        message.orientation_covariance[0] = -1.
        message.linear_acceleration_covariance[0] = -1.
        message.angular_velocity.x = angular_velocity[0]
        message.angular_velocity.y = angular_velocity[1]
        message.angular_velocity.z = angular_velocity[2]
        message.angular_velocity_covariance = list(covariance)
        return message

    def publish_diagnostics(self) -> None:
        now = time.monotonic()
        age = math.inf if self.last_receipt is None else now - self.last_receipt
        sample = self.last_sample
        selected = sample is not None and sample.calibration_state in (ImuState.RUNNING, ImuState.FALLBACK)
        source = ('fallback' if sample.calibration_state == ImuState.FALLBACK else 'startup') if selected else 'none'
        reason = 'waiting for MCU telemetry'
        if sample is not None:
            reason = {
                ImuState.REASON_NONE: 'completed' if selected else 'MCU boot calibration',
                ImuState.REASON_TIMEOUT: 'calibration timeout',
                ImuState.REASON_SENSOR: 'sensor failure during calibration',
                ImuState.REASON_UNSTABLE: 'unstable calibration',
                ImuState.REASON_SAMPLE: 'invalid calibration sample or timing',
            }.get(sample.calibration_reason, 'MCU calibration')
        if self.last_error:
            level, state, reason = DiagnosticStatus.ERROR, 'ERROR', self.last_error
        elif ((self.last_receipt is None and now - self.started_at > self.stale_timeout)
              or (self.last_receipt is not None and age > self.stale_timeout)):
            level, state, reason = DiagnosticStatus.ERROR, 'ERROR', 'stale IMU feedback'
        elif not selected:
            level, state = DiagnosticStatus.WARN, 'CALIBRATING'
        elif sample.calibration_state == ImuState.FALLBACK:
            level, state = DiagnosticStatus.WARN, 'FALLBACK'
        else:
            level, state = DiagnosticStatus.OK, 'RUNNING'
        bias = tuple(sample.gyro_bias_rad_s) if selected else ()
        key = (state, reason, source, bias, sample.boot_id if sample is not None else None)
        if key == self._last_status_key and now - self._last_status_at < 1.:
            return
        values = {
            'bias_source': source,
            'bias_rad_s': bias,
            'reason': reason, 'feedback_age_s': age,
            'received': self.received, 'rejected': self.rejected,
            'calibration_applied': selected,
            'timestamp_mode': 'receipt_anchored_mcu_intervals',
            'sensor_frame': self.frame_id, 'last_rejection': self.last_rejection,
        }
        if self.last_sample is not None:
            values.update({
                'device_mac': ':'.join(f'{v:02x}' for v in self.last_sample.device_mac),
                'boot_id': self.last_sample.boot_id, 'sequence': self.last_sample.sequence,
                'temperature_c': self.last_sample.temperature_c,
            })
        status = DiagnosticStatus(
            name='imu_driver', hardware_id='mpu6050', level=level, message=state,
            values=[KeyValue(key=k, value=str(v)) for k, v in values.items()],
        )
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.status = [status]
        self.status.publish(output)
        self.diags.publish(output)
        self._last_status_key, self._last_status_at = key, now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ImuDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
