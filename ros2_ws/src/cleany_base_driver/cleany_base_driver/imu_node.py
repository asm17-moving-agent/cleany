"""Apply fixed factory gyro calibration on the Jetson host."""
from __future__ import annotations

from copy import deepcopy
import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from .imu_core import FactoryCalibration, ImuAdapter, ImuValidationError


class ImuDriver(Node):
    """Validate raw IMU samples and subtract the configured factory gyro bias.

    Subscriptions:
        imu_uncalibrated/data_raw (sensor_msgs/msg/Imu): Gyro in rad/s and
            acceleration including gravity in m/s^2; Agent-synchronized
            acquisition time. QoS: B1.

    Publishers:
        imu/data_raw (sensor_msgs/msg/Imu): Bias-corrected gyro and
            unchanged acceleration in the sensor frame. QoS: B1.
        imu/status (diagnostic_msgs/msg/DiagnosticArray): Input and fixed-bias
            status; reliable, transient local, depth 1.
        diagnostics (diagnostic_msgs/msg/DiagnosticArray): Same status, depth 10.

    Notes:
        Names are remappable defaults. Invalid, stale or unavailable inputs
        produce no calibrated sample. Bias is read-only and never estimated.
        Orientation is marked unavailable; mounting TF is supplied externally.
    """

    def __init__(self, parameter_overrides=None) -> None:
        super().__init__('cleany_imu', parameter_overrides=parameter_overrides or [])
        defaults = {
            'frame_id': 'imu_link',
            'gyro_bias_rad_s': Parameter.Type.DOUBLE_ARRAY,
            'gyro_covariance_rad2_s2': Parameter.Type.DOUBLE_ARRAY,
            'calibration_source': 'configured_factory',
            'max_transport_delay_s': 0.1,
            'max_clock_jump_s': 0.05,
            'stale_timeout_s': 0.25,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default, ParameterDescriptor(read_only=True))

        def value(name: str):
            return self.get_parameter(name).value

        try:
            bias, covariance = value('gyro_bias_rad_s'), value('gyro_covariance_rad2_s2')
        except ParameterUninitializedException as error:
            raise ValueError('factory gyro bias and covariance must be configured') from error
        self.calibration = FactoryCalibration(tuple(bias), tuple(covariance))
        self.source = value('calibration_source')
        self.adapter = ImuAdapter(
            self.calibration, frame_id=value('frame_id'),
            max_transport_delay_s=value('max_transport_delay_s'),
            max_clock_jump_s=value('max_clock_jump_s'),
        )
        self.stale_timeout = value('stale_timeout_s')
        if not math.isfinite(self.stale_timeout) or self.stale_timeout <= 0:
            raise ValueError('stale_timeout_s must be finite and positive')
        self.started_at = time.monotonic()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.corrected = self.create_publisher(Imu, 'imu/data_raw', qos)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(DiagnosticArray, 'imu/status', status_qos)
        self.diags = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.create_subscription(Imu, 'imu_uncalibrated/data_raw', self.on_sample, qos)
        self.received = self.rejected = 0
        self.last_receipt: float | None = None
        self.last_error = self.last_rejection = ''
        self._last_status_key = None
        self._last_status_at = -math.inf
        self.create_timer(
            0.1, self.publish_diagnostics, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.publish_diagnostics()

    def on_sample(self, message: Imu) -> None:
        """Publish corrected gyro measurements after input and time validation."""
        try:
            if (message.angular_velocity_covariance[0] == -1.
                    or message.linear_acceleration_covariance[0] == -1.):
                raise ImuValidationError('gyro and acceleration must be available')
            stamp = message.header.stamp
            if not 0 <= stamp.nanosec < 1_000_000_000:
                raise ImuValidationError('invalid ROS timestamp nanoseconds')
            corrected = self.adapter.process(
                stamp.sec * 1_000_000_000 + stamp.nanosec, message.header.frame_id,
                (message.angular_velocity.x, message.angular_velocity.y,
                 message.angular_velocity.z),
                (message.linear_acceleration.x, message.linear_acceleration.y,
                 message.linear_acceleration.z),
                self.get_clock().now().nanoseconds, time.monotonic_ns(),
            )
        except ImuValidationError as error:
            self.rejected += 1
            self.last_error = self.last_rejection = str(error)
            self.publish_diagnostics()
            return
        output = deepcopy(message)
        output.angular_velocity.x, output.angular_velocity.y, output.angular_velocity.z = corrected
        output.angular_velocity_covariance = list(self.calibration.covariance)
        output.orientation.x = output.orientation.y = output.orientation.z = output.orientation.w = 0.
        output.orientation_covariance = [-1.] + [0.] * 8
        self.corrected.publish(output)
        self.last_receipt = time.monotonic()
        self.last_error = ''
        self.received += 1
        self.publish_diagnostics()

    def publish_diagnostics(self) -> None:
        now = time.monotonic()
        age = math.inf if self.last_receipt is None else now - self.last_receipt
        if self.last_error:
            level, state, reason = DiagnosticStatus.ERROR, 'ERROR', self.last_error
        elif ((self.last_receipt is None and now - self.started_at > self.stale_timeout)
              or (self.last_receipt is not None and age > self.stale_timeout)):
            level, state, reason = DiagnosticStatus.ERROR, 'ERROR', 'stale IMU feedback'
        elif self.last_receipt is None:
            level, state, reason = DiagnosticStatus.WARN, 'WAITING', 'waiting for raw IMU'
        else:
            level, state, reason = DiagnosticStatus.OK, 'RUNNING', 'fixed factory bias applied'
        key = (state, reason)
        if key == self._last_status_key and now - self._last_status_at < 1.:
            return
        values = {
            'bias_source': 'factory', 'calibration_source': self.source,
            'bias_rad_s': self.calibration.bias,
            'reason': reason, 'feedback_age_s': age,
            'received': self.received, 'rejected': self.rejected,
            'calibration_applied': self.last_receipt is not None,
            'timestamp_mode': 'agent_synchronized_acquisition',
            'sensor_frame': self.adapter.frame_id, 'last_rejection': self.last_rejection,
        }
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
