from __future__ import annotations

from math import isfinite, sin, cos
from time import monotonic

from cleany_interfaces.msg import WheelEncoderTicks
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from cleany_base_odometry.encoder_http import EncoderSample
from cleany_base_odometry.encoder_odometry import (
    EncoderAngleTracker, EncoderCalibration, RejectedEncoderSample,
)
from cleany_base_odometry.mecanum_odometry import MecanumGeometry, MecanumOdometry


class HardwareOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__('hardware_wheel_odometry')
        defaults = {
            'input_topic': 'wheel/encoder_ticks',
            'joint_state_topic': 'wheel_encoder/joint_states',
            'output_topic': 'wheel/odom', 'canonical_topic': 'odom',
            'odom_frame_id': 'odom', 'base_frame_id': 'base_link',
            'publish_tf': True, 'calibration_verified': False,
            'ticks_per_revolution': [3172.0] * 4,
            'encoder_signs': [1, -1, -1, 1],
            'wheel_radius_m': 0.0635, 'wheelbase_m': 0.30,
            'wheel_separation_m': 0.51, 'max_sample_gap_sec': 0.5,
            'max_wheel_speed_rad_s': 50.0, 'max_clock_error_sec': 0.25,
            'pose_covariance_diagonal': [0.25, 0.25, 1e6, 1e6, 1e6, 0.25],
            'twist_covariance_diagonal': [0.04, 0.04, 1e6, 1e6, 1e6, 0.09],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        if value('use_sim_time'):
            raise ValueError('Hardware odometry requires use_sim_time=false')
        self._tracker = EncoderAngleTracker(
            EncoderCalibration(tuple(value('ticks_per_revolution')), tuple(value('encoder_signs'))),
            value('max_sample_gap_sec'), value('max_wheel_speed_rad_s'),
            value('max_clock_error_sec'),
        )
        self._odom = MecanumOdometry(MecanumGeometry(
            value('wheel_radius_m'), value('wheelbase_m'), value('wheel_separation_m'),
        ))
        self._frame = value('odom_frame_id')
        self._base = value('base_frame_id')
        if not self._frame or not self._base or self._frame == self._base:
            raise ValueError('Distinct nonempty odom/base frames are required')
        self._pose_covariance = self._covariance(value('pose_covariance_diagonal'))
        self._twist_covariance = self._covariance(value('twist_covariance_diagonal'))
        self._publisher = self.create_publisher(Odometry, value('output_topic'), 10)
        self._canonical = None
        if value('canonical_topic') and self.resolve_topic_name(value('canonical_topic')) != self.resolve_topic_name(value('output_topic')):
            self._canonical = self.create_publisher(Odometry, value('canonical_topic'), 10)
        self._joints = self.create_publisher(JointState, value('joint_state_topic'), 10)
        self._tf = TransformBroadcaster(self) if value('publish_tf') else None
        self._last_good: float | None = None
        self._max_gap_sec = value('max_sample_gap_sec')
        self._stale = True
        self.create_subscription(
            WheelEncoderTicks, value('input_topic'), self._on_ticks, qos_profile_sensor_data,
        )
        self.create_timer(0.1, self._check_stale, clock=Clock(clock_type=ClockType.STEADY_TIME))
        if not value('calibration_verified'):
            self.get_logger().warning(
                'Using firmware encoder calibration and nominal model geometry; '
                'distance/yaw and covariance are not validated on the floor',
            )

    @staticmethod
    def _covariance(diagonal: list[float]) -> list[float]:
        if len(diagonal) != 6 or not all(isfinite(v) and v > 0 for v in diagonal):
            raise ValueError('Covariance requires six positive finite diagonal values')
        result = [0.0] * 36
        for index, entry in enumerate(diagonal):
            result[index * 7] = float(entry)
        return result

    def _check_stale(self) -> None:
        if self._last_good is not None and monotonic() - self._last_good > self._max_gap_sec:
            if not self._stale:
                self.get_logger().warning('Encoder input stale; odometry and TF publication stopped')
                self._stale = True

    def _on_ticks(self, message: WheelEncoderTicks) -> None:
        try:
            if not message.has_mcu_time:
                raise RejectedEncoderSample('Firmware snapshot metadata is required')
            receive_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            age_sec = (self.get_clock().now().nanoseconds - receive_ns) * 1e-9
            if age_sec > self._max_gap_sec or age_sec < -self._max_gap_sec:
                raise RejectedEncoderSample('Stale or future-dated ROS snapshot')
            angles = self._tracker.update(EncoderSample(
                tuple(int(v) for v in message.ticks), message.round_trip_time_sec,
                message.boot_id, int(message.sample_seq), int(message.sample_time_us),
            ), receive_ns)
        except RejectedEncoderSample as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        self._last_good = monotonic()
        if self._stale:
            self.get_logger().info('Timestamped encoder input active')
            self._stale = False
        if angles.baseline_reason:
            self._odom.reset_baseline()
            self.get_logger().info(f'Wheel baseline reset: {angles.baseline_reason}; pose retained')
        stamp = Time(nanoseconds=angles.stamp_ns).to_msg()
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = [
            'front_left_wheel_joint', 'front_right_wheel_joint',
            'rear_left_wheel_joint', 'rear_right_wheel_joint',
        ]
        joints.position = list(angles.positions.as_tuple())
        self._joints.publish(joints)
        estimate = self._odom.update(angles.positions, angles.stamp_ns * 1e-9)
        if estimate is None:
            return
        output = Odometry()
        output.header.stamp = stamp
        output.header.frame_id = self._frame
        output.child_frame_id = self._base
        output.pose.pose.position.x = estimate.x_m
        output.pose.pose.position.y = estimate.y_m
        output.pose.pose.orientation.z = sin(estimate.yaw_rad / 2)
        output.pose.pose.orientation.w = cos(estimate.yaw_rad / 2)
        output.twist.twist.linear.x = estimate.linear_x_mps
        output.twist.twist.linear.y = estimate.linear_y_mps
        output.twist.twist.angular.z = estimate.angular_z_rps
        output.pose.covariance = self._pose_covariance
        output.twist.covariance = self._twist_covariance
        self._publisher.publish(output)
        if self._canonical:
            self._canonical.publish(output)
        if self._tf:
            transform = TransformStamped()
            transform.header = output.header
            transform.child_frame_id = self._base
            transform.transform.translation.x = estimate.x_m
            transform.transform.translation.y = estimate.y_m
            transform.transform.rotation = output.pose.pose.orientation
            self._tf.sendTransform(transform)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = HardwareOdometryNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
