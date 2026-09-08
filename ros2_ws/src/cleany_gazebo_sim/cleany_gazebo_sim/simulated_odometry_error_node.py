"""ROS adapter for the simulation-only wheel odometry error model."""

from __future__ import annotations

from copy import deepcopy
from math import atan2, cos, sin

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from cleany_gazebo_sim.odometry_error import (
    OdometryErrorParameters,
    Pose2D,
    StatefulOdometryError,
)


class SimulatedOdometryErrorNode(Node):
    """Apply stateful planar errors to simulation wheel odometry."""

    def __init__(self) -> None:
        super().__init__('simulated_odometry_error')
        self.declare_parameter('input_topic', 'wheel/odom_raw')
        self.declare_parameter('output_topic', 'wheel/odom')
        self.declare_parameter('forward_scale', 1.0)
        self.declare_parameter('lateral_scale', 1.0)
        self.declare_parameter('rotation_scale', 1.0)
        self.declare_parameter('forward_noise_stddev_per_sqrt_m', 0.0)
        self.declare_parameter('lateral_noise_stddev_per_sqrt_m', 0.0)
        self.declare_parameter('rotation_noise_stddev_per_sqrt_rad', 0.0)
        self.declare_parameter('yaw_bias_walk_stddev_per_sqrt_m', 0.0)
        self.declare_parameter('max_abs_yaw_bias_rad_per_m', 0.0)
        self.declare_parameter('yaw_drift_rad_per_forward_m', 0.0)
        self.declare_parameter('slip_events_per_second', 0.0)
        self.declare_parameter('slip_duration_min_sec', 0.0)
        self.declare_parameter('slip_duration_max_sec', 0.0)
        self.declare_parameter('slip_gain_min', 1.0)
        self.declare_parameter('slip_gain_max', 1.0)
        self.declare_parameter('random_seed', 42)

        self._model = StatefulOdometryError(
            OdometryErrorParameters(
                forward_scale=self._float_parameter('forward_scale'),
                lateral_scale=self._float_parameter('lateral_scale'),
                rotation_scale=self._float_parameter('rotation_scale'),
                forward_noise_stddev_per_sqrt_m=self._float_parameter(
                    'forward_noise_stddev_per_sqrt_m'
                ),
                lateral_noise_stddev_per_sqrt_m=self._float_parameter(
                    'lateral_noise_stddev_per_sqrt_m'
                ),
                rotation_noise_stddev_per_sqrt_rad=self._float_parameter(
                    'rotation_noise_stddev_per_sqrt_rad'
                ),
                yaw_bias_walk_stddev_per_sqrt_m=self._float_parameter(
                    'yaw_bias_walk_stddev_per_sqrt_m'
                ),
                max_abs_yaw_bias_rad_per_m=self._float_parameter(
                    'max_abs_yaw_bias_rad_per_m'
                ),
                yaw_drift_rad_per_forward_m=self._float_parameter(
                    'yaw_drift_rad_per_forward_m'
                ),
                slip_events_per_second=self._float_parameter(
                    'slip_events_per_second'
                ),
                slip_duration_min_sec=self._float_parameter(
                    'slip_duration_min_sec'
                ),
                slip_duration_max_sec=self._float_parameter(
                    'slip_duration_max_sec'
                ),
                slip_gain_min=self._float_parameter('slip_gain_min'),
                slip_gain_max=self._float_parameter('slip_gain_max'),
                random_seed=int(self.get_parameter('random_seed').value),
            )
        )
        self._slip_active = False
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._publisher = self.create_publisher(Odometry, output_topic, 10)
        self.create_subscription(Odometry, input_topic, self._on_odometry, 10)

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _on_odometry(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        yaw_rad = atan2(
            2.0
            * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0
            - 2.0
            * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        stamp = message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        estimate = self._model.update(
            Pose2D(
                x_m=float(message.pose.pose.position.x),
                y_m=float(message.pose.pose.position.y),
                yaw_rad=yaw_rad,
            ),
            stamp_s,
        )
        if estimate.slip_active != self._slip_active:
            state = 'started' if estimate.slip_active else 'ended'
            self.get_logger().info(f'Synthetic odometry slip {state}')
            self._slip_active = estimate.slip_active

        output = deepcopy(message)
        output.header.stamp = stamp
        output.pose.pose.position.x = estimate.pose.x_m
        output.pose.pose.position.y = estimate.pose.y_m
        output.pose.pose.orientation.x = 0.0
        output.pose.pose.orientation.y = 0.0
        output.pose.pose.orientation.z = sin(estimate.pose.yaw_rad / 2.0)
        output.pose.pose.orientation.w = cos(estimate.pose.yaw_rad / 2.0)
        output.twist.twist.linear.x = estimate.linear_x_mps
        output.twist.twist.linear.y = estimate.linear_y_mps
        output.twist.twist.angular.z = estimate.angular_z_rps
        self._publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimulatedOdometryErrorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
