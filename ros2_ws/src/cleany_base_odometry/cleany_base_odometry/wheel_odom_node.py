from __future__ import annotations

from math import cos, sin

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from cleany_base_odometry.mecanum_odometry import (
    MecanumGeometry,
    MecanumOdometry,
    WheelPositions,
)


class WheelOdometryNode(Node):
    """Publish planar wheel odometry from four cumulative joint positions."""

    def __init__(self) -> None:
        super().__init__('wheel_odometry')
        self.declare_parameter('input_topic', 'joint_states')
        self.declare_parameter('output_topic', 'wheel/odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('wheel_radius_m', 0.0635)
        self.declare_parameter('wheelbase_m', 0.30)
        self.declare_parameter('wheel_separation_m', 0.51)
        self.declare_parameter(
            'front_left_joint', 'front_left_wheel_joint'
        )
        self.declare_parameter(
            'front_right_joint', 'front_right_wheel_joint'
        )
        self.declare_parameter('rear_left_joint', 'rear_left_wheel_joint')
        self.declare_parameter('rear_right_joint', 'rear_right_wheel_joint')

        geometry = MecanumGeometry(
            wheel_radius_m=float(self.get_parameter('wheel_radius_m').value),
            wheelbase_m=float(self.get_parameter('wheelbase_m').value),
            wheel_separation_m=float(
                self.get_parameter('wheel_separation_m').value
            ),
        )
        self._odometry = MecanumOdometry(geometry)
        self._odom_frame_id = str(self.get_parameter('odom_frame_id').value)
        self._base_frame_id = str(self.get_parameter('base_frame_id').value)
        self._joint_names = {
            'front_left': str(self.get_parameter('front_left_joint').value),
            'front_right': str(
                self.get_parameter('front_right_joint').value
            ),
            'rear_left': str(self.get_parameter('rear_left_joint').value),
            'rear_right': str(self.get_parameter('rear_right_joint').value),
        }
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._publisher = self.create_publisher(Odometry, output_topic, 10)
        self.create_subscription(
            JointState,
            input_topic,
            self._on_joint_state,
            qos_profile_sensor_data,
        )

    def _on_joint_state(self, message: JointState) -> None:
        if len(message.position) < len(message.name):
            self.get_logger().warning(
                'Ignoring JointState with fewer positions than names',
                throttle_duration_sec=5.0,
            )
            return

        position_by_name = dict(zip(message.name, message.position))
        missing = [
            joint_name
            for joint_name in self._joint_names.values()
            if joint_name not in position_by_name
        ]
        if missing:
            self.get_logger().warning(
                f'Ignoring JointState missing wheel joints: {missing}',
                throttle_duration_sec=5.0,
            )
            return

        positions = WheelPositions(
            front_left=float(
                position_by_name[self._joint_names['front_left']]
            ),
            front_right=float(
                position_by_name[self._joint_names['front_right']]
            ),
            rear_left=float(
                position_by_name[self._joint_names['rear_left']]
            ),
            rear_right=float(
                position_by_name[self._joint_names['rear_right']]
            ),
        )
        stamp = message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        estimate = self._odometry.update(positions, stamp_s)
        if estimate is None:
            return

        output = Odometry()
        output.header.stamp = stamp
        output.header.frame_id = self._odom_frame_id
        output.child_frame_id = self._base_frame_id
        output.pose.pose.position.x = estimate.x_m
        output.pose.pose.position.y = estimate.y_m
        output.pose.pose.orientation.z = sin(estimate.yaw_rad / 2.0)
        output.pose.pose.orientation.w = cos(estimate.yaw_rad / 2.0)
        output.twist.twist.linear.x = estimate.linear_x_mps
        output.twist.twist.linear.y = estimate.linear_y_mps
        output.twist.twist.angular.z = estimate.angular_z_rps
        self._publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WheelOdometryNode()
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
