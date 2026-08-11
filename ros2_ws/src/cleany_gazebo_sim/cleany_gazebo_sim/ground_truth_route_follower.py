from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

from cleany_gazebo_sim.route_control import (
    Pose2D,
    RouteLimits,
    RouteTracker,
    waypoints_from_flat,
)


class GroundTruthRouteFollower(Node):
    def __init__(self) -> None:
        super().__init__('ground_truth_route_follower')
        self.declare_parameter('waypoints_xy', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('max_linear_speed', 0.15)
        self.declare_parameter('max_angular_speed', 0.25)
        self.declare_parameter('heading_gain', 1.2)
        self.declare_parameter('position_tolerance', 0.09)
        self.declare_parameter('turn_in_place_threshold', 0.45)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('odom_timeout_sec', 0.5)

        waypoints = waypoints_from_flat(
            self.get_parameter('waypoints_xy').value
        )
        self._tracker = RouteTracker(
            waypoints,
            RouteLimits(
                max_linear_speed=float(
                    self.get_parameter('max_linear_speed').value
                ),
                max_angular_speed=float(
                    self.get_parameter('max_angular_speed').value
                ),
                heading_gain=float(self.get_parameter('heading_gain').value),
                position_tolerance=float(
                    self.get_parameter('position_tolerance').value
                ),
                turn_in_place_threshold=float(
                    self.get_parameter('turn_in_place_threshold').value
                ),
            ),
        )
        rate = float(self.get_parameter('control_rate_hz').value)
        self._odom_timeout_sec = float(
            self.get_parameter('odom_timeout_sec').value
        )
        if rate <= 0.0 or self._odom_timeout_sec <= 0.0:
            raise ValueError('control rate and odom timeout must be positive')

        self._latest_pose: Pose2D | None = None
        self._latest_odom_ns: int | None = None
        self._completed = False
        self._last_reported_index = -1
        self._publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(
            Odometry, 'ground_truth/odom', self._on_odometry, 10
        )
        self.create_timer(1.0 / rate, self._on_control)
        self.get_logger().info(
            f'Loaded route with {self._tracker.waypoint_count} waypoints'
        )

    def _on_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
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
        self._latest_pose = Pose2D(position.x, position.y, yaw)
        self._latest_odom_ns = self.get_clock().now().nanoseconds

    def _on_control(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._latest_pose is None or self._latest_odom_ns is None:
            self._publish_stop()
            return
        age_sec = (now_ns - self._latest_odom_ns) / 1_000_000_000.0
        if age_sec > self._odom_timeout_sec:
            self._publish_stop()
            return
        if self._completed:
            self._publish_stop()
            return

        command = self._tracker.command(self._latest_pose)
        if command.waypoint_index != self._last_reported_index:
            self._last_reported_index = command.waypoint_index
            self.get_logger().info(
                'Following waypoint '
                f'{command.waypoint_index + 1}/'
                f'{self._tracker.waypoint_count}'
            )
        if command.completed:
            self._completed = True
            self._publish_stop()
            self.get_logger().info('Study-cafe evaluation route completed')
            return

        message = Twist()
        message.linear.x = command.linear_x
        message.angular.z = command.angular_z
        self._publisher.publish(message)

    def _publish_stop(self) -> None:
        self._publisher.publish(Twist())


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GroundTruthRouteFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
