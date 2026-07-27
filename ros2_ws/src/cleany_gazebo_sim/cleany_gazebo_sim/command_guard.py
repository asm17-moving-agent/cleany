from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from cleany_gazebo_sim.parameters import (
    ChassisCommand,
    CommandLimits,
    are_finite_values,
    bounded_command,
)


class GazeboCommandGuard(Node):
    def __init__(self) -> None:
        super().__init__('gazebo_command_guard')
        self.declare_parameter('max_linear_x', 0.3)
        self.declare_parameter('max_linear_y', 0.3)
        self.declare_parameter('max_angular_z', 0.8)
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)
        self.declare_parameter('timeout_check_rate_hz', 20.0)

        self._limits = CommandLimits(
            max_linear_x=self.get_parameter('max_linear_x').value,
            max_linear_y=self.get_parameter('max_linear_y').value,
            max_angular_z=self.get_parameter('max_angular_z').value,
        )
        self._timeout_sec = float(self.get_parameter('cmd_vel_timeout_sec').value)
        timeout_check_rate_hz = float(
            self.get_parameter('timeout_check_rate_hz').value
        )
        if self._timeout_sec <= 0.0 or timeout_check_rate_hz <= 0.0:
            raise ValueError('timeout values must be positive')

        self._last_command_time: float | None = None
        self._publisher = self.create_publisher(Twist, 'gazebo_cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_command, 10)
        self.create_timer(1.0 / timeout_check_rate_hz, self._on_timeout)

    def _on_command(self, message: Twist) -> None:
        all_axes = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        if not are_finite_values(all_axes):
            self.get_logger().warning('Ignoring non-finite cmd_vel and stopping the base')
            self._last_command_time = None
            self._publish_stop()
            return

        unsupported = (
            message.linear.z,
            message.angular.x,
            message.angular.y,
        )
        if any(value != 0.0 for value in unsupported):
            self.get_logger().warning(
                'Ignoring unsupported cmd_vel axes; only linear.x, linear.y, and angular.z are used'
            )

        try:
            command = bounded_command(
                ChassisCommand(
                    linear_x=message.linear.x,
                    linear_y=message.linear.y,
                    angular_z=message.angular.z,
                ),
                self._limits,
            )
        except ValueError:
            self.get_logger().warning('Ignoring invalid cmd_vel and stopping the base')
            self._last_command_time = None
            self._publish_stop()
            return

        output = Twist()
        output.linear.x = command.linear_x
        output.linear.y = command.linear_y
        output.angular.z = command.angular_z
        self._publisher.publish(output)
        self._last_command_time = time.monotonic()

    def _on_timeout(self) -> None:
        if self._last_command_time is None:
            return
        if time.monotonic() - self._last_command_time <= self._timeout_sec:
            return
        self.get_logger().warning('cmd_vel timed out; stopping the base')
        self._last_command_time = None
        self._publish_stop()

    def _publish_stop(self) -> None:
        self._publisher.publish(Twist())


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboCommandGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            # ros2 launch can signal the process again while rclpy tears down.
            pass
