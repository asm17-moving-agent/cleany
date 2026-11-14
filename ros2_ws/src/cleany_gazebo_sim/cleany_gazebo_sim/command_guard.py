from __future__ import annotations

import signal
import time
from types import FrameType

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from cleany_gazebo_sim.command_limits import (
    ChassisCommand,
    CommandLimits,
    are_finite_values,
    bounded_command,
)


class GazeboCommandGuard(Node):
    """Gazebo에 전달하기 전에 평면 속도 명령을 제한합니다.

    Subscriptions:
        cmd_vel (geometry_msgs/msg/Twist): 차체 좌표계의 평면 속도 요청.

    Publishers:
        gazebo_cmd_vel (geometry_msgs/msg/Twist): 제한한 명령. 입력이 무효하거나
            명령 시간이 초과되면 0을 발행합니다.

    Notes:
        지원하지 않는 축은 무시합니다. 명령 경과 시간은 단조 증가 시계로 계산하지만
        시간 초과 콜백은 노드 타이머를 사용하므로 해당 시계가 진행되어야 합니다.
    """
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
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.SIGTERM)
    node = GazeboCommandGuard()
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    stop_requested = False

    def request_stop(_signum: int, _frame: FrameType | None) -> None:
        nonlocal stop_requested
        stop_requested = True

    # Humble의 take_message 변환 중 KeyboardInterrupt를 발생시키면 RuntimeError로
    # 나타날 수 있어 콜백이 끝난 뒤 ROS를 종료합니다.
    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    try:
        while rclpy.ok() and not stop_requested:
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            # rclpy 정리 중에도 ros2 launch가 종료 신호를 다시 보낼 수 있습니다.
            pass
        finally:
            signal.signal(signal.SIGINT, previous_sigint)
