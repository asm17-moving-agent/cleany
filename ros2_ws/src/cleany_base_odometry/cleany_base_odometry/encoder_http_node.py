from __future__ import annotations

from math import isfinite

from cleany_interfaces.msg import WheelEncoderTicks
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from cleany_base_odometry.encoder_http import EncoderHttpClient, EncoderReadError


class EncoderHttpNode(Node):
    """Poll raw ticks without commanding motors or estimating robot motion."""

    def __init__(self) -> None:
        super().__init__('encoder_http')
        self.declare_parameter('host', '192.168.4.1')
        self.declare_parameter('port', 80)
        self.declare_parameter('poll_rate_hz', 20.0)
        self.declare_parameter('request_timeout_sec', 0.2)
        self.declare_parameter('output_topic', 'wheel/encoder_ticks')
        rate = float(self.get_parameter('poll_rate_hz').value)
        if not isfinite(rate) or rate <= 0.0:
            raise ValueError('poll_rate_hz must be positive and finite')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('Hardware encoder reception requires use_sim_time=false')
        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)
        self._client = EncoderHttpClient(
            host, port,
            float(self.get_parameter('request_timeout_sec').value),
        )
        self._publisher = self.create_publisher(
            WheelEncoderTicks,
            str(self.get_parameter('output_topic').value),
            10,
        )
        self._receiving = False
        self._timer = self.create_timer(
            1.0 / rate, self._poll, clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.get_logger().info(
            f'Reading http://{host}:{port}/api/status at up to {rate:g} Hz; '
            'raw order M1 FL, M2 FR, M3 RR, M4 RL',
        )

    def _poll(self) -> None:
        try:
            sample = self._client.read()
        except EncoderReadError as error:
            self._receiving = False
            self.get_logger().warning(
                f'No encoder sample published: {error}', throttle_duration_sec=5.0,
            )
            return
        message = WheelEncoderTicks()
        message.header.stamp = self.get_clock().now().to_msg()
        message.ticks = list(sample.ticks)
        message.round_trip_time_sec = sample.round_trip_time_sec
        message.has_mcu_time = bool(sample.boot_id)
        message.boot_id = sample.boot_id
        message.sample_seq = sample.sample_seq
        message.sample_time_us = sample.sample_time_us
        self._publisher.publish(message)
        if not self._receiving:
            self.get_logger().info(f'Encoder reception active: {sample.ticks}')
            self._receiving = True

    def destroy_node(self) -> bool:
        self._client.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = EncoderHttpNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
