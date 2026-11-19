"""Passive, live-data readiness checks; never enable the base or publish commands."""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Any


@dataclass
class Stream:
    received_at: float = -math.inf
    stamp: int = -1
    samples: int = 0
    error: str = "no samples received"

    def receive(self, stamp: int, now: float, error: str = "") -> None:
        if stamp <= self.stamp:
            error = error or "duplicate or backwards timestamp"
        self.samples = 0 if error else self.samples + 1
        self.error = error
        self.stamp = stamp
        self.received_at = now

    def fresh(self, now: float, timeout: float) -> bool:
        return self.samples >= 3 and not self.error and now - self.received_at <= timeout


class ReadinessState:
    def __init__(self, odom_timeout_s: float, scan_timeout_s: float, min_range_m: float):
        self.odom_timeout_s = odom_timeout_s
        self.scan_timeout_s = scan_timeout_s
        self.min_range_m = min_range_m
        self.odom = Stream()
        self.scan = Stream()
        self.scan_delivery = Stream()
        self.wheel = Stream()

    @staticmethod
    def stamp(message: Any) -> int:
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def receive_odom(self, msg: Any, now: float, ros_now_ns: int) -> None:
        if now - self.odom.received_at > self.odom_timeout_s:
            self.odom.samples = 0
        pose, twist = msg.pose.pose, msg.twist.twist
        values = (pose.position.x, pose.position.y, pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w, twist.linear.x, twist.linear.y,
                  twist.angular.z)
        age = (ros_now_ns - self.stamp(msg)) / 1e9
        error = ""
        if not all(math.isfinite(v) for v in values):
            error = "nonfinite odometry"
        elif not 0 <= age <= self.odom_timeout_s:
            error = "stale or future odometry stamp"
        self.odom.receive(self.stamp(msg), now, error)

    def receive_scan(self, msg: Any, now: float, ros_now_ns: int) -> None:
        if now - self.scan.received_at > self.scan_timeout_s:
            self.scan.samples = self.scan_delivery.samples = 0
        age = (ros_now_ns - self.stamp(msg)) / 1e9
        count = sum(math.isfinite(r) and max(msg.range_min, self.min_range_m) < r < msg.range_max
                    for r in msg.ranges)
        error = ""
        if not 0 <= age <= self.scan_timeout_s:
            error = "stale or future scan stamp"
        self.scan_delivery.receive(self.stamp(msg), now, error)
        if not error and count < 30:
            error = "fewer than 30 external-environment returns after range filtering"
        self.scan.receive(self.stamp(msg), now, error)

    def receive_wheel(self, msg: Any, now: float) -> None:
        if now - self.wheel.received_at > .25:
            self.wheel.samples = 0
        error = ""
        if msg.enabled or msg.fault_bits:
            error = "base enabled or MCU fault; require disabled fault-free state"
        elif any(msg.target_rad_s) or any(msg.commanded_rad_s) or any(msg.pwm_percent):
            error = "nonzero wheel command/PWM"
        self.wheel.receive(msg.timestamp_us, now, error)

    def snapshot(self, now: float, publishers: dict[str, int], enable_available: bool) -> dict:
        fresh = {
            "odom": self.odom.fresh(now, self.odom_timeout_s),
            "scan": self.scan.fresh(now, self.scan_timeout_s),
            # MCU feedback stale deadline is the base protocol's 250 ms.
            "wheel": self.wheel.fresh(now, .25),
        }
        errors = []
        for name in ("odom", "scan", "wheel"):
            stream = getattr(self, name)
            if not fresh[name]:
                errors.append(f"{name}: {stream.error or 'waiting for three fresh advancing samples'}")
        if not enable_available:
            errors.append("service /base/enable unavailable")
        if publishers["cmd_vel"]:
            errors.append("another /cmd_vel publisher is active")
        for name in ("wheel_command", "wheel_state", "odom", "scan"):
            if publishers[name] != 1:
                errors.append(f"{name}: expected one publisher, found {publishers[name]}")
        return {
            "ready": not errors, "errors": errors, "enable_available": enable_available,
            "publishers": publishers, **{f"{name}_fresh": value for name, value in fresh.items()},
            # A live scan with poor scene geometry must not trigger a driver restart.
            "scan_fresh": self.scan_delivery.fresh(now, self.scan_timeout_s),
        }


class RosReadinessProbe:
    """ROS imports/initialization occur only for a real run, never for dry-run."""

    def __init__(self, odom_topic: str, odom_timeout_s: float,
                 scan_timeout_s: float, min_range_m: float):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from cleany_base_interfaces.msg import WheelState
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        from std_srvs.srv import SetBool

        self.rclpy = rclpy
        self.context = Context()
        rclpy.init(context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = Node("wheel_eval_readiness", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.state = ReadinessState(odom_timeout_s, scan_timeout_s, min_range_m)
        self.topics = {"cmd_vel": "/cmd_vel", "wheel_command": "/base/wheel_command",
                       "wheel_state": "/base/wheel_state", "odom": odom_topic, "scan": "/scan"}
        # Match evaluator's reliable odom subscription; telemetry uses best effort.
        telemetry = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions = [
            self.node.create_subscription(
                Odometry, odom_topic,
                lambda msg: self.state.receive_odom(
                    msg, time.monotonic(), self.node.get_clock().now().nanoseconds), 10),
            self.node.create_subscription(
                LaserScan, "/scan",
                lambda msg: self.state.receive_scan(
                    msg, time.monotonic(), self.node.get_clock().now().nanoseconds), telemetry),
            self.node.create_subscription(
                WheelState, "/base/wheel_state",
                lambda msg: self.state.receive_wheel(msg, time.monotonic()), telemetry),
        ]
        self.enable = self.node.create_client(SetBool, "/base/enable")

    def spin(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and self.context.ok():
            self.executor.spin_once(timeout_sec=max(0., min(.05, deadline - time.monotonic())))

    def snapshot(self) -> dict:
        return self.state.snapshot(time.monotonic(), {
            name: self.node.count_publishers(topic) for name, topic in self.topics.items()
        }, self.enable.service_is_ready())

    def close(self) -> None:
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.try_shutdown()
