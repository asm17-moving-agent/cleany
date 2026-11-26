"""관절, 배터리, Odometry와 Supervisor 관측을 전달합니다."""
from __future__ import annotations

import json
import math

import rclpy
from cleany_interfaces.msg import (
    SafetyStatus, SystemEvent, SystemFaults, SystemRecoveries, SystemState,
)
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import String

from .robot_observation import ObservationBuffer, battery_sample


def vector(value: object, names: str = "xyz") -> list[float]:
    return [float(getattr(value, name)) for name in names]


class RobotObserver(Node):
    """제어 권한 없이 관측값의 원본 토픽, 시각과 출처를 보존합니다.

    Subscriptions:
        /joint_states, /battery_state, /odom: 기본 센서 토픽, sensor_data QoS.
        /system/state, /system/faults, /system/recoveries: reliable transient_local.
        /system/events, /safety/status: reliable volatile.
        joint_targets_topic: 명시 설정한 경우에만 별도의 명령 목표를 관측합니다.
    Publishers:
        /telemetry/robot_observations (std_msgs/msg/String): 버전 1 JSON 관측.
    Notes:
        provenance 파라미터를 real, sim 또는 mock으로 명시해야 합니다.
        ROS 시각으로 오래된 값을 거부하며 명령, 서비스, TF를 발행하지 않습니다.
    """

    def __init__(self, **kwargs) -> None:
        super().__init__("robot_observer", **kwargs)
        values = dict(provenance="", joints_topic="/joint_states", battery_topic="/battery_state",
                      odom_topic="/odom", joint_targets_topic="", supervisor_state_topic="/system/state",
                      supervisor_faults_topic="/system/faults", supervisor_recoveries_topic="/system/recoveries",
                      supervisor_events_topic="/system/events", safety_topic="/safety/status",
                      output_topic="/telemetry/robot_observations", rate=10.0, input_timeout=3.0,
                      battery_timeout=30.0, supervisor_timeout=5.0, future_tolerance=0.1,
                      event_capacity=100)
        for name, value in values.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        if not math.isfinite(p("rate")) or p("rate") <= 0:
            raise ValueError("rate must be finite and positive")
        self.buffer = ObservationBuffer(p("provenance"), p("input_timeout"), p("battery_timeout"),
                                        p("supervisor_timeout"), p("future_tolerance"), p("event_capacity"))
        self.publisher = self.create_publisher(String, p("output_topic"), 100)
        self.create_timer(1 / p("rate"), self.flush)
        sensor = qos_profile_sensor_data
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions_owned = []

        def subscribe(cls, parameter, convert, qos):
            topic = p(parameter)
            if topic:
                self.subscriptions_owned.append(self.create_subscription(cls, topic,
                    lambda message: self.observe(topic, message, convert), qos))

        subscribe(JointState, "joints_topic", lambda m: dict(kind="joints", names=list(m.name),
                  positions=list(m.position)), sensor)
        subscribe(JointState, "joint_targets_topic", lambda m: dict(kind="joint_targets",
                  names=list(m.name), positions=list(m.position)), sensor)
        subscribe(BatteryState, "battery_topic", lambda m: battery_sample(m.percentage, m.voltage,
                  m.power_supply_status), sensor)
        subscribe(Odometry, "odom_topic", self.base, sensor)
        subscribe(SystemState, "supervisor_state_topic", lambda m: dict(kind="supervisor_state",
                  epoch=m.epoch, operating_state=m.operating_state), latched)
        subscribe(SystemFaults, "supervisor_faults_topic", lambda m: dict(kind="supervisor_faults",
                  epoch=m.epoch, faults=[{k: getattr(f, k) for k in (
                      "code", "source", "severity", "active", "latched", "recovery_policy", "detail")}
                      for f in m.faults]), latched)
        subscribe(SystemRecoveries, "supervisor_recoveries_topic", lambda m: dict(
                  kind="supervisor_recoveries", epoch=m.epoch, recoveries=[{k: getattr(r, k) for k in (
                      "job_id", "fault_code", "state", "attempt", "verifying_health", "resource_held", "detail")}
                      for r in m.recoveries]), latched)
        subscribe(SystemEvent, "supervisor_events_topic", lambda m: dict(kind="supervisor_event",
                  epoch=m.epoch, event_sequence=m.sequence, event_kind=m.kind, code=m.code,
                  severity=m.severity, detail=m.detail), 100)
        subscribe(SafetyStatus, "safety_topic", lambda m: dict(kind="safety", **{k: getattr(m, k)
                  for k in ("protection_ready", "estop_engaged", "physical_release_confirmed", "stopped", "detail")}), 10)

    @staticmethod
    def base(message: Odometry) -> dict:
        pose, twist = message.pose.pose, message.twist.twist
        q = vector(pose.orientation, "xyzw")
        norm = math.sqrt(sum(v*v for v in q))
        if not .99 <= norm <= 1.01 or not message.header.frame_id or not message.child_frame_id:
            raise ValueError("invalid odometry frame or orientation")
        return dict(kind="base", frame_id=message.header.frame_id, child_frame_id=message.child_frame_id,
                    position=vector(pose.position), orientation=[v/norm for v in q],
                    linear_velocity=vector(twist.linear), angular_velocity=vector(twist.angular))

    def observe(self, topic: str, message: object, convert) -> None:
        try:
            sample = convert(message)
            json.dumps(sample, allow_nan=False)
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            self.buffer.accept(topic, stamp, self.get_clock().now().nanoseconds*1e-9, sample)
        except (ValueError, TypeError, OverflowError):
            self.get_logger().warning("Invalid observation ignored", throttle_duration_sec=5)

    def flush(self) -> None:
        for frame in self.buffer.drain(self.get_clock().now().nanoseconds*1e-9):
            self.publisher.publish(String(data=json.dumps(frame, allow_nan=False)))
        if self.buffer.dropped_events:
            self.get_logger().warning(f"Observation events dropped: {self.buffer.dropped_events}",
                                      throttle_duration_sec=5)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotObserver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
