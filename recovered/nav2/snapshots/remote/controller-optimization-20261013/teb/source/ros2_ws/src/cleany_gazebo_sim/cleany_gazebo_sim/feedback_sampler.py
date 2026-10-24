"""Gazebo adapter: publish coherent clock/joint snapshots at bounded frequency."""
from __future__ import annotations
import rclpy
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


class FeedbackSampler(Node):
    def __init__(self) -> None:
        super().__init__('gazebo_feedback_sampler')
        self.declare_parameter('publish_period_s', .02)
        period = self.get_parameter('publish_period_s').value
        if not 0. <= period <= 1. or period == 0:
            raise ValueError('publish_period_s must be in (0, 1]')
        self.clock_msg = self.joints = self.coherent_joints = None
        self.last_clock = self.last_joint = None
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
        self.create_subscription(Clock, '/gazebo/raw_clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(JointState, '/gazebo/raw_joint_states', self.on_joint, qos_profile_sensor_data)
        self.create_timer(period, self.publish_latest, clock=RosClock(clock_type=ClockType.STEADY_TIME))

    def on_clock(self, msg: Clock) -> None:
        if self.clock_msg is not None and self.stamp(msg.clock) < self.stamp(self.clock_msg.clock):
            self.coherent_joints = None
        self.clock_msg = msg
        self.capture_joint()

    def on_joint(self, msg: JointState) -> None:
        self.joints = msg
        self.capture_joint()

    @staticmethod
    def stamp(t):
        return t.sec, t.nanosec

    def capture_joint(self) -> None:
        if self.clock_msg is not None and self.joints is not None and self.stamp(self.joints.header.stamp) <= self.stamp(self.clock_msg.clock):
            self.coherent_joints = self.joints

    def publish_latest(self) -> None:
        if self.clock_msg is None:
            return
        t = self.clock_msg.clock
        stamp = (t.sec, t.nanosec)
        if self.last_clock is not None and stamp < self.last_clock:
            self.last_joint = None
        if stamp != self.last_clock:
            self.clock_pub.publish(self.clock_msg)
            self.last_clock = stamp
        if self.coherent_joints is not None:
            t = self.coherent_joints.header.stamp
            joint = (t.sec, t.nanosec)
            if joint <= stamp and joint != self.last_joint:
                self.joint_pub.publish(self.coherent_joints)
                self.last_joint = joint
        # Never restamp or repeat a stopped simulator's data as fresh feedback.


def main(args=None) -> None:
    rclpy.init(args=args);node = FeedbackSampler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
