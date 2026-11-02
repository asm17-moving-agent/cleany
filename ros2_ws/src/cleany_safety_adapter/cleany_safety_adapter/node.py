"""Publishes /safety/status from configured hardware evidence. It never fabricates readiness."""
from time import monotonic

import rclpy
from cleany_interfaces.msg import SafetyStatus
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from .evidence import SafetyEvidence


class SafetyAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__('safety_adapter', **kwargs)
        # Empty topic = the evidence does not exist on this robot; reported as unsafe.
        estop_topic = self.declare_parameter('estop_released_topic', '').value
        protection_topic = self.declare_parameter('protection_ready_topic', '').value
        self.evidence = SafetyEvidence(
            timeout_s=self.declare_parameter('evidence_timeout_s', .3).value,
            linear_mps=self.declare_parameter('stopped_linear_mps', .01).value,
            angular_radps=self.declare_parameter('stopped_angular_radps', .02).value,
            joint_radps=self.declare_parameter('stopped_joint_radps', .02).value,
            monitored_joints=list(self.declare_parameter('monitored_joints', ['']).value or []),
            estop_configured=bool(estop_topic), protection_configured=bool(protection_topic))
        self.evidence.monitored_joints = [name for name in self.evidence.monitored_joints if name]
        if estop_topic:
            self.create_subscription(Bool, estop_topic,
                                     lambda msg: self.evidence.estop_released(msg.data, monotonic()), 10)
        if protection_topic:
            self.create_subscription(Bool, protection_topic,
                                     lambda msg: self.evidence.protection_ready(msg.data, monotonic()), 10)
        self.create_subscription(Odometry, self.declare_parameter('odom_topic', 'odom').value, self._odom,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, self.declare_parameter('joint_states_topic', 'joint_states').value,
                                 lambda msg: self.evidence.joints(msg.name, msg.velocity, monotonic()),
                                 qos_profile_sensor_data)
        self.publisher = self.create_publisher(SafetyStatus, 'safety/status', 10)
        self.create_timer(1. / self.declare_parameter('rate_hz', 20.).value, self._publish,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        if not (estop_topic and protection_topic):
            self.get_logger().warn('safety evidence source not configured: operation stays locked')

    def _odom(self, msg):
        twist = msg.twist.twist
        self.evidence.odometry(twist.linear.x, twist.linear.y, twist.angular.z, monotonic())

    def _publish(self):
        if self.evidence.awaiting_first_evidence():
            self.get_logger().warn('waiting for first hardware safety evidence', throttle_duration_sec=5.)
            return
        report = self.evidence.evaluate(monotonic())
        msg = SafetyStatus(protection_ready=report.protection_ready, estop_engaged=report.estop_engaged,
                           physical_release_confirmed=report.physical_release_confirmed,
                           stopped=report.stopped, detail=report.detail)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
