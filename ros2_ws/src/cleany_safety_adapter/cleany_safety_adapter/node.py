"""설정된 하드웨어 근거에서 /safety/status를 발행하며 준비 상태를 만들어 내지 않습니다."""
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
    """독립 하드웨어 입력과 움직임 피드백을 안전 상태 근거로 변환합니다.

    Subscriptions:
        estop_released_topic, protection_ready_topic (std_msgs/msg/Bool): 설정한 경우에만
            구독하는 하드웨어 해제 및 보호 준비 근거. 기본값은 빈 문자열입니다.
        odom (nav_msgs/msg/Odometry), joint_states (sensor_msgs/msg/JointState):
            정지 판정용 속도 피드백. 토픽 파라미터 기본값이며 sensor_data QoS입니다.

    Publishers:
        safety/status (cleany_interfaces/msg/SafetyStatus): 해제, 보호 준비와 정지 근거.
            reliable, depth 10이며 ROS 발행 시각을 기록합니다.

    Notes:
        입력 최신성과 timer는 단조 시계를 사용합니다. 설정된 하드웨어의 최초 근거를
        기다리는 동안 발행을 보류해 Supervisor의 부팅 유예를 사용합니다. 구성되지
        않거나 오래된 근거를 안전한 값으로 대체하지 않습니다. E-stop을 구동하거나
        물리 해제를 수행하는 노드는 아닙니다.
    """

    def __init__(self, **kwargs):
        super().__init__('safety_adapter', **kwargs)
        # 빈 토픽은 이 로봇에 해당 근거가 없음을 뜻하며 안전하지 않은 값으로 보고합니다.
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
