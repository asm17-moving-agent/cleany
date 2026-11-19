"""기존 Nav2 안전 경로 뒤의 최종 ToF 안전 제한입니다."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cleany_interfaces.msg import ComponentHealth
from .proximity import ProximityDetector, limit_velocity
from .ros_support import Inputs, SupervisorView, settings, steady_timer, twist, velocity


class ProximityGuard(Node):
    """proximity/input을 받아 30Hz로 cmd_vel(Twist, depth 1)을 발행합니다.

    tof/frame과 odom이 최신이어야 합니다. proximity/status(String, depth 1)는
    로컬 제한 이유만 보고하며 이 노드는 Mission이나 Supervisor 상태를 소유하지
    않습니다. 프로세스 종료는 모터 드라이버의 독립 명령 watchdog이 처리합니다.
    """
    def __init__(self, **kwargs):
        super().__init__('proximity_guard', **kwargs)
        self.s = settings(self)
        self.inputs = Inputs(self, self.s)
        self.detector = ProximityDetector(self.s)
        self.frame_at = -math.inf
        self.supervision = SupervisorView(self)
        self.health = self.create_publisher(ComponentHealth, 'system/component_health', 10)
        self.command, self.command_at = (0., 0., 0.), -math.inf
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.status = self.create_publisher(String, 'proximity/status', 1)
        self.create_subscription(Twist, 'proximity/input', self.on_command, 1)
        self.graph_ok = False
        steady_timer(self, .2, self.check_graph)
        steady_timer(self, 1/30, self.tick)

    def check_graph(self):
        # launch 구성 실수로 두 번째 최종 속도 소유자가 생겼는지 검출합니다.
        self.graph_ok = self.count_publishers('cmd_vel') == 1 and self.count_publishers('proximity/input') == 1

    def on_command(self, msg):
        self.command, self.command_at = velocity(msg), time.monotonic()

    def tick(self):
        output, reason = (0., 0., 0.), 'COMMAND_STALE'
        healthy, detail = False, 'TOF_UNAVAILABLE'
        try:
            observation, age, measured = self.inputs.snapshot()
            if self.frame_at != self.inputs.frame_at:
                self.detector.update(observation, self.inputs.frame_at)
                self.frame_at = self.inputs.frame_at
            healthy = self.s.calibrated and observation.available and self.graph_ok
            detection = self.detector.result
            detail = json.dumps(dict(available=healthy, zones=observation.counts(),
                obstacle_zones=len(detection.points), pending_zones=detection.pending_zones,
                emergency=detection.emergency, held=detection.held))
            if not self.graph_ok:
                reason = 'COMMAND_GRAPH_CONFLICT'
            elif not self.supervision.operable():
                reason = 'SUPERVISOR_UNAVAILABLE'
            elif time.monotonic()-self.command_at <= self.s.command_timeout_s:
                output, reason = limit_velocity(self.command, measured, detection, age, self.s)
        except ValueError as exc:
            self.detector.reset()
            self.frame_at = -math.inf
            reason = detail = str(exc)
        self.pub.publish(twist(output))
        self.status.publish(String(data=reason))
        report = ComponentHealth(source='tof_proximity', status=ComponentHealth.AVAILABLE if healthy else ComponentHealth.UNAVAILABLE, detail=detail)
        report.header.stamp = self.get_clock().now().to_msg()
        self.health.publish(report)


def main():
    rclpy.init()
    node = ProximityGuard()
    try:
        rclpy.spin(node)
    finally:
        node.pub.publish(twist())
        node.destroy_node()
        rclpy.shutdown()
