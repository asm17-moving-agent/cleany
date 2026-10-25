"""Final ToF safety limit after the existing Nav2 safety chain."""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cleany_interfaces.msg import ComponentHealth
from .core import limit_velocity, coverage
from .ros_support import Inputs, SupervisorView, settings, steady_timer, twist, velocity


class ProximityGuard(Node):
    """Consume proximity/input and publish cmd_vel at 30 Hz (Twist, depth 1).

    tof/frame and odom must be fresh. proximity/status (String, depth 1) reports
    local limiting reasons only; this node owns no Mission or Supervisor state.
    The motor driver's independent command watchdog covers process death.
    """
    def __init__(self, **kwargs):
        super().__init__('proximity_guard', **kwargs)
        self.s = settings(self)
        self.inputs = Inputs(self, self.s)
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
        # Detect accidental launch composition with a second final velocity owner.
        self.graph_ok = self.count_publishers('cmd_vel') == 1 and self.count_publishers('proximity/input') == 1

    def on_command(self, msg):
        self.command, self.command_at = velocity(msg), time.monotonic()

    def tick(self):
        output, reason = (0., 0., 0.), 'COMMAND_STALE'
        healthy, detail = False, 'TOF_UNAVAILABLE'
        try:
            points, good, age, measured = self.inputs.snapshot()
            healthy = self.s.calibrated and coverage(good, self.s) and self.graph_ok
            detail = 'ToF quality, calibration, odometry and command graph verified' if healthy else 'ToF calibration, coverage or graph unavailable'
            if not self.graph_ok:
                reason = 'COMMAND_GRAPH_CONFLICT'
            elif not self.supervision.operable():
                reason = 'SUPERVISOR_UNAVAILABLE'
            elif time.monotonic()-self.command_at <= self.s.command_timeout_s:
                output, reason = limit_velocity(self.command, measured, points, good, age, self.s)
        except ValueError as exc:
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
