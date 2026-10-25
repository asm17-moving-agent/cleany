"""Exclusive Nav2/desk command selection upstream of every existing safety stage."""
import math
import time
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from rclpy.task import Future
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from cleany_interfaces.action import DeskApproach
from .core import ApproachController, coverage, estimate_plane
from .ros_support import Inputs, SupervisorView, settings, steady_timer, twist, velocity


class DeskApproachNode(Node):
    """Serve desk/approach (DeskApproach) and select motion/cmd_vel (Twist).

    Nav2 commands arrive on nav2/cmd_vel. All selected commands enter the existing
    pan/Collision Monitor/3-D chain and final proximity guard. Active Nav2 goals
    exclude desk ownership. Cancellation, invalid input and timeout emit zero.
    Plane success includes sustained stopped odometry, not just a pose estimate.
    """
    def __init__(self, **kwargs):
        super().__init__('desk_approach', **kwargs)
        self.s = settings(self)
        self.inputs = Inputs(self, self.s)
        self.supervision = SupervisorView(self)
        self.pub = self.create_publisher(Twist, 'motion/cmd_vel', 1)
        self.nav_command, self.nav_at = (0., 0., 0.), -math.inf
        self.nav_active, self.nav_seen = False, False
        self.reserved = False
        self.goal_handle = self.controller = self.completion = None
        self.plane = None
        self.graph_ok = False
        self.create_subscription(Twist, 'nav2/cmd_vel', self.on_nav, 1)
        self.create_subscription(GoalStatusArray, 'navigate_to_pose/_action/status', self.on_status,
                                 qos_profile_action_status_default)
        self.server = ActionServer(self, DeskApproach, 'desk/approach', self.execute,
                                   goal_callback=self.goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   handle_accepted_callback=self.accepted)
        steady_timer(self, .2, self.check_graph)
        steady_timer(self, 1/30, self.tick)

    def check_graph(self):
        self.graph_ok = self.count_publishers('motion/cmd_vel') == 1

    def on_nav(self, msg):
        self.nav_command, self.nav_at = velocity(msg), time.monotonic()

    def on_status(self, msg):
        self.nav_seen = True
        self.nav_active = any(s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                                           GoalStatus.STATUS_CANCELING) for s in msg.status_list)

    def goal(self, request):
        if not self.supervision.authorized(request):
            return GoalResponse.REJECT
        if self.reserved or self.nav_active or not self.nav_seen or not self.graph_ok or not self.s.calibrated:
            return GoalResponse.REJECT
        try:
            ApproachController(request.distance_m, time.monotonic(), self.s)
            points, good, _, measured = self.inputs.snapshot()
            if not coverage(good, self.s) or math.hypot(*measured[:2]) > self.s.stopped_speed or abs(measured[2]) > self.s.stopped_yaw_rate:
                return GoalResponse.REJECT
            estimate_plane(points, good, self.s)
        except ValueError:
            return GoalResponse.REJECT
        self.reserved = True
        return GoalResponse.ACCEPT

    def accepted(self, handle):
        self.goal_handle = handle
        self.controller = ApproachController(handle.request.distance_m, time.monotonic(), self.s)
        self.completion = Future()
        self.plane = None
        self.nav_at = -math.inf
        handle.execute()

    async def execute(self, handle):
        reason, plane = await self.completion
        if reason == 'SUCCEEDED':
            handle.succeed()
        elif reason == 'CANCELED':
            handle.canceled()
        else:
            handle.abort()
        result = DeskApproach.Result(success=reason == 'SUCCEEDED', reason=reason)
        if plane:
            result.distance_m, result.yaw_error_rad = plane.distance, plane.yaw
        self.goal_handle = self.controller = self.completion = None
        self.reserved = False
        self.nav_at = -math.inf
        return result

    def finish(self, reason):
        self.pub.publish(twist())
        if self.completion and not self.completion.done():
            self.completion.set_result((reason, self.plane))

    def tick(self):
        if not self.reserved:
            command = self.nav_command if self.graph_ok and self.supervision.operable() and time.monotonic()-self.nav_at <= self.s.command_timeout_s else (0., 0., 0.)
            self.pub.publish(twist(command))
            return
        if self.goal_handle is None or self.completion.done():
            self.pub.publish(twist())
            return
        if self.goal_handle.is_cancel_requested:
            self.finish('CANCELED')
            return
        if not self.supervision.authorized(self.goal_handle.request):
            self.finish('SUPERVISOR_AUTHORITY_LOST')
            return
        if self.nav_active or not self.graph_ok:
            self.finish('COMMAND_OWNERSHIP_CONFLICT')
            return
        try:
            points, good, _, measured = self.inputs.snapshot()
            if not coverage(good, self.s):
                raise ValueError('TOF_COVERAGE_INVALID')
            self.plane = estimate_plane(points, good, self.s)
            command, state = self.controller.step(self.plane, measured, time.monotonic())
        except ValueError as exc:
            self.finish(str(exc))
            return
        self.pub.publish(twist(command))
        self.goal_handle.publish_feedback(DeskApproach.Feedback(
            distance_m=self.plane.distance, yaw_error_rad=self.plane.yaw, state=state))
        if state not in ('SETTLING', 'APPROACHING', 'ALIGNING'):
            self.finish(state)


def main():
    rclpy.init()
    node = DeskApproachNode()
    try:
        rclpy.spin(node)
    finally:
        node.pub.publish(twist())
        node.destroy_node()
        rclpy.shutdown()
