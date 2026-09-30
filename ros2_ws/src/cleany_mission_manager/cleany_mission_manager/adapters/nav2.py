"""NavigateToPose adapter with goal identity, cancellation and stopped feedback."""

import math
import time
from dataclasses import dataclass
from uuid import UUID, uuid4

from action_msgs.msg import GoalStatus, GoalInfo, GoalStatusArray
from action_msgs.srv import CancelGoal
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, qos_profile_action_status_default
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from unique_identifier_msgs.msg import UUID as GoalUUID

from cleany_mission_manager.core.journal import MissionJournal
from cleany_mission_manager.core.result import FailureCode, ModuleResult, ResultStatus

from .config import TargetMap


@dataclass
class NavigationOperation:
    operation_id: str
    handle: object = None
    result_future: object = None
    cancelled: bool = False
    result: ModuleResult | None = None
    result_received_at: float = 0.0
    terminal_known: bool = False


class Nav2Navigator:
    def __init__(self, node, targets: TargetMap, journal: MissionJournal) -> None:
        self.node, self.targets, self.journal = node, targets, journal
        self.client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self.cancel_client = node.create_client(CancelGoal, "navigate_to_pose/_action/cancel_goal")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.operation: NavigationOperation | None = None
        self.latest_odom = None
        self.odom_received = 0.0
        self.active_nodes = {}
        self.state_received = {}
        self.query_started = {}
        self.state_queries = {}
        self.state_clients = {
            name: node.create_client(GetState, name + "/get_state") for name in
            ("map_server", "amcl", "planner_server", "controller_server", "bt_navigator")
        }
        self.last_query = 0.0
        self.recovery_future = None
        self.recovery_started = 0.0
        self.recovery_goal = journal.value("nav_goal_uuid")
        self.odom_topic = node.get_parameter("odom_topic").value
        self.input_timeout = float(node.get_parameter("input_timeout").value)
        self.stop_linear = float(node.get_parameter("stopped_linear_speed").value)
        self.stop_angular = float(node.get_parameter("stopped_angular_speed").value)
        self.settle_timeout = float(node.get_parameter("navigation_settle_timeout").value)
        self.lifecycle_timeout = float(node.get_parameter("lifecycle_timeout").value)
        if (any(not math.isfinite(v) or v <= 0 for v in (
                self.input_timeout, self.settle_timeout, self.lifecycle_timeout,
        )) or any(not math.isfinite(v) or v < 0 for v in (self.stop_linear, self.stop_angular))):
            raise ValueError("navigation freshness/stopped parameters must be finite and bounded")
        node.create_subscription(Odometry, self.odom_topic, self._odom, qos_profile_sensor_data)
        node.create_subscription(GoalStatusArray, "navigate_to_pose/_action/status",
                                 self._status, qos_profile_action_status_default)

    def _status(self, message) -> None:
        if self.operation is None:
            return
        for status in message.status_list:
            if (str(UUID(bytes=bytes(status.goal_info.goal_id.uuid))) == self.operation.operation_id
                    and status.status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                                          GoalStatus.STATUS_ABORTED)):
                self.operation.terminal_known = True

    def _odom(self, message: Odometry) -> None:
        self.latest_odom = message
        self.odom_received = time.monotonic()

    def maintain(self) -> None:
        now = time.monotonic()
        if now - self.last_query >= 1.0:
            self.last_query = now
            for name, client in self.state_clients.items():
                if name in self.state_queries and not self.state_queries[name].done():
                    if now - self.query_started[name] < self.lifecycle_timeout:
                        continue
                    self.state_queries[name].cancel()
                    self.active_nodes[name] = False
                if client.service_is_ready():
                    future = client.call_async(GetState.Request())
                    self.state_queries[name] = future
                    self.query_started[name] = now
                    future.add_done_callback(lambda f, key=name: self._state(key, f))
                else:
                    self.active_nodes[name] = False
        if self.recovery_goal and self.recovery_future is None and self.cancel_client.service_is_ready():
            goal = GoalInfo(goal_id=GoalUUID(uuid=list(UUID(self.recovery_goal).bytes)))
            self.recovery_future = self.cancel_client.call_async(CancelGoal.Request(goal_info=goal))
            self.recovery_started = now
        if (self.recovery_future and not self.recovery_future.done()
                and now - self.recovery_started > self.lifecycle_timeout):
            self.recovery_future.cancel()
            self.recovery_future = None
        if (self.recovery_future and self.recovery_future.done()
                and self._base_stopped()):
            try:
                response = self.recovery_future.result()
            except Exception:
                self.recovery_future = None
                return
            if response.return_code in (CancelGoal.Response.ERROR_NONE,
                                        CancelGoal.Response.ERROR_GOAL_TERMINATED,
                                        CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID):
                self.recovery_goal = ""
                self.journal.set("nav_goal_uuid", "")
            else:
                self.recovery_future = None

    def _state(self, name, future) -> None:
        if self.state_queries.get(name) is not future:
            return
        try:
            self.active_nodes[name] = future.result().current_state.id == 3
            self.state_received[name] = time.monotonic()
        except Exception:
            self.active_nodes[name] = False

    def ready(self) -> tuple[bool, str]:
        if self.recovery_goal:
            return False, "INTERRUPTED_NAVIGATION"
        if not self.client.server_is_ready() or not all(
            self.active_nodes.get(n) and time.monotonic() - self.state_received.get(n, 0) <= self.lifecycle_timeout
            for n in self.state_clients
        ):
            return False, "NAV2_NOT_READY"
        if time.monotonic() - self.odom_received > self.input_timeout:
            return False, "ODOMETRY_STALE"
        try:
            transform = self.buffer.lookup_transform(self.targets.frame_id, "base_link", Time())
            stamp = transform.header.stamp
            age = self.node.get_clock().now().nanoseconds * 1e-9 - (stamp.sec + stamp.nanosec * 1e-9)
            if abs(age) > self.input_timeout:
                return False, "LOCALIZATION_STALE"
        except Exception:
            return False, "LOCALIZATION_UNAVAILABLE"
        return True, ""

    def start(self, target: object) -> str:
        if self.operation and self.operation.result is None:
            raise RuntimeError("navigation already running")
        if not self.ready()[0] or not self.stopped():
            raise RuntimeError("navigation or stopped feedback unavailable")
        pose = self.targets.home if target == "home" else self.targets.seats[str(target)]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.targets.frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose.x, pose.y
        goal.pose.pose.orientation.z = math.sin(pose.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(pose.yaw / 2)
        operation = NavigationOperation(str(uuid4()))
        self.operation = operation
        # The goal UUID is durable before it reaches Nav2. On restart cancel only this goal.
        self.journal.set("nav_goal_uuid", operation.operation_id)
        future = self.client.send_goal_async(
            goal, goal_uuid=GoalUUID(uuid=list(UUID(operation.operation_id).bytes))
        )
        future.add_done_callback(lambda f: self._accepted(operation, f))
        return operation.operation_id

    def _accepted(self, operation, future) -> None:
        try:
            handle = future.result()
            operation.handle = handle
            if not handle.accepted:
                operation.terminal_known = True
                operation.result = ModuleResult.failed(FailureCode.NAVIGATION_FAIL,
                                                      message="Nav2 rejected the goal.")
                operation.result_received_at = time.monotonic()
                return
            operation.result_future = handle.get_result_async()
            operation.result_future.add_done_callback(lambda f: self._finished(operation, f))
            # A pending acceptance may arrive after cancellation. It must never start new work.
            if operation.cancelled or self.operation is not operation:
                handle.cancel_goal_async()
        except Exception as exc:
            operation.result = ModuleResult.fatal(FailureCode.NAVIGATION_FAIL, message=str(exc))
            operation.result_received_at = time.monotonic()

    def _finished(self, operation, future) -> None:
        try:
            response = future.result()
            operation.terminal_known = response.status in (
                GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED,
            )
            if response.status == GoalStatus.STATUS_SUCCEEDED:
                result = ModuleResult.success(message="Nav2 arrived.")
            elif response.status == GoalStatus.STATUS_CANCELED:
                result = ModuleResult(False, ResultStatus.CANCELLED, message="Nav2 cancelled.")
            else:
                result = ModuleResult.failed(FailureCode.NAVIGATION_FAIL,
                                             message=f"Nav2 status {response.status}")
            operation.result = result
        except Exception as exc:
            operation.result = ModuleResult.fatal(FailureCode.NAVIGATION_FAIL, message=str(exc))
        operation.result_received_at = time.monotonic()

    def poll(self, operation_id: str) -> ModuleResult | None:
        operation = self.operation
        if operation is None or operation.operation_id != operation_id:
            return None  # Late results cannot advance another operation.
        if operation.result is None:
            return None
        if not operation.terminal_known or not self._base_stopped():
            if time.monotonic() - operation.result_received_at >= self.settle_timeout:
                return ModuleResult.fatal(FailureCode.NAVIGATION_FAIL,
                                          message="Navigation ended without fresh stopped feedback.")
            return None
        self.journal.set("nav_goal_uuid", "")
        return operation.result

    def cancel(self, operation_id: str) -> None:
        operation = self.operation
        if operation and operation.operation_id == operation_id:
            operation.cancelled = True
            if operation.handle and operation.handle.accepted and operation.result is None:
                operation.handle.cancel_goal_async()
            elif self.cancel_client.service_is_ready():
                goal = GoalInfo(goal_id=GoalUUID(uuid=list(UUID(operation_id).bytes)))
                self.cancel_client.call_async(CancelGoal.Request(goal_info=goal))

    def _base_stopped(self) -> bool:
        if self.latest_odom is None or time.monotonic() - self.odom_received > self.input_timeout:
            return False
        twist = self.latest_odom.twist.twist
        return (math.isfinite(twist.linear.x) and math.isfinite(twist.linear.y)
                and math.isfinite(twist.angular.z)
                and math.hypot(twist.linear.x, twist.linear.y) <= self.stop_linear
                and abs(twist.angular.z) <= self.stop_angular)

    def stopped(self) -> bool:
        return (not self.recovery_goal and self._base_stopped()
                and (self.operation is None or self.operation.result is not None
                     and self.operation.terminal_known))
