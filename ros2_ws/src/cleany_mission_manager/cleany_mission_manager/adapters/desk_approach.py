"""Journaled DeskApproach client; uncertain termination cannot certify a stop."""
import time
from uuid import uuid4
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from rclpy.action import ActionClient
from cleany_interfaces.action import DeskApproach
from cleany_mission_manager.core.result import FailureCode, ModuleResult, ResultStatus
from .goals import cancel_request, goal_uuid


class DeskApproachPort:
    def __init__(self, node, journal, base_stopped, distance, supervision=None):
        self.supervision = supervision
        self.client = ActionClient(node, DeskApproach, 'desk/approach')
        self.cancel_client = node.create_client(CancelGoal, 'desk/approach/_action/cancel_goal')
        self.journal, self.base_stopped, self.distance = journal, base_stopped, distance
        self.recovery_goal = journal.value('desk_goal_uuid')
        self.recovery_future = None
        self.recovery_at = 0.
        self.operation = None
        self.handle = self.result = None
        self.terminal = True
        self.cancelled = False

    def maintain(self):
        if not self.recovery_goal:
            return
        now = time.monotonic()
        if self.recovery_future is not None and not self.recovery_future.done() and now-self.recovery_at > 3.:
            self.recovery_future.cancel()
            self.recovery_future = None
        if self.recovery_future is None and self.cancel_client.service_is_ready():
            self.recovery_future = self.cancel_client.call_async(cancel_request(self.recovery_goal))
            self.recovery_at = now
        if self.recovery_future is not None and self.recovery_future.done():
            try:
                response = self.recovery_future.result()
                # An accepted cancel is not completed cancellation. Retry until the
                # server reports the goal terminal/unknown AND odometry is stopped.
                if response.return_code in (CancelGoal.Response.ERROR_GOAL_TERMINATED,
                                            CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID) and self.base_stopped():
                    self.recovery_goal = ''
                    self.journal.set('desk_goal_uuid', '')
            except Exception:
                pass
            self.recovery_future = None

    def ready(self):
        return (True, '') if not self.recovery_goal and self.client.server_is_ready() else (False, 'DESK_APPROACH_UNAVAILABLE')

    def stopped(self):
        return not self.recovery_goal and self.terminal and self.base_stopped()

    def start(self, _target):
        if not self.ready()[0] or not self.stopped():
            raise RuntimeError('desk approach unavailable')
        self.operation = str(uuid4())
        self.handle = self.result = None
        self.terminal, self.cancelled = False, False
        self.journal.set('desk_goal_uuid', self.operation)
        goal = DeskApproach.Goal(distance_m=self.distance)
        if self.supervision is not None:
            goal.mission_id = self.supervision.mission_id
            goal.lease_id = self.supervision.lease_id
            goal.lease_epoch = self.supervision.lease_epoch
            goal.lease_version = self.supervision.version
        future = self.client.send_goal_async(goal, goal_uuid=goal_uuid(self.operation))
        future.add_done_callback(self.accepted)
        return self.operation

    def accepted(self, future):
        try:
            self.handle = future.result()
            if not self.handle.accepted:
                self.terminal = True
                self.result = ModuleResult.failed(FailureCode.NAVIGATION_FAIL, message='Desk approach rejected')
                return
            self.handle.get_result_async().add_done_callback(self.finished)
            if self.cancelled:
                self.handle.cancel_goal_async()
        except Exception as exc:
            self.result = ModuleResult.fatal(FailureCode.NAVIGATION_FAIL, message=str(exc))

    def finished(self, future):
        try:
            response = future.result()
            self.terminal = response.status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)
            if response.status == GoalStatus.STATUS_SUCCEEDED and response.result.success and not self.cancelled:
                self.result = ModuleResult.success(data={'desk_distance_m': response.result.distance_m,
                    'desk_yaw_error_rad': response.result.yaw_error_rad}, message='Desk approach stopped and settled')
            elif response.status == GoalStatus.STATUS_CANCELED or self.cancelled:
                self.result = ModuleResult(False, ResultStatus.CANCELLED)
            else:
                self.result = ModuleResult.failed(FailureCode.NAVIGATION_FAIL, message=response.result.reason)
        except Exception as exc:
            self.result = ModuleResult.fatal(FailureCode.NAVIGATION_FAIL, message=str(exc))

    def poll(self, operation_id):
        if operation_id != self.operation or self.result is None:
            return None
        if not self.terminal or not self.base_stopped():
            return None
        self.journal.set('desk_goal_uuid', '')
        return self.result

    def cancel(self, operation_id):
        if operation_id == self.operation:
            self.cancelled = True
            if self.handle and self.handle.accepted:
                self.handle.cancel_goal_async()
            elif self.cancel_client.service_is_ready():
                self.cancel_client.call_async(cancel_request(operation_id))
