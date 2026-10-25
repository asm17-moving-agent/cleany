"""ROS clients for real inspection and independently measured drive readiness."""
import time

from cleany_interfaces.action import InspectScene
from rclpy.action import ActionClient
from std_srvs.srv import Trigger


class InspectionTransport:
    def __init__(self, node, action: str):
        self.client = ActionClient(node, InspectScene, action)

    def ready(self) -> bool:
        return self.client.server_is_ready()

    def send(self, session: str):
        return self.client.send_goal_async(InspectScene.Goal(tracking_session_id=session))


class DriveReadiness:
    """Expire successful checks; service discovery alone is never readiness."""
    def __init__(self, node, service: str, *, clock=time.monotonic):
        self.client = node.create_client(Trigger, service)
        self.clock = clock
        self.pending = None
        self.sent = self.received = float('-inf')
        self.safe, self.reason = False, 'MANIPULATION_READINESS_UNAVAILABLE'

    def __call__(self) -> tuple[bool, str]:
        now = self.clock()
        if self.pending is not None and self.pending.done():
            future, self.pending = self.pending, None
            try:
                response = future.result()
                self.safe, self.reason = response.success, response.message
                # Freshness starts at request, not receipt of a delayed response.
                self.received = self.sent
            except Exception:
                self.safe, self.reason = False, 'MANIPULATION_READINESS_FAILED'
        if self.pending is not None and now - self.sent > 1.:
            self.client.remove_pending_request(self.pending)
            self.pending.cancel()
            self.pending = None
        if self.pending is None and now - self.sent >= .2 and self.client.service_is_ready():
            self.sent = now
            self.pending = self.client.call_async(Trigger.Request())
        return ((self.safe, self.reason) if now - self.received < .8 else
                (False, 'MANIPULATION_READINESS_STALE'))
