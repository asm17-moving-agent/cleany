"""Cancel only the selected arm's observed active JTC goal, never its holding jaw."""
from copy import deepcopy
import time

from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile


class ControllerStop:
    def __init__(self, node):
        self.node = node
        self.statuses = {'left': [], 'right': []}
        self.clients = {}
        self.subscriptions = []
        for arm in self.statuses:
            action = f'/{arm}_arm_controller/follow_joint_trajectory/_action'
            self.clients[arm] = node.create_client(CancelGoal, f'{action}/cancel_goal')
            self.subscriptions.append(node.create_subscription(GoalStatusArray, f'{action}/status',
                lambda msg, side=arm: self.statuses.update({side: msg.status_list}),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))

    def stop(self, arm: str, result_future) -> None:
        node = self.node
        active = [s for s in self.statuses[arm] if s.status in (1, 2, 3)]
        deadline = time.monotonic()+2.
        while not active and not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            active = [s for s in self.statuses[arm] if s.status in (1, 2, 3)]
        if not active and not result_future.done():
            raise RuntimeError('Cannot confirm selected arm stop: no controller goal identity')
        ids = [bytes(s.goal_info.goal_id.uuid) for s in active]
        for status in active:
            request = CancelGoal.Request(goal_info=deepcopy(status.goal_info))
            if not any(request.goal_info.goal_id.uuid):
                raise RuntimeError('Refusing wildcard controller cancellation')
            response = node._future(self.clients[arm].call_async(request), 3., 'selected arm controller cancel')
            node.get_logger().error(f'ARM CONTROLLER cancel ack: arm={arm} code={response.return_code}')
        deadline = time.monotonic()+3.
        while ids and time.monotonic() < deadline:
            terminal = {bytes(s.goal_info.goal_id.uuid) for s in self.statuses[arm] if s.status in (4, 5, 6)}
            if set(ids) <= terminal:
                node.get_logger().error(f'ARM CONTROLLER terminal confirmed: arm={arm}')
                return
            rclpy.spin_once(node, timeout_sec=.02)
        if ids:
            raise RuntimeError('Selected arm controller did not confirm terminal state')
