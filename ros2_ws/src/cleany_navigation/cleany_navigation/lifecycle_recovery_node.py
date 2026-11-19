"""Owner-side Supervisor recovery that delegates to the existing Nav2 lifecycle managers."""
from threading import Event, Lock
from time import monotonic

import rclpy
from cleany_interfaces.action import RecoverComponent
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node


class LifecycleRecovery(Node):
    """Serve RecoverComponent goals by RESET then STARTUP of one lifecycle manager.

    Actions:
        navigation/recover, localization/recover (cleany_interfaces/action/RecoverComponent):
            Servers; actions default. One goal at a time across all actions.

    Services:
        lifecycle_manager_navigation/manage_nodes,
        lifecycle_manager_localization/manage_nodes (nav2_msgs/srv/ManageLifecycleNodes):
            Clients paired with actions; lifecycle_managers default.

    Notes:
        The lifecycle manager performs the reset; this node does not restart
        processes or call Nav2 BT recoveries. A cancel takes effect between steps.
        Success only means the manager reported success: the Supervisor verifies
        recovery with fresh lifecycle and localization health afterwards.
    """

    def __init__(self, **kwargs):
        super().__init__('nav2_lifecycle_recovery', **kwargs)
        self.timeout_s = self.declare_parameter('call_timeout_s', 10.).value
        self.group = ReentrantCallbackGroup()
        self.busy = Lock()
        self.servers = []
        actions = self.declare_parameter('actions', ['navigation/recover', 'localization/recover']).value
        managers = self.declare_parameter(
            'lifecycle_managers', ['lifecycle_manager_navigation', 'lifecycle_manager_localization']).value
        if len(actions) != len(managers):
            raise ValueError('actions and lifecycle_managers must have equal length')
        for action, manager in zip(actions, managers):
            client = self.create_client(ManageLifecycleNodes, manager + '/manage_nodes', callback_group=self.group)
            self.servers.append(ActionServer(
                self, RecoverComponent, action, lambda goal, client=client: self._execute(goal, client),
                goal_callback=self._goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                callback_group=self.group))

    def _goal(self, _request):
        # Conflicting recovery (any lifecycle manager) is rejected, including Supervisor retries.
        return GoalResponse.ACCEPT if self.busy.acquire(blocking=False) else GoalResponse.REJECT

    def _call(self, client, command):
        if not client.wait_for_service(timeout_sec=1.):
            return False, f'{client.srv_name} unavailable'
        done = Event()
        future = client.call_async(ManageLifecycleNodes.Request(command=command))
        future.add_done_callback(lambda _: done.set())
        if not done.wait(self.timeout_s):
            return False, f'{client.srv_name} command {command} timed out'
        response = future.result()
        return bool(response and response.success), f'{client.srv_name} command {command} ' + (
            'succeeded' if response and response.success else 'failed')

    def _execute(self, goal, client):
        try:
            detail = ''
            for command in (ManageLifecycleNodes.Request.RESET, ManageLifecycleNodes.Request.STARTUP):
                if goal.is_cancel_requested:
                    goal.canceled()
                    return RecoverComponent.Result(success=False, detail='canceled between lifecycle steps; '
                                                   + detail)
                goal.publish_feedback(RecoverComponent.Feedback(detail=f'lifecycle command {command}'))
                started = monotonic()
                ok, detail = self._call(client, command)
                self.get_logger().info(f'{detail} in {monotonic() - started:.2f}s')
                if not ok:
                    goal.abort()
                    return RecoverComponent.Result(success=False, detail=detail)
            goal.succeed()
            # Success means the lifecycle manager reported success; health is verified by the Supervisor.
            return RecoverComponent.Result(success=True, detail='lifecycle reset and startup reported success')
        finally:
            self.busy.release()


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleRecovery()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
