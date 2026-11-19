"""기존 Nav2 lifecycle manager에 위임하는 컴포넌트 소유자 측 Supervisor 복구입니다."""
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
    """lifecycle manager 하나를 RESET 후 STARTUP해 RecoverComponent 목표를 처리합니다.

    Actions:
        navigation/recover, localization/recover (cleany_interfaces/action/RecoverComponent):
            Server이며 actions 기본값입니다. 모든 action을 통틀어 목표 하나만 처리합니다.

    Services:
        lifecycle_manager_navigation/manage_nodes,
        lifecycle_manager_localization/manage_nodes (nav2_msgs/srv/ManageLifecycleNodes):
            action과 대응하는 Client이며 lifecycle_managers 기본값입니다.

    Notes:
        reset은 lifecycle manager가 수행하며 이 노드는 프로세스를 재시작하거나
        Nav2 BT 복구를 호출하지 않습니다. 취소는 단계 사이에 적용합니다.
        성공은 manager의 성공 보고만 뜻하며 Supervisor가 이후 최신 lifecycle과
        위치 추정 상태로 복구를 검증합니다.
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
        # Supervisor 재시도를 포함해 어느 lifecycle manager든 충돌하는 복구를 거부합니다.
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
            # 성공은 lifecycle manager의 성공 보고이며 상태 확인은 Supervisor가 담당합니다.
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
