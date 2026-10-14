"""ROS Action/Service wrapper for the mock manipulation execution core."""

from __future__ import annotations

from dataclasses import asdict, fields
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from action_msgs.srv import CancelGoal
from cleany_interfaces.action import ExecuteManipulationSkill
from cleany_interfaces.msg import ManipulationExecutionRecord
from cleany_interfaces.srv import CancelManipulation, GetManipulationExecution
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future

from .manipulation.core import ExecutionCore
from .manipulation.mock import MockAdapter, MockConfig
from .manipulation.models import Goal, Record, Stage, Status, resolve_cancel_mode
from .manipulation.store import ExecutionStore, StoreError, default_database_path


def goal_from_message(message: ExecuteManipulationSkill.Goal) -> Goal:
    return Goal(**{field.name: getattr(message, field.name) for field in fields(Goal)})


def result_message(record: Record) -> ExecuteManipulationSkill.Result:
    return ExecuteManipulationSkill.Result(**asdict(record.result))


def record_message(record: Record) -> ManipulationExecutionRecord:
    values = asdict(record.goal)
    values.update({field.name: getattr(record, field.name) for field in fields(Record)
                   if field.name not in ('goal', 'result')})
    if record.result is not None:
        values.update(asdict(record.result))
    values['has_result'] = record.result is not None
    return ManipulationExecutionRecord(**values)


class ManipulationNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__('manipulation_server', **kwargs)
        self.declare_parameter('backend', 'mock')
        self.declare_parameter('database_path', default_database_path())
        self.declare_parameter('mock_config', str(Path(get_package_share_directory(
            'cleany_skill_executor')) / 'config/manipulation_mock.yaml'))
        self.declare_parameter('scenario', 'success')
        if self.get_parameter('backend').value != 'mock':
            raise ValueError('This server supports only backend=mock')
        config = MockConfig.load(self.get_parameter('mock_config').value)
        port = MockAdapter(config, self.get_parameter('scenario').value)
        self.store = ExecutionStore(self.get_parameter('database_path').value)
        try:
            self.core = ExecutionCore(port, self.store, config.timeout)
        except Exception:
            self.store.close()
            raise
        self._handle = None
        self._result_future: Future | None = None
        self._futures: dict[str, Future] = {}
        group = ReentrantCallbackGroup()
        self._events = self.create_publisher(
            ManipulationExecutionRecord, 'manipulation/execution_events',
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._service = self.create_service(GetManipulationExecution,
                                            'manipulation/get_execution', self._get,
                                            callback_group=group)
        self._action = ActionServer(
            self, ExecuteManipulationSkill, 'manipulation/execute_skill',
            execute_callback=self._execute, goal_callback=self._goal,
            handle_accepted_callback=self._accepted, cancel_callback=self._cancel,
            callback_group=group)
        self._setup_cancel_service(group)
        # Steady clock also works when ROS use_sim_time is paused or unset.
        self._timer = self.create_timer(0.05, self._tick, callback_group=group,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._publish_events()
        self.get_logger().info(f'Mock manipulation ready; database={self.store.path}; '
                               f'inhibited={self.core.inhibited}')

    def _goal(self, request) -> GoalResponse:
        with self.core.lock:
            if self._handle is not None:
                return GoalResponse.REJECT
            accepted, reason = self.core.accept(goal_from_message(request))
            if not accepted:
                self.get_logger().warning(f'Goal rejected: {reason}')
            return GoalResponse.ACCEPT if accepted else GoalResponse.REJECT

    def _accepted(self, handle) -> None:
        with self.core.lock:
            self._handle = handle
            self._result_future = Future(executor=self.executor)
            self._futures[handle.request.execution_id] = self._result_future
            handle.execute()

    async def _execute(self, handle) -> ExecuteManipulationSkill.Result:
        # Suspending a coroutine leaves executor threads free for cancels/timers.
        with self.core.lock:
            future = self._futures[handle.request.execution_id]
        try:
            return await future
        finally:
            with self.core.lock:
                self._futures.pop(handle.request.execution_id, None)

    def _cancel(self, handle) -> CancelResponse:
        with self.core.lock:
            execution_id = handle.request.execution_id
            mode = self._cancel_modes.pop(execution_id, None)
            accepted = self.core.request_cancel(execution_id, mode)
            return CancelResponse.ACCEPT if accepted else CancelResponse.REJECT

    def _setup_cancel_service(self, group: ReentrantCallbackGroup) -> None:
        self._cancel_modes: dict[str, str] = {}
        self._cancel_client = self.create_client(
            CancelGoal, 'manipulation/execute_skill/_action/cancel_goal', callback_group=group)
        self._cancel_service = self.create_service(
            CancelManipulation, 'manipulation/cancel', self._request_cancel, callback_group=group)

    async def _request_cancel(self, request: CancelManipulation.Request,
                              response: CancelManipulation.Response) -> CancelManipulation.Response:
        # Forward through the standard Action service so the goal really enters
        # CANCELING before we report its terminal ROS status. Never set that
        # state through rclpy's private GoalHandle API.
        with self.core.lock:
            handle, record = self._handle, self.core.record
            if (handle is None or record is None or record.result is not None
                    or record.stage == Stage.FINALIZING
                    or record.goal.execution_id != request.execution_id):
                response.message = 'No matching active execution'
                return response
            try:
                mode = resolve_cancel_mode(
                    record.cancel_mode or self._cancel_modes.get(request.execution_id, ''), request.mode)
            except ValueError as error:
                response.message = str(error)
                return response
            if handle.is_cancel_requested:
                response.accepted = self.core.request_cancel(request.execution_id, mode)
                response.message = f'Cancellation mode: {mode.value}'
                return response
            self._cancel_modes[request.execution_id] = mode.value
            forwarded = CancelGoal.Request()
            forwarded.goal_info.goal_id = handle.goal_id
            future = self._cancel_client.call_async(forwarded)
        reply = await future
        with self.core.lock:
            response.accepted = bool(reply is not None and reply.goals_canceling)
            if not response.accepted and self._handle is handle and handle.is_cancel_requested:
                # A concurrent request may have performed the ROS transition.
                response.accepted = self.core.request_cancel(request.execution_id, mode)
            self._cancel_modes.pop(request.execution_id, None)
            response.message = (f'Cancellation mode: {mode.value}' if response.accepted
                                else 'Action cancellation was rejected')
        return response

    def _get(self, request, response):
        with self.core.lock:
            try:
                record = self.core.get(request.execution_id)
            except StoreError as exc:
                self.core.inhibited = True
                self.get_logger().error(f'Execution lookup failed: {exc}')
                record = None
            response.found = record is not None
            if record is not None:
                response.record = record_message(record)
        return response

    def _publish_events(self) -> None:
        for record in self.core.drain_events():
            self._events.publish(record_message(record))
            if self._handle is not None and record.goal.execution_id == self._handle.request.execution_id:
                self._handle.publish_feedback(ExecuteManipulationSkill.Feedback(
                    execution_id=record.goal.execution_id, stage=record.stage.value,
                    substage=record.substage,
                    selected_arm=record.selected_arm, message=record.message))

    def _tick(self) -> None:
        with self.core.lock:
            if self._handle is not None:
                self.core.tick()
            self._publish_events()
            record = self.core.record
            if self._handle is None or record.result is None:
                return
            status = record.result.status
            # rclpy transitions to CANCELING just after our cancel callback returns.
            # Wait for that transition before issuing CANCELED, under the same lock.
            if status == Status.CANCELED and not self._handle.is_cancel_requested:
                return
            if status in (Status.SUCCESS, Status.BLOCKED):
                self._handle.succeed()
            elif status == Status.CANCELED:
                self._handle.canceled()
            else:
                self._handle.abort()
            self._result_future.set_result(result_message(record))
            self._handle, self._result_future = None, None

    def destroy_node(self):
        self._timer.cancel()
        self._action.destroy()
        # An active record remains ACTIVE for recovery; shutdown is not stop evidence.
        self.store.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = ManipulationNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
