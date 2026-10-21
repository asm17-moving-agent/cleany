"""Jetson Action wrapper. Uses the existing contract and a separate persistent journal."""
from __future__ import annotations

import os
import signal
import threading
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from cleany_interfaces.action import ExecuteManipulationSkill
from cleany_interfaces.msg import ManipulationExecutionRecord
from cleany_interfaces.srv import CancelManipulation, ConfirmManipulationRecovery, GetManipulationExecution
from cleany_skill_executor.manipulation_node import ManipulationNode, record_message
from cleany_skill_executor.manipulation.models import RecordState
from cleany_skill_executor.core.collection import temporary_return_mode
from .store import BTExecutionStore
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger
import json
import zmq

from ._bt_runner import Runner
from .backend import WorkerBackend
from .core import ExecutionCore
from .monitor import LiveMonitor
from .jetson_operations import JetsonOperations
from .arm_return_server import ArmReturnServer
from rclpy.parameter import Parameter


def default_database_path() -> str:
    state = Path(os.environ.get('XDG_STATE_HOME') or Path.home() / '.local/state')
    return str(state / 'cleany/manipulation_jetson/executions.sqlite3')


class JetsonManipulationNode(ManipulationNode):
    def __init__(self, *, backend=None, operations=None, **kwargs) -> None:
        self._closing = False
        Node.__init__(self, 'manipulation_jetson_server', **kwargs)
        self.operations, self.backend = operations, backend
        try:
            self._initialize(backend=backend, operations=operations, **kwargs)
        except Exception:
            # A failed bind or invalid config must release physical ownership
            # before another constructor is attempted in the same process.
            self.destroy_node()
            raise

    def _initialize(self, *, backend=None, operations=None, **kwargs) -> None:
        share = Path(get_package_share_directory('cleany_manipulation_bt'))
        self.declare_parameter('database_path', default_database_path())
        self.declare_parameter('tree_xml', str(share / 'trees/collect_trash_mujoco.xml'))
        self.declare_parameter('monitor_port', 1667)
        self.declare_parameter('bt_operation_timeout_sec', 180.0)
        self.declare_parameter('bt_selection_timeout_sec', 600.0)
        self.declare_parameter('bt_stop_timeout_sec', 10.0)
        self.declare_parameter('collection_config', '')
        self.declare_parameter('arm_return_pose_file', '')
        port = int(self.get_parameter('monitor_port').value)
        if not 1 <= port <= 65535:
            raise ValueError('monitor_port must be between 1 and 65535')
        self.operations = operations
        if backend is None:
            if operations is None:
                config = json.loads(Path(self.get_parameter('collection_config').value).expanduser().read_text())
                overrides = list(kwargs.get('parameter_overrides', []))
                overrides += [Parameter('require_sensor_scene', value=True),
                              Parameter('use_sim_time', value=False)]
                self.operations = JetsonOperations(config, context=self.context,
                                                    parameter_overrides=overrides)
            else:
                self.operations = operations
            backend = WorkerBackend(self.operations.execute, quiescent=self.operations.quiescent)
        self.backend = backend
        self.store = BTExecutionStore(self.get_parameter('database_path').value)
        self.core = ExecutionCore(self.backend, self.store, self.get_parameter('tree_xml').value, Runner,
                                  timeout_sec=float(self.get_parameter('bt_operation_timeout_sec').value),
                                  stop_timeout_sec=float(self.get_parameter('bt_stop_timeout_sec').value) + 2.,
                                  execution_profile='jetson',
                                  operation_timeouts={'SelectArmAndPath': float(
                                      self.get_parameter('bt_selection_timeout_sec').value)},
                                  allow_assumed_placement=temporary_return_mode(
                                      getattr(self.operations, 'config', {})))
        self._handle = self._result_future = None
        self._closing = False
        self._futures = {}
        group = ReentrantCallbackGroup()
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._events = self.create_publisher(ManipulationExecutionRecord, 'manipulation/execution_events', qos)
        self._bt_events = self.create_publisher(String, 'manipulation/bt_transitions', qos)
        self._service = self.create_service(GetManipulationExecution, 'manipulation/get_execution', self._get,
                                            callback_group=group)
        self._operator_state = self.create_service(Trigger, 'manipulation/control_state',
                                                   self._control_state, callback_group=group)
        self._recovery_service = self.create_service(ConfirmManipulationRecovery,
            'manipulation/confirm_recovery', self._confirm_recovery, callback_group=group)
        action_name = self.resolve_topic_name('manipulation/execute_skill')
        self._action = ActionServer(self, ExecuteManipulationSkill, action_name,
                                    execute_callback=self._execute, goal_callback=self._goal,
                                    handle_accepted_callback=self._accepted, cancel_callback=self._cancel,
                                    callback_group=group)
        self._setup_cancel_service(group, action_name)
        self.arm_return = ArmReturnServer(self, group, self.get_parameter('arm_return_pose_file').value)
        self.monitor = LiveMonitor()
        # Even the idle tree is a real native tree. It is never ticked or used
        # for motion until an accepted execution creates its own context.
        self.monitor.update(Runner(self.get_parameter('tree_xml').value, self.core, '').snapshot())
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.REP)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(f'tcp://127.0.0.1:{port}')
        self._monitor_execution = ''
        self._monitor_failed_execution = ''
        self._timer = self.create_timer(0.05, self._tick, callback_group=group,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._progress_timer = self.create_timer(1.0, self._publish_progress, callback_group=group,
                                                clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._publish_events()
        self.get_logger().info(f'Jetson BT ready; database={self.store.path}; monitor=127.0.0.1:{port}; '
                               f'inhibited={self.core.inhibited}')

    def _control_state(self, _request, response):
        """Report current executor ownership without commanding or clearing faults."""
        with self.core.lock:
            record = self.core.record
            active = record is not None and record.result is None and record.record_state == RecordState.ACTIVE
            operations = self.operations
            real = getattr(operations, 'real', None)
            guard = getattr(real, 'guard', None)
            guard_time = getattr(real, 'guard_time', None)
            max_age = real.value('hardware_guard_max_age_sec') if real is not None else 1.
            fresh = guard_time is not None and time.monotonic() - guard_time < max_age
            response.message = json.dumps(dict(schema=1,
                active_execution_id=record.goal.execution_id if active else '',
                active_stage=record.stage.value if active else '',
                active_substage=record.substage if active else '',
                arm_return_active=self.core.arm_return_active,
                recovery_active=self.core._recovery_in_progress,
                backend_ready=self.backend.ready(), inhibited=self.core.inhibited,
                blocked=[dict(execution_id=item.goal.execution_id, revision=item.revision)
                         for item in self.core.recovery_blocks()],
                guard=guard if fresh else None, guard_fresh=fresh,
                hardware_fault=getattr(real, 'fault', ''),
                payload_attached=getattr(operations, 'carrying', None) is not None,
                operation_mode=getattr(operations, 'config', {}).get('operation_mode', ''),
                last_result=record_message(record).status if record is not None and record.result is not None else '',
                last_stop_confirmed=record.stop_confirmed if record is not None and record.result is not None else False))
            response.success = True
        return response

    def _publish_progress(self) -> None:
        """Repeat active evidence for observers without advancing durable progress."""
        with self.core.lock:
            record = self.core.record
            if record is not None and record.result is None and record.record_state == RecordState.ACTIVE:
                self._events.publish(record_message(record))

    def _confirm_recovery(self, request, response):
        if not request.robot_stopped_confirmed and not request.gripper_empty_confirmed:
            response.success, response.message = True, 'Recovery status; no state changed'
        else:
            verify = getattr(self.operations, 'confirm_recovery_stationary', None)
            if verify is None:
                response.success, response.message = False, 'Physical stop verification unavailable'
            else:
                response.success, response.message = self.core.confirm_recovery(
                    request.execution_id, request.expected_revision,
                    stopped=request.robot_stopped_confirmed,
                    gripper_empty=request.gripper_empty_confirmed,
                    note=request.operator_note, verify_stationary=verify)
                if response.success:
                    self.get_logger().info(f'Recovery confirmed for {request.execution_id}: {response.message}')
        with self.core.lock:
            blocked = self.core.recovery_blocks()
            response.inhibited = self.core.inhibited
            response.blocked_execution_ids = [record.goal.execution_id for record in blocked]
            response.blocked_revisions = [record.revision for record in blocked]
        return response

    def _tick(self) -> None:
        with self.core.lock:
            if self._closing:
                return
            if self.core.record is not None:
                super()._tick()
                self._update_operation_progress()
            if self.core.tree is not None:
                execution = self.core.record.goal.execution_id
                if execution != self._monitor_failed_execution:
                    try:
                        self.monitor.update(self.core.tree.snapshot(), new_execution=execution != self._monitor_execution)
                    except Exception as error:
                        self._monitor_failed_execution = execution
                        self.get_logger().warning(f'BT monitor unavailable for {execution}: {error}')
                    else:
                        self._monitor_execution = execution
                for event in self.core.transitions:
                    self._bt_events.publish(String(data=json.dumps(dict(execution_id=execution, **event))))
                self.core.transitions.clear()
            for _ in range(10):
                try:
                    frames = self.socket.recv_multipart(flags=zmq.NOBLOCK)
                except zmq.Again:
                    break
                self.socket.send_multipart(self.monitor.reply(frames))

    def _update_operation_progress(self) -> None:
        record = self.core.record
        context = getattr(getattr(self, 'operations', None), 'execution_context', None)
        if (record is not None and record.result is None
                and record.substage == 'SelectArmAndPath' and context is not None
                and context.goal.execution_id == record.goal.execution_id
                and not context.abort.is_set() and context.progress
                and context.progress != record.message):
            self.core._save(message=context.progress)

    def destroy_node(self):
        core = getattr(self, 'core', None)
        if core is not None:
            with core.lock:
                self._closing = True
        arm_return = getattr(self, 'arm_return', None)
        if arm_return is not None:
            arm_return.destroy()
        timer, action, core = (getattr(self, name, None) for name in ('_timer', '_action', 'core'))
        if timer is not None:
            timer.cancel()
        progress_timer = getattr(self, '_progress_timer', None)
        if progress_timer is not None:
            progress_timer.cancel()
        if action is not None:
            action.destroy()
        # Shutdown only halts work; the ACTIVE record remains for recovery.
        if core is not None and core.tree is not None and core.record.result is None:
            core.tree.halt()
        if self.backend is not None:
            self.backend.close()
        if self.operations is not None:
            self.operations.destroy_node()
        for name, method in (('socket', 'close'), ('zmq_context', 'term'), ('store', 'close')):
            resource = getattr(self, name, None)
            if resource is not None:
                getattr(resource, method)()
        return Node.destroy_node(self)


def drain_shutdown(node: JetsonManipulationNode, executor: MultiThreadedExecutor) -> bool:
    """Keep DDS alive for IMMEDIATE cancellation and independent stop assessment."""
    with node.core.lock:
        node.core.inhibited = True
    arm_return = getattr(node, 'arm_return', None)
    if arm_return is not None:
        arm_return.request_shutdown()
        if node.core.arm_return_active:
            if not arm_return.drain(executor, node.core.stop_timeout_sec + 2.):
                return False
    with node.core.lock:
        record = node.core.record
        if record is None or record.result is not None:
            return True
        execution_id = record.goal.execution_id
    request = CancelManipulation.Request(execution_id=execution_id, mode='IMMEDIATE')
    deadline = time.monotonic() + node.core.stop_timeout_sec + 2.0
    task = receipt = None
    # The application future completing precedes rclpy dispatching its Result
    # responses. A standard GetResult round trip keeps those callbacks alive.
    result_client = node.create_client(ExecuteManipulationSkill.Impl.GetResultService,
                                       'manipulation/execute_skill/_action/get_result')
    try:
        while node.context.ok() and time.monotonic() < deadline:
            with node.core.lock:
                record = node.core.record
                if (record.result is not None and node._handle is None and not node._futures
                        and receipt is not None and receipt.done() and receipt.exception() is None):
                    return record.result.stop_confirmed
                if node._handle is not None:
                    if receipt is None:
                        receipt = result_client.call_async(ExecuteManipulationSkill.Impl.GetResultService.Request(
                            goal_id=node._handle.goal_id))
                    if task is None or (task.done() and (
                            task.exception() is not None or not task.result().accepted)):
                        task = executor.create_task(node._request_cancel, request, CancelManipulation.Response())
            executor.spin_once(timeout_sec=0.05)
        node.backend.abort()
        node.get_logger().error('Shutdown stop unconfirmed; inspect the execution journal and physical state')
        return False
    finally:
        node.destroy_client(result_client)


def main(args=None) -> None:
    # ROS's default handlers shut the context down before controller cancellation
    # can receive its acknowledgement or fresh feedback. Retain DDS until drained.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    shutdown = threading.Event()
    previous_handlers = {sig: signal.signal(sig, lambda *_: shutdown.set())
                         for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = JetsonManipulationNode()
        executor.add_node(node)
        if node.operations is not None:
            executor.add_node(node.operations)
        while rclpy.ok() and not shutdown.is_set():
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            try:
                if node is not None and rclpy.ok():
                    drain_shutdown(node, executor)
            finally:
                # launch may repeat signals while resources are being released.
                for sig in previous_handlers:
                    signal.signal(sig, signal.SIG_IGN)
                executor.shutdown()
                if node is not None:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
        finally:
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)
