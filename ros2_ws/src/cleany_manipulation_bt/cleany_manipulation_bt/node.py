"""MuJoCo Action wrapper. Uses the existing contract and a separate persistent journal."""
from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from cleany_interfaces.action import ExecuteManipulationSkill
from cleany_interfaces.msg import ManipulationExecutionRecord
from cleany_interfaces.srv import GetManipulationExecution
from cleany_skill_executor.manipulation_node import ManipulationNode
from .store import BTExecutionStore
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import json
import zmq

from ._bt_runner import Runner
from .backend import WorkerBackend
from .core import ExecutionCore
from .monitor import LiveMonitor
from .ros_operations import MujocoOperations


def default_database_path() -> str:
    state = Path(os.environ.get('XDG_STATE_HOME') or Path.home() / '.local/state')
    return str(state / 'cleany/manipulation_mujoco/executions.sqlite3')


class MujocoManipulationNode(ManipulationNode):
    def __init__(self, *, backend=None, operations=None, **kwargs) -> None:
        kwargs.setdefault('namespace', '/sim')
        Node.__init__(self, 'manipulation_mujoco_server', **kwargs)
        share = Path(get_package_share_directory('cleany_manipulation_bt'))
        self.declare_parameter('database_path', default_database_path())
        self.declare_parameter('tree_xml', str(share / 'trees/collect_trash_mujoco.xml'))
        self.declare_parameter('monitor_port', 1667)
        self.declare_parameter('bt_operation_timeout_sec', 180.0)
        self.declare_parameter('bt_stop_timeout_sec', 10.0)
        port = int(self.get_parameter('monitor_port').value)
        if not 1 <= port <= 65535:
            raise ValueError('monitor_port must be between 1 and 65535')
        self.operations = operations
        if backend is None:
            self.operations = operations or MujocoOperations()
            backend = WorkerBackend(self.operations.execute, quiescent=self.operations.quiescent)
        self.backend = backend
        self.store = BTExecutionStore(self.get_parameter('database_path').value)
        self.core = ExecutionCore(self.backend, self.store, self.get_parameter('tree_xml').value, Runner,
                                  timeout_sec=float(self.get_parameter('bt_operation_timeout_sec').value),
                                  stop_timeout_sec=float(self.get_parameter('bt_stop_timeout_sec').value) + 2.)
        self._handle = self._result_future = None
        self._futures = {}
        group = ReentrantCallbackGroup()
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._events = self.create_publisher(ManipulationExecutionRecord, 'manipulation/execution_events', qos)
        self._bt_events = self.create_publisher(String, 'manipulation/bt_transitions', qos)
        self._service = self.create_service(GetManipulationExecution, 'manipulation/get_execution', self._get,
                                            callback_group=group)
        self._action = ActionServer(self, ExecuteManipulationSkill, 'manipulation/execute_skill',
                                    execute_callback=self._execute, goal_callback=self._goal,
                                    handle_accepted_callback=self._accepted, cancel_callback=self._cancel,
                                    callback_group=group)
        self.monitor = LiveMonitor()
        # Even the idle tree is a real native tree. It is never ticked or used
        # for motion until an accepted execution creates its own context.
        self.monitor.update(Runner(self.get_parameter('tree_xml').value, self.core, '').snapshot())
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.REP)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(f'tcp://127.0.0.1:{port}')
        self._monitor_execution = ''
        self._timer = self.create_timer(0.05, self._tick, callback_group=group,
                                       clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._publish_events()
        self.get_logger().info(f'MuJoCo BT ready; database={self.store.path}; monitor=127.0.0.1:{port}; '
                               f'inhibited={self.core.inhibited}')

    def _tick(self) -> None:
        with self.core.lock:
            if self.core.record is not None:
                super()._tick()
            if self.core.tree is not None:
                execution = self.core.record.goal.execution_id
                self.monitor.update(self.core.tree.snapshot(), new_execution=execution != self._monitor_execution)
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

    def destroy_node(self):
        self._timer.cancel()
        self._action.destroy()
        # Shutdown only halts work; the ACTIVE record remains for recovery.
        if self.core.tree is not None and self.core.record.result is None:
            self.core.tree.halt()
        self.backend.close()
        if self.operations is not None:
            self.operations.destroy_node()
        self.socket.close()
        self.zmq_context.term()
        self.store.close()
        return Node.destroy_node(self)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = MujocoManipulationNode()
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
