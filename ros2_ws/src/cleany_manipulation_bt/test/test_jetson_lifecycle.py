"""Construct the real physical adapter while idle; never submit robot Actions."""
from pathlib import Path
import socket
import threading
import time
from types import SimpleNamespace as NS
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from rclpy.node import Node
from rclpy.parameter import Parameter
import zmq
from cleany_interfaces.srv import GetManipulationExecution
from cleany_manipulation_bt.jetson_node import JetsonManipulationNode
from cleany_manipulation_bt.backend import ExecutionContext
from cleany_robot_interface.control_state_guard import ControlStateGuard, EXPECTED_JOINTS
from cleany_skill_executor.core.execution_ownership import ExecutionOwnership
from cleany_skill_executor.manipulation.models import Goal


def overrides(tmp_path, port):
    return [Parameter('collection_config', value=str(
        Path(__file__).resolve().parents[4] / 'ros2_ws/src/cleany_skill_executor/test/data/supervised_collection_fixture.json')),
        Parameter('database_path', value=str(tmp_path / 'executions.sqlite3')),
        Parameter('monitor_port', value=port)]


def free_port():
    with socket.socket() as available:
        available.bind(('127.0.0.1', 0))
        return available.getsockname()[1]


def test_physical_server_advertises_idle_query_without_submitting_motion(tmp_path):
    context = Context()
    rclpy.init(context=context)
    server = client = None
    executor = SingleThreadedExecutor(context=context)
    try:
        server = JetsonManipulationNode(context=context,
            parameter_overrides=overrides(tmp_path, free_port()))
        client = Node('idle_query_client', context=context)
        for node in (server, server.operations, client):
            executor.add_node(node)
        query = client.create_client(GetManipulationExecution, '/manipulation/get_execution')
        assert query.wait_for_service(timeout_sec=2.)
        future = query.call_async(GetManipulationExecution.Request(execution_id='not-started'))
        executor.spin_until_future_complete(future, timeout_sec=3.)
        assert future.done() and not future.result().found
        assert server.core.record is None
        assert not server.operations.submissions
        assert server.operations.scene_monitor.required
        with pytest.raises(RuntimeError, match='owns trajectory execution'):
            ExecutionOwnership()
    finally:
        executor.shutdown()
        if server is not None:
            server.destroy_node()
        if client is not None:
            client.destroy_node()
        context.shutdown()
    ExecutionOwnership().close()


@pytest.mark.parametrize('failure', ['port_in_use', 'invalid_stationary_samples'])
def test_failed_start_releases_physical_execution_ownership(tmp_path, failure):
    context = Context()
    rclpy.init(context=context)
    zmq_context = zmq.Context()
    occupied = zmq_context.socket(zmq.REP)
    occupied.setsockopt(zmq.LINGER, 0)
    port = occupied.bind_to_random_port('tcp://127.0.0.1')
    parameters = overrides(tmp_path, port)
    if failure == 'invalid_stationary_samples':
        parameters.append(Parameter('bt_stationary_samples', value=0))
    try:
        with pytest.raises((zmq.ZMQError, ValueError)):
            JetsonManipulationNode(context=context, parameter_overrides=parameters)
        ExecutionOwnership().close()
    finally:
        occupied.close()
        zmq_context.term()
        context.shutdown()


def test_actual_guard_position_output_can_confirm_idle_physical_stop(tmp_path):
    context = Context()
    rclpy.init(context=context)
    server = source = thread = None
    executor = SingleThreadedExecutor(context=context)
    try:
        server = JetsonManipulationNode(context=context,
            parameter_overrides=overrides(tmp_path, free_port()))
        source = Node('synthetic_position_broadcaster', context=context)
        publisher = source.create_publisher(JointState, '/joint_states', 10)
        guard = NS(fault='', max_age=.25, last_stamp=-1, last_received=None,
                   publisher=publisher, get_clock=source.get_clock,
                   latch=lambda reason: pytest.fail(reason))
        def publish():
            raw = JointState(name=sorted(EXPECTED_JOINTS), position=[0.] * 14,
                             velocity=[float('nan')] * 14)
            raw.header.stamp = source.get_clock().now().to_msg()
            ControlStateGuard.receive(guard, raw)
        source.create_timer(.02, publish)
        for node in (server, server.operations, source):
            executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        time.sleep(.1)
        execution = ExecutionContext(Goal('m', 't', 'idle', 'collect_trash', 's', 1, 'd'))
        assert server.operations._assess_stationary(execution, cancel=False)
        assert execution.stop_confirmed and not server.operations.submissions
    finally:
        executor.shutdown()
        if thread is not None:
            thread.join(timeout=3.)
        if server is not None:
            server.destroy_node()
        if source is not None:
            source.destroy_node()
        context.shutdown()
