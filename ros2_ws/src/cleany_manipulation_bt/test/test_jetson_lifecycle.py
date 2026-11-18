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
from control_msgs.msg import DynamicJointState, InterfaceValue
from rclpy.node import Node
from rclpy.parameter import Parameter
import zmq

from action_msgs.msg import GoalStatus, GoalStatusArray
from cleany_interfaces.srv import GetManipulationExecution
from std_srvs.srv import Trigger
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
        for status, busy in ((GoalStatus.STATUS_EXECUTING, True), (GoalStatus.STATUS_SUCCEEDED, False)):
            server._controller_status(('left', 'arm'), GoalStatusArray(status_list=[GoalStatus(status=status)]))
            snapshot = server._control_state(Trigger.Request(), Trigger.Response())
            assert f'"external_motion_active": {str(busy).lower()}' in snapshot.message
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
        guard = NS(fault='', max_age=.25, last_stamp=None,
                   cad_coordinates=None, expected_joint_names=EXPECTED_JOINTS,
                   require_driver_fault=True,
                   publisher=publisher, get_clock=source.get_clock,
                   latch=lambda reason: pytest.fail(reason))
        guard.feedback_is_fresh = lambda stamp: ControlStateGuard.feedback_is_fresh(guard, stamp)
        def publish():
            stamp = source.get_clock().now().nanoseconds
            names = sorted(EXPECTED_JOINTS)
            rows = [InterfaceValue(interface_names=['position'], values=[0.])
                    if name.startswith('head_') else InterfaceValue(
                        interface_names=['position', 'feedback_stamp_ns', 'hardware_fault'],
                        values=[0., float(stamp), 0.]) for name in names]
            ControlStateGuard.receive_hardware(guard, DynamicJointState(
                joint_names=names, interface_values=rows))
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


def test_completion_notifications_finish_native_tree_without_periodic_ticks(tmp_path):
    from cleany_manipulation_bt.backend import Observation, WorkerBackend
    from cleany_skill_executor.manipulation.models import ObjectState, Placement
    from cleany_interfaces.action import ExecuteManipulationSkill
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor

    values = {
        'SelectArmAndPath': dict(selected_arm='right'),
        'ConfirmGrasp': dict(object_state=ObjectState.HELD),
        'ConfirmRelease': dict(object_state=ObjectState.LEFT_GRIPPER, placement_state=Placement.CONFIRMED),
        'ReturnArm': dict(arm_recovered=True),
        'VerifyPlacedObject': dict(placement_state=Placement.CONFIRMED, stop_confirmed=True),
    }
    context = Context()
    rclpy.init(context=context)
    backend = WorkerBackend(lambda name, _: Observation(**values.get(name, {})))
    server = JetsonManipulationNode(context=context, backend=backend,
        parameter_overrides=[Parameter('database_path', value=str(tmp_path/'wake.sqlite3')),
                             Parameter('monitor_port', value=free_port())])
    server._timer.cancel()
    server.monitor.update = lambda *args, **kw: pytest.fail('UI updated from completion wakeup')
    client_node = Node('completion_test', context=context)
    client = ActionClient(client_node, ExecuteManipulationSkill, 'manipulation/execute_skill')
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(server)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    def wait(future):
        deadline = time.monotonic()+5.
        while not future.done() and time.monotonic() < deadline:
            time.sleep(.005)
        assert future.done()
        return future.result()
    try:
        assert client.wait_for_server(timeout_sec=5.)
        handle = wait(client.send_goal_async(ExecuteManipulationSkill.Goal(
            mission_id='m', task_id='t', execution_id='wake', skill_name='collect_trash',
            snapshot_id='s', object_id=1, destination_id='trash_right')))
        assert handle.accepted
        wrapped = wait(handle.get_result_async())
        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
        assert wrapped.result.stop_confirmed
        assert server.store.get('wake').result is not None
    finally:
        executor.shutdown()
        thread.join(timeout=3.)
        server.destroy_node()
        client_node.destroy_node()
        context.shutdown()
