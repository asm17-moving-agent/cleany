"""Real DDS traffic while the actual native BT waits on slow fake operations."""
import socket
import threading
import time
import uuid

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from cleany_interfaces.action import ExecuteManipulationSkill
from cleany_interfaces.srv import GetManipulationExecution
from cleany_manipulation_bt.backend import Observation, WorkerBackend
from cleany_manipulation_bt.node import MujocoManipulationNode
from cleany_skill_executor.manipulation.models import ObjectState, Placement


def wait(future):
    deadline = time.monotonic() + 8.
    while not future.done() and time.monotonic() < deadline:
        time.sleep(.01)
    assert future.done()
    return future.result()


@pytest.mark.parametrize('cancel', [False, True])
def test_dds_result_feedback_cancel_and_query(tmp_path, cancel):
    context = Context()
    rclpy.init(context=context)
    namespace = '/bt_test_' + uuid.uuid4().hex
    observed = []
    def operation(node, execution):
        observed.append(node)
        time.sleep(.03)  # Intentionally outlasts some BT ticks.
        values = {
            'SelectArmAndPath': dict(selected_arm='right'),
            'ConfirmGrasp': dict(object_state=ObjectState.HELD),
            'ConfirmRelease': dict(object_state=ObjectState.LEFT_GRIPPER, placement_state=Placement.CONFIRMED),
            'ReturnArm': dict(arm_recovered=True),
            'VerifyPlacedObject': dict(placement_state=Placement.CONFIRMED, stop_confirmed=True),
            'StopAndAssess': dict(stop_confirmed=True),
        }
        return Observation(**values.get(node, {}))
    backend = WorkerBackend(operation)
    with socket.socket() as available:
        available.bind(('127.0.0.1', 0))
        port = available.getsockname()[1]
    server = MujocoManipulationNode(context=context, namespace=namespace, backend=backend,
        parameter_overrides=[Parameter('database_path', value=str(tmp_path/'runtime.sqlite3')),
                             Parameter('monitor_port', value=port)])
    client_node = Node('client', context=context, namespace=namespace)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(server)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client = ActionClient(client_node, ExecuteManipulationSkill, 'manipulation/execute_skill')
    service = client_node.create_client(GetManipulationExecution, 'manipulation/get_execution')
    feedback = []
    try:
        assert client.wait_for_server(timeout_sec=5.)
        goal = ExecuteManipulationSkill.Goal(mission_id='m', task_id='t', execution_id=uuid.uuid4().hex,
            skill_name='collect_trash', snapshot_id='snapshot', object_id=1, destination_id='trash_right')
        handle = wait(client.send_goal_async(goal, feedback_callback=lambda event: feedback.append(event.feedback)))
        assert handle.accepted
        if cancel:
            deadline = time.monotonic() + 5.
            while not any(f.substage == 'GraspObject' for f in feedback) and time.monotonic() < deadline:
                time.sleep(.01)
            assert wait(handle.cancel_goal_async()).goals_canceling
        result = wait(handle.get_result_async()).result
        assert result.status == ('CANCELED' if cancel else 'SUCCESS')
        assert result.execution_profile == 'mujoco' and result.stop_confirmed
        response = wait(service.call_async(GetManipulationExecution.Request(execution_id=goal.execution_id)))
        assert response.found and response.record.has_result
        assert response.record.status == result.status
        assert any(f.substage == 'MoveToPregrasp' for f in feedback)
        if cancel:
            assert 'ConfirmGrasp' in observed and 'LiftObject' not in observed
        else:
            assert result.arm_recovered
            assert server.core.tree.snapshot()['nodes'][0]['status'] != 'RUNNING'
    finally:
        executor.shutdown(timeout_sec=5.)
        thread.join(timeout=5.)
        server.destroy_node()
        client_node.destroy_node()
        context.shutdown()
