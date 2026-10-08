"""Real DDS traffic while the actual native BT waits on slow fake operations."""
import socket
import threading
import time
import uuid

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from cleany_interfaces.action import ExecuteManipulationSkill
from cleany_interfaces.srv import CancelManipulation, GetManipulationExecution
from cleany_manipulation_bt.backend import Observation, WorkerBackend
from cleany_manipulation_bt.node import MujocoManipulationNode
from cleany_skill_executor.manipulation.models import ObjectState, Placement
from cleany_skill_executor.manipulation.store import StoreError


def wait(future):
    deadline = time.monotonic() + 8.
    while not future.done() and time.monotonic() < deadline:
        time.sleep(.01)
    assert future.done()
    return future.result()


@pytest.mark.parametrize('cancel,mode,write_failure', [
    (False, None, 'none'),
    (True, None, 'none'),
    (True, 'RETURN_ARM', 'none'),
    (True, 'IMMEDIATE', 'none'),
    (False, None, 'all'),
    (False, None, 'final'),
    (False, None, 'transitions'),
    # Exercise recovery and ROS cancellation together during a write outage.
    (True, 'RETURN_ARM', 'all'),
])
def test_dds_result_feedback_cancel_and_query(tmp_path, monkeypatch, cancel, mode, write_failure):
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
            'ReleaseInPlace': dict(object_state=ObjectState.LEFT_GRIPPER),
            'ReturnArmAfterCancel': dict(arm_recovered=True),
            'StopAfterRecovery': dict(stop_confirmed=True),
        }
        return Observation(**values.get(node, {}))
    backend = WorkerBackend(operation)
    with socket.socket() as available:
        available.bind(('127.0.0.1', 0))
        port = available.getsockname()[1]
    server = MujocoManipulationNode(context=context, namespace=namespace, backend=backend,
        parameter_overrides=[Parameter('database_path', value=str(tmp_path/'runtime.sqlite3')),
                             Parameter('monitor_port', value=port)])
    save = server.store.save
    def fail_record(record, **kwargs):
        if write_failure == 'all' or record.result is not None:
            raise StoreError('injected runtime record write failure')
        save(record, **kwargs)
    def fail_transitions(*args, **kwargs):
        raise StoreError('injected runtime transition write failure')
    if write_failure in ('all', 'final'):
        monkeypatch.setattr(server.store, 'save', fail_record)
    elif write_failure == 'transitions':
        monkeypatch.setattr(server.store, 'save_transitions', fail_transitions)
    client_node = Node('client', context=context, namespace=namespace)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(server)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client = ActionClient(client_node, ExecuteManipulationSkill, 'manipulation/execute_skill')
    service = client_node.create_client(GetManipulationExecution, 'manipulation/get_execution')
    cancel_service = client_node.create_client(CancelManipulation, 'manipulation/cancel')
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
            if mode is None:
                assert wait(handle.cancel_goal_async()).goals_canceling
            else:
                assert cancel_service.wait_for_service(timeout_sec=5.)
                requested = wait(cancel_service.call_async(
                    CancelManipulation.Request(execution_id=goal.execution_id, mode=mode)))
                assert requested.accepted, requested.message
        wrapped = wait(handle.get_result_async())
        assert wrapped.status == (GoalStatus.STATUS_CANCELED if cancel else GoalStatus.STATUS_SUCCEEDED)
        result = wrapped.result
        assert result.status == ('CANCELED' if cancel else 'SUCCESS')
        assert result.execution_profile == 'mujoco' and result.stop_confirmed
        response = wait(service.call_async(GetManipulationExecution.Request(execution_id=goal.execution_id)))
        assert response.found and response.record.has_result
        assert response.record.status == result.status
        assert response.record.record_state == 'FINISHED'
        if write_failure in ('all', 'final'):
            saved = server.store.get(goal.execution_id)
            assert saved is None or saved.result is None
        assert any(f.substage == 'MoveToPregrasp' for f in feedback)
        if cancel:
            assert 'LiftObject' not in observed
            assert result.cancel_mode == (mode or 'CHECKPOINT')
            if mode is None:
                assert 'ConfirmGrasp' in observed
            if mode == 'RETURN_ARM':
                assert result.arm_recovered and result.object_state == 'LEFT_GRIPPER'
                assert result.placement_state == 'NOT_CHECKED'
                assert observed[-4:] == ['StopAndAssess', 'ReleaseInPlace', 'ReturnArmAfterCancel', 'StopAfterRecovery']
            elif mode == 'IMMEDIATE':
                assert 'ReleaseInPlace' not in observed and not result.arm_recovered
        else:
            assert result.arm_recovered
            assert server.core.tree.snapshot()['nodes'][0]['status'] != 'RUNNING'
    finally:
        executor.shutdown(timeout_sec=5.)
        thread.join(timeout=5.)
        server.destroy_node()
        client_node.destroy_node()
        context.shutdown()
