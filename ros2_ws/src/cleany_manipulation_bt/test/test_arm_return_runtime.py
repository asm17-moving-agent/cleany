"""Real ROS Action traffic with synthetic operations; no hardware access."""
import threading
import time
import uuid
import subprocess

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from cleany_interfaces.action import ReturnArm
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from cleany_manipulation_bt.backend import Observation, WorkerBackend
from cleany_manipulation_bt.jetson_node import JetsonManipulationNode
from cleany_skill_executor.manipulation.models import Goal
from test_arm_return import FakeOperations, HOME_FILE
from test_jetson_lifecycle import free_port


@pytest.fixture
def runtime(tmp_path):
    context = Context()
    rclpy.init(context=context)
    operations = FakeOperations()
    backend = WorkerBackend(lambda *_: Observation())
    namespace = '/arm_return_' + uuid.uuid4().hex
    server = JetsonManipulationNode(context=context, namespace=namespace,
        operations=operations, backend=backend, parameter_overrides=[
            Parameter('database_path', value=str(tmp_path / 'executions.sqlite3')),
            Parameter('monitor_port', value=free_port()),
            Parameter('arm_return_pose_file', value=str(HOME_FILE))])
    node = Node('arm_return_test', context=context, namespace=namespace)
    client = ActionClient(node, ReturnArm, 'manipulation/return_arm')
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(server)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    assert client.wait_for_server(timeout_sec=5.)
    try:
        yield server, operations, node, client
    finally:
        server.arm_return.request_shutdown()
        deadline = time.monotonic() + 5.
        while server.core.arm_return_active and time.monotonic() < deadline:
            time.sleep(.01)
        executor.shutdown(timeout_sec=5.)
        thread.join(timeout=5.)
        client.destroy()
        server.destroy_node()
        node.destroy_node()
        context.shutdown()


def wait(future):
    deadline = time.monotonic() + 5.
    while not future.done() and time.monotonic() < deadline:
        time.sleep(.01)
    assert future.done()
    return future.result()


def test_return_remains_available_after_failure_without_clearing_inhibition(runtime):
    server, ops, _, client = runtime
    server.core.inhibited = True
    feedback = []
    handle = wait(client.send_goal_async(ReturnArm.Goal(arm='both'),
        feedback_callback=lambda msg: feedback.append(msg.feedback.stage)))
    assert handle.accepted
    reply = wait(handle.get_result_async())
    assert reply.status == GoalStatus.STATUS_SUCCEEDED
    assert reply.result.success and reply.result.stop_confirmed
    assert server.core.inhibited and server.core.record is None
    assert feedback.index('opening_right') < feedback.index('returning_left')
    assert not server.core.arm_return_active


def test_cancel_opening_waits_for_stop_and_blocks_overlapping_work(runtime):
    server, ops, node, client = runtime
    ops.block = True
    handle = wait(client.send_goal_async(ReturnArm.Goal(arm='right')))
    assert handle.accepted and ops.started.wait(3.)
    duplicate = wait(client.send_goal_async(ReturnArm.Goal(arm='both')))
    assert not duplicate.accepted
    goal = Goal('mission', 'task', 'new', 'collect_trash', 'snapshot', 1, 'return_to_pick')
    with server.core.lock:
        assert not server.core.accept(goal)[0]
    assert wait(handle.cancel_goal_async()).goals_canceling
    result = wait(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_CANCELED
    assert not result.result.success and result.result.stop_confirmed
    assert ops.calls[-1] == 'stop'
    assert not any(c[0] == 'return' for c in ops.calls if isinstance(c, tuple))


def test_bad_arm_or_active_execution_is_rejected_before_motion(runtime):
    server, ops, _, client = runtime
    assert not wait(client.send_goal_async(ReturnArm.Goal(arm='head'))).accepted
    goal = Goal('mission', 'task', 'new', 'collect_trash', 'snapshot', 1, 'return_to_pick')
    with server.core.lock:
        assert server.core.accept(goal)[0]
    assert not wait(client.send_goal_async(ReturnArm.Goal(arm='left'))).accepted
    assert not ops.calls
    # Avoid ticking a normal collection during fixture cleanup.
    with server.core.lock:
        server.core.tree.halt()
        server.core.tree = None


def test_final_stop_failure_inhibits_new_collection(runtime):
    server, ops, _, client = runtime
    ops.stop_confirmed = False
    handle = wait(client.send_goal_async(ReturnArm.Goal(arm='left')))
    result = wait(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED
    assert not result.result.success and not result.result.stop_confirmed
    assert not result.result.recovery_required
    assert server.core.inhibited


def test_shutdown_interrupts_opening_and_delivers_stop_result(runtime):
    server, ops, _, client = runtime
    ops.block = True
    handle = wait(client.send_goal_async(ReturnArm.Goal(arm='right')))
    assert ops.started.wait(3.)
    server.arm_return.request_shutdown()
    result = wait(handle.get_result_async())
    assert not result.result.success and result.result.stop_confirmed
    assert not wait(client.send_goal_async(ReturnArm.Goal(arm='left'))).accepted
    assert ops.calls[-1] == 'stop'


def test_cli_timeout_cancels_and_waits_for_stop(runtime):
    server, ops, node, _ = runtime
    ops.block = True
    namespace = node.get_namespace()
    result = subprocess.run(['ros2', 'run', 'cleany_manipulation_bt', 'arm_return',
        '--arm', 'right', '--timeout', '.1', '--namespace', namespace],
        text=True, capture_output=True, timeout=15.)
    assert result.returncode == 1, result.stdout + result.stderr
    assert '취소 후 정지 확인: True' in result.stderr
    assert ops.calls[-1] == 'stop'
    assert not server.core.arm_return_active


def test_latched_guard_returns_recovery_guidance_without_submitting_work(runtime):
    server, ops, _, client = runtime
    ops.real.fault = 'Operator software stop'
    ops.real.guard = {'state': 'fault'}
    handle = wait(client.send_goal_async(ReturnArm.Goal(arm='right')))
    assert handle.accepted
    response = wait(handle.get_result_async())
    assert response.status == GoalStatus.STATUS_ABORTED
    assert not response.result.success and not response.result.stop_confirmed
    assert response.result.recovery_required
    assert 'make collection-recover-control' in response.result.message
    assert 'make manipulation-restart' in response.result.message
    assert ops.calls == [] and server.core.inhibited


def test_cli_identifies_pre_motion_latch_for_managed_recovery(runtime):
    _, ops, node, _ = runtime
    ops.real.fault = 'Operator software stop'
    result = subprocess.run(['ros2', 'run', 'cleany_manipulation_bt', 'arm_return',
        '--arm', 'right', '--namespace', node.get_namespace()],
        text=True, capture_output=True, timeout=15.)
    assert result.returncode == 21, result.stdout + result.stderr
    assert 'blocked_before_motion' in result.stdout and not ops.calls
