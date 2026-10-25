"""Lifecycle recovery owner against a synthetic lifecycle manager service."""
from threading import Thread
from time import monotonic, sleep

import pytest

rclpy = pytest.importorskip('rclpy')
from cleany_interfaces.action import RecoverComponent  # noqa: E402
from nav2_msgs.srv import ManageLifecycleNodes  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402

from cleany_navigation.lifecycle_recovery_node import LifecycleRecovery  # noqa: E402


def wait(predicate, timeout=5.):
    deadline = monotonic() + timeout
    while monotonic() < deadline and not predicate():
        sleep(.01)
    assert predicate()


@pytest.fixture
def graph():
    rclpy.init(domain_id=231)
    peer = rclpy.create_node('lifecycle_manager_peer')
    group = ReentrantCallbackGroup()
    commands, outcome = [], {'success': True}

    def manage(request, response):
        commands.append(request.command)
        sleep(.2)
        response.success = outcome['success']
        return response
    peer.create_service(ManageLifecycleNodes, 'lifecycle_manager_navigation/manage_nodes', manage,
                        callback_group=group)
    owner = LifecycleRecovery()
    executor = MultiThreadedExecutor(num_threads=6)
    for node in (peer, owner):
        executor.add_node(node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield peer, commands, outcome
    finally:
        executor.shutdown(timeout_sec=2.)
        owner.destroy_node()
        peer.destroy_node()
        rclpy.shutdown()


def send(client, job='job'):
    sent = client.send_goal_async(RecoverComponent.Goal(job_id=job, fault_code='NAVIGATION_UNAVAILABLE',
                                                        source='navigation'))
    wait(sent.done)
    return sent.result()


def test_reset_then_startup_and_conflicting_goal_rejected(graph):
    peer, commands, _ = graph
    client = ActionClient(peer, RecoverComponent, 'navigation/recover')
    assert client.wait_for_server(timeout_sec=5.)
    first = send(client)
    assert first.accepted
    assert not send(client, 'other').accepted  # One recovery at a time.
    result = first.get_result_async()
    wait(result.done)
    assert result.result().result.success
    assert commands == [ManageLifecycleNodes.Request.RESET, ManageLifecycleNodes.Request.STARTUP]


def test_failed_manager_step_is_reported_and_stops(graph):
    peer, commands, outcome = graph
    outcome['success'] = False
    client = ActionClient(peer, RecoverComponent, 'navigation/recover')
    assert client.wait_for_server(timeout_sec=5.)
    result = send(client).get_result_async()
    wait(result.done)
    assert not result.result().result.success and commands == [ManageLifecycleNodes.Request.RESET]


def test_missing_lifecycle_manager_fails_honestly(graph):
    peer, _, _ = graph
    client = ActionClient(peer, RecoverComponent, 'localization/recover')
    assert client.wait_for_server(timeout_sec=5.)
    result = send(client).get_result_async()
    wait(result.done, 8.)
    assert not result.result().result.success and 'unavailable' in result.result().result.detail
