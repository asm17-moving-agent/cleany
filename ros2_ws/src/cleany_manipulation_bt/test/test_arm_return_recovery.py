"""Recovery classification and live DDS probes without controllers or motors."""
import json
import threading
import uuid

import pytest
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from cleany_manipulation_bt.arm_return_recovery import (
    SOFTWARE_STOP, check_driver_stop, classify_control, probe_control)


READY = {'state': 'ready', 'reason': ''}
STOP = {'state': 'fault', 'reason': 'Operator software stop',
        'controllers_stopped': True, 'actions_cancelled': True}


@pytest.mark.parametrize('status,code', [
    (READY, 0), (STOP, SOFTWARE_STOP),
    ({**STOP, 'controllers_stopped': False}, 1),
    ({**STOP, 'actions_cancelled': False}, 1),
    ({**STOP, 'controllers_stopped': 'true'}, 1),
    ({**STOP, 'reason': 'Hardware feedback timeout'}, 1),
    ({'state': 'waiting'}, 1), ({}, 1),
])
def test_only_completed_operator_software_stop_is_recoverable(status, code):
    assert classify_control(status).code == code


@pytest.mark.parametrize('fault,code', [
    ('All command controllers stopped', SOFTWARE_STOP),
    ('Serial read timeout ID 14', 1), ('Motor temperature limit', 1), ('Lifecycle error', 1),
    ('', 1),
])
def test_guard_software_stop_cannot_hide_an_underlying_driver_fault(tmp_path, fault, code):
    path = tmp_path / 'active-control.log'
    path.write_text(f'LATCHED hardware fault: {fault}; no automatic restart\n' if fault else '')
    assert check_driver_stop(str(path)).code == code


def test_missing_control_log_is_not_recoverable(tmp_path):
    assert check_driver_stop(str(tmp_path / 'missing')).code == 1


def test_appended_log_uses_latest_driver_latch(tmp_path):
    path = tmp_path / 'recovery.log'
    path.write_text('LATCHED hardware fault: Serial read timeout; no automatic restart\n'
                    'LATCHED hardware fault: All command controllers stopped; no automatic restart\n')
    assert check_driver_stop(str(path)).code == SOFTWARE_STOP
    with path.open('a') as stream:
        stream.write('LATCHED hardware fault: Motor error; no automatic restart\n')
    assert check_driver_stop(str(path)).code == 1


@pytest.mark.parametrize('mode,expected', [('ready', 0), ('stop', SOFTWARE_STOP),
                                         ('hard_fault', 1), ('cached_only', 1)])
def test_probe_requires_live_status_and_never_sends_motion(mode, expected):
    context = Context()
    rclpy.init(context=context)
    namespace = '/return_probe_' + uuid.uuid4().hex
    publisher_node = Node('synthetic_guard', context=context, namespace=namespace)
    observer = Node('return_probe', context=context, namespace=namespace)
    publisher = publisher_node.create_publisher(String, 'real_control/status',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    status = READY if mode in ('ready', 'cached_only') else STOP
    if mode == 'hard_fault':
        status = {**STOP, 'reason': 'Hardware feedback timeout'}
    message = String(data=json.dumps(status))
    stop = threading.Event()

    def publish():
        while not stop.wait(.03):
            publisher.publish(message)

    thread = threading.Thread(target=publish, daemon=True)
    if mode == 'cached_only':
        publisher.publish(message)
    else:
        thread.start()
    try:
        result = probe_control(observer, namespace, 1.)
        assert result.code == expected, result.message
        assert not list(observer.subscriptions) and not list(observer.clients)
        assert all('/follow_joint_trajectory' not in publisher.topic_name
                   for publisher in observer.publishers)
    finally:
        stop.set()
        if thread.is_alive():
            thread.join(1.)
        observer.destroy_node()
        publisher_node.destroy_node()
        context.shutdown()


def test_cancelled_probe_does_not_request_recovery():
    context = Context()
    rclpy.init(context=context)
    observer = Node('cancelled_probe', context=context)
    try:
        with pytest.raises(InterruptedError):
            probe_control(observer, '/missing', 1., lambda: True)
    finally:
        observer.destroy_node()
        context.shutdown()
