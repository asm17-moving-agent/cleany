import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from sensor_msgs.msg import JointState

from cleany_robot_interface.control_state_guard import ControlStateGuard, EXPECTED_JOINTS, valid_positions


def state():
    message = JointState()
    message.header.stamp.sec = 10
    message.name = sorted(EXPECTED_JOINTS)
    message.position = [0.0]*14
    return message


@pytest.mark.parametrize('issue', ['nan', 'missing', 'duplicate', 'stale', 'future'])
def test_invalid_state_is_not_accepted(issue):
    message = state()
    now = 10_100_000_000
    if issue == 'nan':
        message.position[0] = math.nan
    elif issue == 'missing':
        message.position.pop()
    elif issue == 'duplicate':
        message.name[0] = message.name[1]
    elif issue == 'stale':
        now += 1_000_000_000
    elif issue == 'future':
        now -= 1_000_000_000
    assert not valid_positions(message, now, .25)


def test_only_fresh_finite_positions_forwarded_and_repeated_stamp_not_refreshed():
    message = state()
    message.velocity = message.effort = [math.nan]*14
    node = SimpleNamespace(fault='', max_age=.25, last_stamp=-1, last_received=None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_100_000_000)),
        latch=Mock(), publisher=Mock())
    ControlStateGuard.receive(node, message)
    published = node.publisher.publish.call_args.args[0]
    assert len(published.position) == 14
    assert not published.velocity and not published.effort
    old = node.last_received
    ControlStateGuard.receive(node, message)
    assert node.last_received == old
    assert node.publisher.publish.call_count == 1
    node.fault = 'latched'
    message.header.stamp.nanosec = 1
    ControlStateGuard.receive(node, message)
    assert node.publisher.publish.call_count == 1
