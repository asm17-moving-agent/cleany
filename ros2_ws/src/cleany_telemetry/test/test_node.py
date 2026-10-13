"""Check the ROS message boundary without starting ROS or network workers."""
from types import SimpleNamespace
import math

import pytest
from nav_msgs.msg import Odometry
from cleany_telemetry.node import TelemetryNode


def test_odometry_orientation_is_forwarded_and_invalid_rotation_omitted():
    calls = []
    relay = SimpleNamespace(update_pose=lambda *args, **kwargs: calls.append((args, kwargs)))
    node = SimpleNamespace(_relay=relay)
    msg = Odometry()
    msg.pose.pose.position.x = 1.0
    msg.pose.pose.position.y = -2.0
    msg.header.stamp.sec = 3
    msg.pose.pose.orientation.z = math.sin(math.pi / 4)
    msg.pose.pose.orientation.w = math.cos(math.pi / 4)
    TelemetryNode._odom(node, msg)
    assert calls[-1][0] == (1.0, -2.0, 3.0)
    assert calls[-1][1]['yaw'] == pytest.approx(math.pi / 2)
    msg.pose.pose.orientation.z = float('nan')
    TelemetryNode._odom(node, msg)
    assert calls[-1][1]['yaw'] is None
    msg.pose.pose.position.x = float('nan')
    TelemetryNode._odom(node, msg)
    assert len(calls) == 2
