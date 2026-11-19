"""Check Backend-specific metadata and freshness at the ROS adapter boundary."""
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from cleany_control_bridge.pose_node import PoseBackendBridge
from cleany_telemetry.pose import PoseFreshness
from geometry_msgs.msg import PoseStamped


def test_map_identity_is_only_attached_to_map_coordinates():
    calls = []
    node = SimpleNamespace(
        freshness=PoseFreshness(), map_id='slam-fixture', map_frame='map',
        relay=SimpleNamespace(update_pose=lambda *a, **kw: calls.append((a, kw))),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)),
    )
    msg = PoseStamped()
    msg.header.stamp.sec = 9
    msg.header.frame_id = 'odom'
    msg.pose.position.x = 1.0
    msg.pose.orientation.w = 1.0
    PoseBackendBridge._pose(node, msg)
    assert calls[-1][1] == dict(yaw=0.0, frame_id='odom', map_id=None, source_age=1.0)
    msg.header.frame_id = 'map'
    PoseBackendBridge._pose(node, msg)
    assert calls[-1][1]['map_id'] == 'slam-fixture'
    PoseBackendBridge._pose(node, msg)
    msg.header.stamp.sec = 1
    PoseBackendBridge._pose(node, msg)
    assert len(calls) == 2
