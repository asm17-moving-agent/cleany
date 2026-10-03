"""Keep rate limiting from inventing fresh or future robot feedback."""
from types import SimpleNamespace
import pytest
rclpy = pytest.importorskip('rclpy')
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from cleany_gazebo_sim.feedback_sampler import FeedbackSampler


def test_feedback_stamps_stall_future_and_clock_rewind(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '223')
    rclpy.init();node = FeedbackSampler()
    try:
        clocks, joints = [], []
        node.clock_pub = SimpleNamespace(publish=clocks.append)
        node.joint_pub = SimpleNamespace(publish=joints.append)
        c = Clock();c.clock.sec = 3
        j = JointState();j.header.stamp.sec = 4
        node.on_joint(j);node.on_clock(c);node.publish_latest()
        assert len(clocks) == 1 and not joints
        c = Clock();c.clock.sec = 4
        node.on_clock(c);node.publish_latest();node.publish_latest()
        assert len(clocks) == 2 and joints == [j]
        assert joints[0].header.stamp.sec == 4
        c = Clock();c.clock.sec = 1
        node.on_clock(c);node.publish_latest()
        assert len(clocks) == 3 and len(joints) == 1
        j = JointState();j.header.stamp.sec = 1
        node.on_joint(j);node.publish_latest()
        assert len(joints) == 2 and joints[-1].header.stamp.sec == 1
    finally:
        node.destroy_node();rclpy.shutdown()
