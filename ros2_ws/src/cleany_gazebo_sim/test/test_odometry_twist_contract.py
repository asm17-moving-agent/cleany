"""The ideal error adapter must not reintroduce encoder velocity aliasing."""
from types import SimpleNamespace
from unittest.mock import Mock
import pytest

pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry
from cleany_gazebo_sim.odometry_error import OdometryErrorParameters, StatefulOdometryError
from cleany_gazebo_sim.simulated_odometry_error_node import SimulatedOdometryErrorNode


def test_ideal_adapter_preserves_measured_twist_and_input_message():
    adapter = SimpleNamespace(_model=StatefulOdometryError(OdometryErrorParameters()),
                              _slip_active=False, _publisher=Mock())
    message = Odometry()
    message.header.stamp.sec = 1
    message.header.frame_id = 'odom'
    message.child_frame_id = 'base_link'
    message.pose.pose.orientation.w = 1.
    message.twist.twist.linear.x = .04
    message.twist.twist.linear.y = -.02
    message.twist.twist.angular.z = .1
    for ns in (0, 10000000):
        message.header.stamp.nanosec = ns
        SimulatedOdometryErrorNode._on_odometry(adapter, message)
        output = adapter._publisher.publish.call_args.args[0]
        assert output.twist == message.twist
        assert output.header == message.header
        assert output.child_frame_id == message.child_frame_id
        assert output is not message
    assert message.twist.twist.linear.x == .04


def test_nonideal_adapter_keeps_twist_consistent_with_injected_pose():
    adapter = SimpleNamespace(_model=StatefulOdometryError(OdometryErrorParameters(forward_scale=1.1)),
                              _slip_active=False, _publisher=Mock())
    message = Odometry()
    message.header.stamp.sec = 1
    message.pose.pose.orientation.w = 1.
    SimulatedOdometryErrorNode._on_odometry(adapter, message)
    message.header.stamp.nanosec = 100000000
    message.pose.pose.position.x = .01
    message.twist.twist.linear.x = .04
    SimulatedOdometryErrorNode._on_odometry(adapter, message)
    output = adapter._publisher.publish.call_args.args[0]
    assert output.pose.pose.position.x == pytest.approx(.011)
    assert output.twist.twist.linear.x == pytest.approx(.11)
    assert message.twist.twist.linear.x == .04
