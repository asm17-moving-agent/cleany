"""Synthetic ROS pose tests; no motion nodes or Backend are started."""
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from cleany_telemetry.pose_node import PoseObserver
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


@pytest.mark.parametrize('source_kind', ['odom', 'tf'])
def test_standalone_observer_preserves_frame_stamp_and_expires(source_kind):
    rclpy.init()
    observer = PoseObserver(parameter_overrides=[
        Parameter('source', value=source_kind), Parameter('input_timeout', value=0.2),
        Parameter('rate', value=30.0)])
    source = rclpy.create_node('pose_fixture')
    executor = SingleThreadedExecutor()
    executor.add_node(observer)
    executor.add_node(source)
    received = []
    source.create_subscription(PoseStamped, '/telemetry/pose', received.append, qos_profile_sensor_data)
    publisher = source.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
    tf = TransformBroadcaster(source)
    message = Odometry()
    message.header.frame_id = 'odom'
    message.pose.pose.position.x = 1.25
    message.pose.pose.position.y = -2.5
    message.pose.pose.orientation.w = 2.0
    transform = TransformStamped(child_frame_id='base_link')
    transform.header.frame_id = 'map'
    transform.transform.translation.x = 1.25
    transform.transform.translation.y = -2.5
    transform.transform.rotation.w = 1.0
    stamps = set()

    def send(fresh=True):
        if fresh:
            stamp = source.get_clock().now().to_msg()
            message.header.stamp = stamp
            transform.header.stamp = stamp
            stamps.add((stamp.sec, stamp.nanosec))
        if source_kind == 'tf':
            tf.sendTransform(transform)
        else:
            publisher.publish(message)
        executor.spin_once(timeout_sec=0.01)

    try:
        deadline = time.monotonic() + 8
        while len(received) < 3 and time.monotonic() < deadline:
            send()
        assert len(received) >= 3
        assert all(p.header.frame_id == ('map' if source_kind == 'tf' else 'odom') for p in received)
        assert all((p.header.stamp.sec, p.header.stamp.nanosec) in stamps for p in received)
        assert received[-1].pose.position.x == 1.25
        assert received[-1].pose.position.y == -2.5
        assert received[-1].pose.orientation.w == 1.0
        # Repeated old TF/odom must not become a new observation.
        deadline = time.monotonic() + 0.4
        while time.monotonic() < deadline:
            send(False)
        count = len(received)
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            send(False)
        assert len(received) == count
        if source_kind == 'odom':
            message.pose.pose.position.x = float('nan')
            deadline = time.monotonic() + .2
            while time.monotonic() < deadline:
                send()
            assert len(received) == count
    finally:
        observer.destroy_node()
        source.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
