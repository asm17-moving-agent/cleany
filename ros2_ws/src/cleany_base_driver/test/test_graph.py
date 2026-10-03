"""Isolated graph regression for ENABLE permission, motion and odom ownership."""
import os
import threading
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('BASE_TEST_ROS_DOMAIN_ID', '173')

import rclpy
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage

from cleany_base_driver.driver_node import BaseDriver
from cleany_base_driver.mock_node import MockMcu
from cleany_base_driver.odom_relay import OdomRelay
from cleany_base_odometry.wheel_odom_node import WheelOdometryNode


def _wait(predicate, timeout=3.):
    end = time.monotonic() + timeout
    while not predicate() and time.monotonic() < end:
        time.sleep(.01)
    assert predicate()


def _call(client, request):
    assert client.wait_for_service(timeout_sec=2)
    future = client.call_async(request)
    _wait(future.done)
    response = future.result()
    assert response.success
    return response


def test_mocked_graph_end_to_end():
    rclpy.init(args=['--ros-args', '-p', 'wheel_radius_m:=0.05', '-p', 'wheelbase_m:=0.30',
                     '-p', 'wheel_separation_m:=0.30'])
    params = {'geometry.wheel_radius_m': .05, 'geometry.wheelbase_m': .30,
              'geometry.wheel_separation_m': .30, 'limits.linear_x_mps': .4,
              'limits.linear_y_mps': .4, 'limits.angular_z_rad_s': .5,
              'limits.wheel_rad_s': 10., 'limits.command_timeout_s': .4}
    driver = BaseDriver(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
    mock, odom, relay = MockMcu(), WheelOdometryNode(), OdomRelay()
    assert float(odom.get_parameter('wheel_radius_m').value) == .05
    assert float(odom.get_parameter('wheelbase_m').value) == .30
    assert float(odom.get_parameter('wheel_separation_m').value) == .30
    monitor = rclpy.create_node('graph_monitor')
    received, transforms, wheel_odom, joints, diagnostics = [], [], [], [], []
    monitor.create_subscription(Odometry, 'odom', received.append, 10)
    monitor.create_subscription(TFMessage, 'tf', transforms.append, 10)
    monitor.create_subscription(Odometry, 'wheel/odom', wheel_odom.append, 10)
    monitor.create_subscription(JointState, 'joint_states', joints.append, 10)
    monitor.create_subscription(DiagnosticArray, 'diagnostics', diagnostics.append, 10)
    executor = MultiThreadedExecutor()
    nodes = [driver, mock, odom, relay, monitor]
    for node in nodes:
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        _wait(lambda: driver.latest is not None)
        enable = monitor.create_client(SetBool, 'base/enable')
        _wait(enable.service_is_ready)
        pub = monitor.create_publisher(Twist, 'cmd_vel', 10)
        twist = Twist()
        twist.linear.x = .1
        pub.publish(twist)  # Must be discarded while disabled.
        assert not driver.enabled
        drop = monitor.create_client(Trigger, 'mock/drop_next_enable')
        _call(drop, Trigger.Request())
        future = enable.call_async(SetBool.Request(data=True))
        _wait(lambda: future.done())
        assert future.result().success
        _wait(lambda: driver.latest.enabled and driver.pending_enable is None)
        assert mock.target == [0.] * 4 and driver.latest_twist is None
        end = time.monotonic() + .32
        while time.monotonic() < end:
            pub.publish(twist)
            time.sleep(.02)
        _wait(lambda: bool(received) and received[-1].pose.pose.position.x > .02)
        x = received[-1].pose.pose.position.x
        assert .02 < x < .05
        lateral = Twist()
        lateral.linear.y = .1
        end = time.monotonic() + .32
        while time.monotonic() < end:
            pub.publish(lateral)
            time.sleep(.02)
        _wait(lambda: received[-1].pose.pose.position.y > .02)
        assert .02 < received[-1].pose.pose.position.y < .05
        turning = Twist()
        turning.angular.z = .2
        end = time.monotonic() + .32
        while time.monotonic() < end:
            pub.publish(turning)
            time.sleep(.02)
        _wait(lambda: received[-1].pose.pose.orientation.z > .01)
        assert .02 < 2 * __import__('math').asin(received[-1].pose.pose.orientation.z) < .08
        assert wheel_odom and joints and diagnostics
        assert joints[-1].header.stamp.sec + joints[-1].header.stamp.nanosec > 0
        assert transforms
        assert len(monitor.get_publishers_info_by_topic('/odom')) == 1
        assert len(monitor.get_publishers_info_by_topic('/tf')) == 1
        assert received[-1].header.frame_id == 'odom'
        assert received[-1].child_frame_id == 'base_link'
        future = enable.call_async(SetBool.Request(data=False))
        _wait(lambda: future.done())
        assert future.result().success
        _wait(lambda: not mock.enabled)
        # Reboot preserves odometry while clearing authority and cached targets.
        time.sleep(.08)
        before = received[-1].pose.pose.position
        reboot = monitor.create_client(Trigger, 'mock/reboot')
        old_boot = driver.boot_id
        _call(reboot, Trigger.Request())
        _wait(lambda: driver.boot_id != old_boot and not driver.enabled)
        time.sleep(.08)
        assert abs(received[-1].pose.pose.position.x - before.x) < .01
        assert abs(received[-1].pose.pose.position.y - before.y) < .01
        _call(enable, SetBool.Request(data=True))
        _wait(lambda: driver.pending_enable is None and mock.enabled)
        assert driver.latest_twist is None and mock.target == [0.] * 4
        force = monitor.create_client(Trigger, 'mock/force_disarm')
        _call(force, Trigger.Request())
        _wait(lambda: not driver.enabled)
        assert driver.latest_twist is None
        _call(enable, SetBool.Request(data=True))
        _wait(lambda: driver.pending_enable is None and mock.enabled)
        transport = monitor.create_client(SetBool, 'mock/transport')
        _call(transport, SetBool.Request(data=False))
        _wait(lambda: not driver.enabled and driver.latest is None)
        _call(transport, SetBool.Request(data=True))
        _wait(lambda: driver.latest is not None)
        assert not driver.enabled and mock.target == [0.] * 4
        _call(enable, SetBool.Request(data=True))
        _wait(lambda: driver.pending_enable is None and mock.enabled)
        pub.publish(twist)
        _wait(lambda: mock.target[0] > 0)
        driver._disable('driver shutdown')
        _wait(lambda: not mock.enabled and mock.target == [0.] * 4)
    finally:
        executor.shutdown()
        thread.join(timeout=2)
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
