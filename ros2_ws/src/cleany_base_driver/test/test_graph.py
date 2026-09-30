"""Isolated device-free graph test covering movement, recovery and ownership."""
import os
from copy import deepcopy
import threading
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('BASE_TEST_ROS_DOMAIN_ID', '173')

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
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


def _await(future, timeout=2.0):
    end = time.monotonic() + timeout
    while not future.done() and time.monotonic() < end:
        time.sleep(.01)
    assert future.done()
    return future.result()


def _publish_for(pub, command, seconds=.32):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        pub.publish(command)
        time.sleep(.02)


def test_mocked_graph_end_to_end():
    # Apply a test-scoped ROS parameter override so both the driver and the
    # existing wheel-odometry node consume the selected synthetic geometry.
    rclpy.init(args=[
        '--ros-args',
        '-p', 'wheel_radius_m:=0.05',
        '-p', 'wheelbase_m:=0.30',
        '-p', 'wheel_separation_m:=0.30',
    ])
    geometry = {
        'geometry.wheel_radius_m': .05,
        'geometry.wheelbase_m': .30,
        'geometry.wheel_separation_m': .30,
        'limits.linear_x_mps': .4, 'limits.linear_y_mps': .4,
        'limits.angular_z_rad_s': .5, 'limits.wheel_rad_s': 10.,
        'limits.command_timeout_s': .4,
    }
    driver = BaseDriver(parameter_overrides=[
        Parameter(key, value=value) for key, value in geometry.items()
    ])
    mock, odom, relay = MockMcu(), WheelOdometryNode(), OdomRelay()
    assert float(odom.get_parameter('wheel_radius_m').value) == .05
    assert float(odom.get_parameter('wheelbase_m').value) == .30
    assert float(odom.get_parameter('wheel_separation_m').value) == .30
    monitor = rclpy.create_node('graph_monitor')
    received, transforms, wheels, diagnostics, joints = [], [], [], [], []
    monitor.create_subscription(Odometry, 'odom', received.append, 10)
    monitor.create_subscription(TFMessage, 'tf', transforms.append, 10)
    monitor.create_subscription(Odometry, 'wheel/odom', wheels.append, 10)
    monitor.create_subscription(DiagnosticArray, 'diagnostics', diagnostics.append, 10)
    monitor.create_subscription(JointState, 'joint_states', joints.append, 10)
    executor = MultiThreadedExecutor()
    nodes = [driver, mock, odom, relay, monitor]
    for node in nodes:
        executor.add_node(node)
    thread_errors = []
    def spin():
        try:
            executor.spin()
        except BaseException as error:
            thread_errors.append(error)
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    try:
        end = time.monotonic() + 4
        while not driver.session_confirmed and time.monotonic() < end:
            time.sleep(.01)
        assert driver.session_confirmed, (
            f'session={driver.session} started={driver._started} '
            f'feedback={getattr(driver.latest, "session_id", None)} '
            f'mock_session={mock.session} mock_boot={mock.boot} '
            f'mock_seq={mock.sequence} receipt={driver.last_state_receipt} '
            f'state_subs={mock.pub.get_subscription_count()} '
            f'wheel_state_pubs={driver.count_publishers("base/wheel_state")} '
            f'error={driver.diagnostic_error} thread_errors={thread_errors}'
        )
        assert driver.diagnostic_error == ''
        enable = monitor.create_client(SetBool, 'base/enable')
        drop_begin = monitor.create_client(Trigger, 'mock/drop_next_begin')
        drop_arm = monitor.create_client(Trigger, 'mock/drop_next_arm')
        reboot = monitor.create_client(Trigger, 'mock/reboot')
        transport = monitor.create_client(SetBool, 'mock/transport')
        assert enable.wait_for_service(timeout_sec=2)
        assert drop_begin.wait_for_service(timeout_sec=2)
        assert drop_arm.wait_for_service(timeout_sec=2)

        # A lost BEGIN must be retried without changing the proposed session.
        old_session = driver.session
        _await(drop_begin.call_async(Trigger.Request()))
        assert _await(enable.call_async(SetBool.Request(data=False))).success
        end = time.monotonic() + 2
        while (not driver.session_confirmed or driver.session == old_session) and time.monotonic() < end:
            time.sleep(.01)
        assert driver.session_confirmed and driver.session != old_session

        # Drop first ARM; retries are zero-only until matching armed feedback.
        _await(drop_arm.call_async(Trigger.Request()))
        assert _await(enable.call_async(SetBool.Request(data=True))).success
        pub = monitor.create_publisher(Twist, 'cmd_vel', 10)
        pre_arm = Twist()
        pre_arm.linear.x = .4
        pub.publish(pre_arm)
        end = time.monotonic() + 2
        while not driver.gate.armed and time.monotonic() < end:
            time.sleep(.01)
        assert driver.gate.armed and mock.armed
        assert mock.target == [0.] * 4 and driver.gate.pending_velocity is None

        # ARM acknowledgement timeout clears authority, retires the session,
        # and cannot resume until a separate explicit enable.
        assert _await(enable.call_async(SetBool.Request(data=False))).success
        end = time.monotonic() + 2
        while not driver.session_confirmed and time.monotonic() < end:
            time.sleep(.01)
        assert driver.session_confirmed and not driver.gate.armed
        assert driver.diagnostic_error == ''
        _await(drop_arm.call_async(Trigger.Request()))
        mock.drop_arm_count = 100
        assert _await(enable.call_async(SetBool.Request(data=True))).success
        end = time.monotonic() + 1
        while driver.gate.enabled and time.monotonic() < end:
            time.sleep(.01)
        assert not driver.gate.enabled and not driver.gate.armed
        end = time.monotonic() + 2
        while not driver.session_confirmed and time.monotonic() < end:
            time.sleep(.01)
        assert driver.session_confirmed and not driver.gate.enabled
        mock.drop_arm_count = 0
        assert _await(enable.call_async(SetBool.Request(data=True))).success
        end = time.monotonic() + 2
        while not driver.gate.armed and time.monotonic() < end:
            time.sleep(.01)
        assert driver.gate.armed and mock.armed

        forward = Twist()
        forward.linear.x = .1
        _publish_for(pub, forward, .32)
        end = time.monotonic() + 2
        while (not received or received[-1].pose.pose.position.x < .02) and time.monotonic() < end:
            time.sleep(.01)
        assert .02 < received[-1].pose.pose.position.x < .05

        lateral = Twist()
        lateral.linear.y = .1
        _publish_for(pub, lateral, .32)
        end = time.monotonic() + 2
        while received[-1].pose.pose.position.y < .02 and time.monotonic() < end:
            time.sleep(.01)
        assert .02 < received[-1].pose.pose.position.y < .05

        turning = Twist()
        turning.angular.z = .2
        _publish_for(pub, turning, .32)
        end = time.monotonic() + 2
        while received[-1].pose.pose.orientation.z < .01 and time.monotonic() < end:
            time.sleep(.01)
        yaw = 2 * __import__('math').asin(received[-1].pose.pose.orientation.z)
        assert .02 < yaw < .08
        assert joints and joints[-1].header.stamp.sec + joints[-1].header.stamp.nanosec > 0
        assert wheels and diagnostics and transforms
        assert len(monitor.get_publishers_info_by_topic('/odom')) == 1
        assert len(monitor.get_publishers_info_by_topic('/tf')) == 1
        assert received[-1].header.frame_id == 'odom'
        assert received[-1].child_frame_id == 'base_link'

        # Duplicate/stale telemetry clears all authority and cannot satisfy an
        # enable request even though the prior session had been armed.
        stale = deepcopy(driver.latest)
        driver.on_state(stale)
        denied = driver.on_enable(SetBool.Request(data=True), SetBool.Response())
        assert not denied.success and driver.latest is None and not driver.gate.armed
        end = time.monotonic() + 2
        while not driver.session_confirmed and time.monotonic() < end:
            time.sleep(.01)
        assert driver.session_confirmed and not driver.gate.enabled
        assert _await(enable.call_async(SetBool.Request(data=True))).success
        end = time.monotonic() + 1
        while not driver.gate.armed and time.monotonic() < end:
            time.sleep(.01)
        assert driver.gate.armed
        _publish_for(pub, forward, .1)

        # Unexpected MCU session change and armed=false each retire the host
        # session; neither condition automatically restores enable.
        change = monitor.create_client(Trigger, 'mock/change_session')
        force_disarm = monitor.create_client(Trigger, 'mock/force_disarm')
        old_session = driver.session
        assert _await(change.call_async(Trigger.Request())).success
        end = time.monotonic() + 2
        while (not driver.session_confirmed or driver.session == old_session) and time.monotonic() < end:
            time.sleep(.01)
        assert driver.session_confirmed and not driver.gate.enabled
        assert _await(enable.call_async(SetBool.Request(data=True))).success
        end = time.monotonic() + 1
        while not driver.gate.armed and time.monotonic() < end:
            time.sleep(.01)
        assert driver.gate.armed
        _publish_for(pub, forward, .1)
        old_session = driver.session
        assert _await(force_disarm.call_async(Trigger.Request())).success
        end = time.monotonic() + 2
        while (not driver.session_confirmed or driver.session == old_session) and time.monotonic() < end:
            time.sleep(.01)
        assert driver.session_confirmed and not driver.gate.enabled

        assert reboot.wait_for_service(timeout_sec=1)
        x_before_reboot = received[-1].pose.pose.position.x
        y_before_reboot = received[-1].pose.pose.position.y
        old_session = driver.session
        _await(reboot.call_async(Trigger.Request()))
        end = time.monotonic() + 2
        while (not driver.session_confirmed or driver.session == old_session) and time.monotonic() < end:
            time.sleep(.01)
        assert driver.session_confirmed and not driver.gate.armed
        time.sleep(.08)
        assert abs(received[-1].pose.pose.position.x - x_before_reboot) < .01
        assert abs(received[-1].pose.pose.position.y - y_before_reboot) < .01

        # Reboot requires a new explicit enable and a new post-ARM cmd_vel.
        assert _await(enable.call_async(SetBool.Request(data=True))).success
        end = time.monotonic() + 2
        while not driver.gate.armed and time.monotonic() < end:
            time.sleep(.01)
        assert driver.gate.armed
        _publish_for(pub, forward, .15)

        assert transport.wait_for_service(timeout_sec=1)
        assert _await(transport.call_async(SetBool.Request(data=False))).success
        end = time.monotonic() + 1
        while driver.latest is not None and time.monotonic() < end:
            time.sleep(.01)
        assert not driver.gate.armed and driver.session == 0
        assert _await(transport.call_async(SetBool.Request(data=True))).success
        end = time.monotonic() + 2
        while not driver.session_confirmed and time.monotonic() < end:
            time.sleep(.01)
        assert driver.session_confirmed and not driver.gate.armed
        # Orderly shutdown attempts STOP before destroying the publisher.
        assert _await(enable.call_async(SetBool.Request(data=True))).success
        end = time.monotonic() + 1
        while not driver.gate.armed and time.monotonic() < end:
            time.sleep(.01)
        assert driver.gate.armed
        _publish_for(pub, forward, .08)
        driver._invalidate('driver shutdown')
        end = time.monotonic() + 1
        while mock.armed and time.monotonic() < end:
            time.sleep(.01)
        assert not mock.armed and mock.target == [0.] * 4
    finally:
        executor.shutdown()
        thread.join(timeout=2)
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
