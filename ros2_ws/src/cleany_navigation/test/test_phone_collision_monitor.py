"""Exercise the real monitor with isolated synthetic scans, never robot topics."""
import math
import os
from pathlib import Path
import subprocess
import time

import pytest
import yaml
import numpy as np

rclpy = pytest.importorskip('rclpy')
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.msg import SpeedLimit
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import StaticTransformBroadcaster
from cleany_navigation.runtime_config import adapt_nav2_parameters
from cleany_navigation.point_cloud import xyz_cloud


@pytest.mark.parametrize('body_limits',[False,True])
@pytest.mark.parametrize('depth_enabled',[False,True])
def test_absolute_limit_stop_priority_and_lifecycle(tmp_path,body_limits,depth_enabled):
    domain = 191
    config = yaml.safe_load((Path(__file__).parents[1] / 'config/phone_lidar_safety.yaml').read_text())
    if depth_enabled:
        overlay = yaml.safe_load((Path(__file__).parents[1] / 'config/phone_depth_local.yaml').read_text())
        config['collision_monitor']['ros__parameters'].update(overlay['collision_monitor']['ros__parameters'])
    config = adapt_nav2_parameters(config, 'humble')
    config['collision_monitor']['ros__parameters']['base_shift_correction'] = False
    if not body_limits:
        config['collision_monitor']['ros__parameters'].update(
            translation_limit_m_s=0.,rotation_limit_rad_s=0.,wheel_limit_rad_s=1.5)
    filename = tmp_path / 'monitor.yaml'
    filename.write_text(yaml.safe_dump(config))
    binary = Path(get_package_prefix('cleany_navigation')) / 'lib/cleany_navigation/phone_collision_monitor'
    assert binary.is_file()
    log = (tmp_path / 'monitor.log').open('w')
    process = subprocess.Popen([str(binary), '--ros-args', '--params-file', str(filename)],
                               env={**os.environ, 'ROS_DOMAIN_ID': str(domain)}, stdout=log, stderr=log)
    rclpy.init(domain_id=domain)
    node = rclpy.create_node('wheel_limit_test')
    received = []
    scan_pub = node.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
    depth_pub = node.create_publisher(PointCloud2, '/perception/depth/nav_obstacle_points', qos_profile_sensor_data)
    publish_depth = [True]
    cmd_pub = node.create_publisher(Twist, 'motion/cmd_vel', 1)
    limit_pub = node.create_publisher(SpeedLimit,'navigation/localization_speed_limit',10)
    percentage = [100.]
    subscription = node.create_subscription(Twist, 'navigation/safe_cmd_vel', received.append, 10)
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'test_laser'
    transform.transform.rotation.w = 1.
    change = node.create_client(ChangeState, '/collision_monitor/change_state')

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)

    def transition(identifier):
        request = ChangeState.Request()
        request.transition.id = identifier
        future = change.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
        assert future.done() and future.result().success

    def send(distance, x, y=0., yaw=0.):
        if percentage[0] is not None:
            limit_pub.publish(SpeedLimit(percentage=True,speed_limit=percentage[0]))
        scan = LaserScan()
        scan.header.frame_id = 'test_laser'
        scan.header.stamp = node.get_clock().now().to_msg()
        scan.angle_min, scan.angle_max, scan.angle_increment = -.04, .04, .01
        scan.range_min, scan.range_max = .1, 10.
        scan.ranges = [math.inf if depth_enabled else distance] * 9
        scan_pub.publish(scan)
        if depth_enabled and publish_depth[0]:
            points = np.array([[distance, y, .6] for y in [-.02, 0., .02]]) if math.isfinite(distance) else np.empty((0, 3))
            depth_pub.publish(xyz_cloud(points, 'base_link', scan.header.stamp))
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = x, y, yaw
        cmd_pub.publish(cmd)

    def expect(distance, request, expected):
        start = len(received)
        end = time.monotonic() + 3.
        while time.monotonic() < end:
            send(distance, *request)
            pump(.05)
            for msg in received[start:]:
                actual = (msg.linear.x, msg.linear.y, msg.angular.z)
                if all(abs(a-b) < 1e-7 for a, b in zip(actual, expected)):
                    return
        pytest.fail(f'Expected {expected}; last outputs: {received[-3:]}')

    try:
        assert change.wait_for_service(timeout_sec=5.)
        transition(1)
        transform.header.stamp = node.get_clock().now().to_msg()
        broadcaster.sendTransform(transform)
        pump(.2)
        transition(3)
        expect(math.inf, (.254, 0., 0.), (.254, 0., 0.))
        expect(.35, (.254, 0., 0.), (.16 if body_limits else 1.5 * .0635, 0., 0.))
        expect(.35, (.4 * .0635, 0., 0.), (.4 * .0635, 0., 0.))
        ratio = 1. if body_limits else 1.5 * .0635 / (.06 + .03 + .47 * .1)
        expect(.35, (.06, .03, .1), (.06*ratio, .03*ratio, .1*ratio))
        if body_limits:
            expect(.35, (0.,0.,.7), (0.,0.,.3))
            scale=.16/math.hypot(.2,.2)
            expect(.35, (.2,.2,0.), (.2*scale,.2*scale,0.))
        expect(.2, (.127, 0., .1), (0., 0., 0.))
        expect(math.inf, (.254, 0., 0.), (.254, 0., 0.))
        percentage[0]=30.
        expect(math.inf, (.5,0.,0.), (.15,0.,0.))
        expect(math.inf, (0.,0.,.7), (0.,0.,.21))
        expect(math.inf, (.06,0.,.10), (.06,0.,.10))
        percentage[0]=0.
        expect(math.inf, (.5,0.,0.), (0.,0.,0.))
        percentage[0]=30.
        percentage[0]=None
        pump(.7)
        expect(math.inf, (.5,0.,0.), (0.,0.,0.))
        percentage[0]=100.
        expect(math.inf, (.254,0.,0.), (.254,0.,0.))
        if depth_enabled:
            publish_depth[0] = False
            pump(.7)
            expect(math.inf, (.254,0.,0.), (0.,0.,0.))
            publish_depth[0] = True
            expect(math.inf, (.254,0.,0.), (.254,0.,0.))
        transition(4)
        pump(.1)
        count = len(received)
        send(math.inf, .127)
        pump(.2)
        assert len(received) == count
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.)
        log.close()
