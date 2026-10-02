"""Real-clock ROS integration with a synthetic backend, without Gazebo dependencies.

Opt in with make test-navigation-runtime. Fixtures are test assumptions, not a
physical robot calibration. Every child process is shut down on failure.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest
import yaml

if os.environ.get('CLEANY_RUN_NAVIGATION_RUNTIME') != '1':
    pytest.skip('Opt-in ROS runtime test', allow_module_level=True)

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from cleany_navigation.runtime_config import load_parameters


class Backend(Node):
    def __init__(self, simulated_clock=False):
        super().__init__('synthetic_navigation_backend')
        self.sensors_enabled = True
        self.simulated_clock = simulated_clock
        self.clock_enabled = True
        self.last_stamp = self.get_clock().now().to_msg()
        self.clock_publisher = self.create_publisher(Clock, 'clock', 10)
        self.obstacle = False
        self.commands = {}
        self.states = {}
        self.grids = {}
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic in ('obstacle_memory/grid', 'global_costmap/costmap'):
            self.create_subscription(OccupancyGrid, topic,
                lambda msg, topic=topic: self.grids.__setitem__(topic, msg), durable)
        self.sensor_publishers = {name: self.create_publisher(kind, name, qos_profile_sensor_data)
            for name, kind in [('odom', Odometry), ('scan', LaserScan),
                ('joint_states', JointState), ('camera/head/depth/camera_info', CameraInfo),
                ('camera/head/depth/image_raw', Image)]}
        self.initial = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        for name in ('nav2/cmd_vel', 'pan_ready/cmd_vel', 'safety_2d/cmd_vel', 'cmd_vel'):
            self.create_subscription(Twist, name, lambda msg, name=name: self.commands.__setitem__(name, msg), 10)
        for name in ('head_pan/state', 'body_guard_3d/state'):
            self.create_subscription(String, name, lambda msg, name=name: self.states.__setitem__(name, json.loads(msg.data)), 10)
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        transforms = []
        for child, z, quaternion, parent in [
            ('laser_frame', 0., (0., 0., 0., 1.), 'base_link'),
            ('head_camera_depth_frame', .6, (0., 0., 0., 1.), 'base_link'),
            ('head_camera_depth_optical_frame', 0., (-.5, .5, -.5, .5), 'head_camera_depth_frame')]:
            tf = TransformStamped()
            tf.header.frame_id, tf.child_frame_id = parent, child
            tf.transform.translation.z = z
            (tf.transform.rotation.x, tf.transform.rotation.y,
             tf.transform.rotation.z, tf.transform.rotation.w) = quaternion
            transforms.append(tf)
        self.static.sendTransform(transforms)
        self.create_timer(.05, self.publish_backend)

    def publish_backend(self):
        stamp = self.get_clock().now().to_msg() if self.clock_enabled else self.last_stamp
        self.last_stamp = stamp
        if self.simulated_clock and self.clock_enabled:
            self.clock_publisher.publish(Clock(clock=stamp))
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'odom', 'base_link'
        tf.transform.rotation.w = 1.
        self.tf.sendTransform(tf)
        if not self.sensors_enabled:
            return
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_link'
        odom.pose.pose.orientation.w = 1.
        self.sensor_publishers['odom'].publish(odom)
        joints = JointState()
        joints.header.stamp, joints.name, joints.position, joints.velocity = stamp, ['head_pan_joint'], [0.], [0.]
        self.sensor_publishers['joint_states'].publish(joints)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'laser_frame'
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi/180
        scan.range_min, scan.range_max = .05, 12.
        scan.ranges = [3./max(abs(math.cos(scan.angle_min+i*scan.angle_increment)),
                            abs(math.sin(scan.angle_min+i*scan.angle_increment))) for i in range(361)]
        if self.obstacle:
            for index in range(175, 186):
                scan.ranges[index] = .3
        self.sensor_publishers['scan'].publish(scan)
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = stamp, 'head_camera_depth_optical_frame'
        info.width, info.height = 16, 12
        info.k = [20., 0., 7.5, 0., 20., 5.5, 0., 0., 1.]
        info.p = [20., 0., 7.5, 0., 0., 20., 5.5, 0., 0., 0., 1., 0.]
        self.sensor_publishers['camera/head/depth/camera_info'].publish(info)
        image = Image()
        image.header = info.header
        image.width, image.height, image.step, image.encoding = 16, 12, 64, '32FC1'
        image.data = np.full((12, 16), 5.2, dtype='<f4').tobytes()
        self.sensor_publishers['camera/head/depth/image_raw'].publish(image)

    def initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id, msg.header.stamp = 'map', self.get_clock().now().to_msg()
        msg.pose.pose.orientation.w = 1.
        msg.pose.covariance[0] = msg.pose.covariance[7] = .001
        msg.pose.covariance[35] = .001
        self.initial.publish(msg)


def moving(msg):
    return msg is not None and max(abs(msg.linear.x), abs(msg.linear.y), abs(msg.angular.z)) > .001


def near_origin_clear(grid):
    if grid is None:
        return False
    y, x = np.indices((grid.info.height, grid.info.width))
    x = grid.info.origin.position.x+(x+.5)*grid.info.resolution
    y = grid.info.origin.position.y+(y+.5)*grid.info.resolution
    occupied = np.array(grid.data).reshape(x.shape) == 100
    return not np.any(occupied & (x*x+y*y < .8**2))


@pytest.mark.parametrize('simulated_clock', [False, True])
def test_goal_serial_guard_obstacle_cancel_and_missing_sources(tmp_path, monkeypatch, simulated_clock):
    # Separate cases so DDS discovery from a just-shut-down lifecycle server
    # cannot answer the next case's identically named service clients.
    base_domain = int(os.environ.get('CLEANY_TEST_ROS_DOMAIN_BASE', '187'))
    monkeypatch.setenv('ROS_DOMAIN_ID', str(base_domain+2 if simulated_clock else base_domain))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    root = Path(__file__).resolve().parents[1]
    pixels = np.full((124, 124), 254, dtype=np.uint8)
    pixels[:2] = pixels[-2:] = 0
    pixels[:, :2] = pixels[:, -2:] = 0
    (tmp_path/'map.pgm').write_bytes(b'P5\n124 124\n255\n'+pixels.tobytes())
    (tmp_path/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', resolution=.05,
        origin=[-3.1, -3.1, 0.], negate=0, occupied_thresh=.65, free_thresh=.2)))
    from ament_index_python.packages import get_package_share_directory
    import xacro
    from xml.etree import ElementTree as ET
    description = Path(get_package_share_directory('cleany_description'))
    urdf = ET.fromstring(xacro.process_file(str(description/'urdf/cleany.urdf.xacro')).toxml())
    joints = {joint.get('name'): 0. for joint in urdf.findall('joint')
              if joint.get('type') not in ('fixed',) and not joint.get('name').endswith('_wheel_joint')}
    (tmp_path/'body.yaml').write_text(yaml.safe_dump({'park_joints': joints, 'dynamic_pan': True}))
    robot = load_parameters(root/'config/nav2_safety.yaml')
    for name in ('local_costmap', 'global_costmap'):
        robot[name][name]['ros__parameters']['obstacle_layer'] = {
            'min_obstacle_height': -.33, 'scan': {'min_obstacle_height': -.33}}
    robot['voxel_guard'] = {'ros__parameters': dict(profile=str(tmp_path/'body.yaml'),
        max_speed=.15, max_yaw_rate=.3, reaction_s=.25, linear_deceleration=.4,
        angular_deceleration=.6, processing_budget_s=.5)}
    robot['obstacle_memory'] = {'ros__parameters': {'floor_z': -.38}}
    robot['depth_clearance']['ros__parameters'].update(camera_frame='head_camera_depth_frame',
        optical_frame='head_camera_depth_optical_frame')
    (tmp_path/'robot.yaml').write_text(yaml.safe_dump(robot))
    log = tmp_path/'navigation.log'
    process = None
    rclpy.init()
    backend = Backend(simulated_clock)
    action = ActionClient(backend, NavigateToPose, 'navigate_to_pose')
    clients = {name: backend.create_client(GetState, name+'/get_state')
        for name in ('amcl', 'controller_server', 'planner_server', 'bt_navigator', 'collision_monitor')}

    def wait(predicate, seconds=30.):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(backend, timeout_sec=.03)
            if predicate():
                return
            if process is not None and process.poll() is not None:
                break
        pytest.fail(f'Runtime condition timed out. States: {backend.states}\n'+log.read_text()[-14000:])

    try:
        with log.open('w') as stream:
            process = subprocess.Popen(['ros2', 'launch', 'cleany_navigation', 'navigation.launch.py',
                f'map:={tmp_path}/map.yaml', f'robot_params_file:={tmp_path}/robot.yaml',
                f'use_sim_time:={str(simulated_clock).lower()}'],
                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            for name, client in clients.items():
                wait(lambda client=client: client.service_is_ready(), 60.)
                def active(client=client):
                    request = client.call_async(GetState.Request())
                    wait(request.done, 5.)
                    return request.result().current_state.id == 3
                wait(active, 60.)
                if name == 'amcl':
                    backend.initial_pose()
            wait(lambda: action.server_is_ready())
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id, goal.pose.header.stamp = 'map', backend.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.orientation.w = .8, 1.
            future = action.send_goal_async(goal)
            wait(future.done)
            handle = future.result()
            assert handle.accepted, log.read_text()[-14000:]
            wait(lambda: moving(backend.commands.get('cmd_vel')), 40.)
            for name in ('nav2/cmd_vel', 'pan_ready/cmd_vel', 'safety_2d/cmd_vel'):
                assert moving(backend.commands.get(name)), (name, backend.states)
            assert backend.count_publishers('cmd_vel') == 1
            assert all('gazebo' not in name for name in backend.get_node_names())
            backend.obstacle = True
            wait(lambda: not moving(backend.commands.get('safety_2d/cmd_vel')), 3.)
            wait(lambda: not moving(backend.commands.get('cmd_vel')), 3.)
            cancel = handle.cancel_goal_async()
            wait(cancel.done)
            assert cancel.result().goals_canceling
            result = handle.get_result_async()
            wait(result.done, 8.)
            assert result.result().status == 5  # STATUS_CANCELED
            wait(lambda: not moving(backend.commands.get('cmd_vel')), 3.)
            backend.obstacle = False
            wait(lambda: near_origin_clear(backend.grids.get('obstacle_memory/grid')), 8.)
            cleared = backend.grids['obstacle_memory/grid'].header.stamp
            def costmap_cleared():
                grid = backend.grids.get('global_costmap/costmap')
                return near_origin_clear(grid) and (
                    (grid.header.stamp.sec, grid.header.stamp.nanosec) >= (cleared.sec, cleared.nanosec))
            wait(costmap_cleared, 8.)
            # Start a new action after obstacle clearing; no automatic recovery
            # timing is assumed after intentionally disconnecting a source.
            future = action.send_goal_async(goal)
            wait(future.done)
            second = future.result()
            assert second.accepted
            wait(lambda: moving(backend.commands.get('cmd_vel')), 20.)
            if simulated_clock:
                backend.clock_enabled = False
            else:
                backend.sensors_enabled = False
            expected_reason = 'CLOCK_TIMEOUT' if simulated_clock else 'WALL_TIMEOUT'
            wait(lambda: backend.states.get('head_pan/state', {}).get('reason') == expected_reason, 3.)
            wait(lambda: not moving(backend.commands.get('cmd_vel')), 3.)
            if not simulated_clock:
                wait(lambda: backend.states.get('body_guard_3d/state', {}).get('decision') == 'STOP_UNAVAILABLE', 3.)
            backend.clock_enabled = True
            backend.sensors_enabled = True
            cancel = second.cancel_goal_async()
            wait(cancel.done)
            assert cancel.result().goals_canceling
            wait(second.get_result_async().done, 8.)
            wait(lambda: not moving(backend.commands.get('cmd_vel')), 3.)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=12.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.)
        action.destroy()
        backend.destroy_node()
        rclpy.shutdown()
