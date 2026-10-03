#!/usr/bin/env python3
"""Capture real Fortress renders/scans and optionally drive a bounded facility route.

Run inside the Humble environment after make build-gazebo. Each run owns its
ROS domain, Gazebo partition and child process groups; it never moves hardware.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from xml.etree import ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as ImageMessage, LaserScan, JointState

from cleany_gazebo_sim.gazebo_slam_experiment import (
    load_mount_profiles,
    write_sensor_tf_config,
)
from cleany_gazebo_sim.route_control import (
    Pose2D,
    RouteLimits,
    RouteTracker,
    waypoints_from_flat,
)
from cleany_gazebo_sim.world.facility_generator import materialize_facility_world
from cleany_gazebo_sim.world.facility_layout import load_facility_layout
from cleany_gazebo_sim.world.generator import (
    materialize_study_cafe_world,
    _add_office_chair,
)
from cleany_gazebo_sim.world.roly import add_roly_chair, load_roly_config


def camera(world, name, position, target, width=1280, height=900, fov=0.8):
    dx, dy, dz = (b - a for a, b in zip(position, target))
    pitch, yaw = -math.atan2(dz, math.hypot(dx, dy)), math.atan2(dy, dx)
    if math.hypot(dx, dy) < 1e-6:
        yaw = math.pi / 2
    model = ET.SubElement(world, 'model', name=f'review_{name}')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = ' '.join(map(str, (*position, 0.0, pitch, yaw)))
    sensor = ET.SubElement(
        ET.SubElement(model, 'link', name='camera'), 'sensor', name=name, type='camera'
    )
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = '1'
    ET.SubElement(sensor, 'topic').text = f'/review/{name}'
    c = ET.SubElement(sensor, 'camera')
    ET.SubElement(c, 'horizontal_fov').text = str(fov)
    image = ET.SubElement(c, 'image')
    for key, value in (('width', width), ('height', height), ('format', 'R8G8B8')):
        ET.SubElement(image, key).text = str(value)
    clip = ET.SubElement(c, 'clip')
    ET.SubElement(clip, 'near').text = '.05'
    ET.SubElement(clip, 'far').text = '100'


def stop(process):
    if process is None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2)
    except ProcessLookupError:
        pass
    finally:
        # ros2 launch may exit before a stalled Gazebo child. The process group
        # was created exclusively for this run, so reap its remaining children.
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


class Observer(Node):
    def __init__(self, names, output):
        super().__init__('facility_runtime_review')
        self.output = output
        self.images = set()
        self.pose = None
        self.odom_count = 0
        self.odom_sim = 0.0
        self.scan_count = 0
        self.joint_count = 0
        self.first_sim = None
        self.last_sim = None
        self.first_wall = None
        self.last_wall = None
        self.scan = None
        self.trace = []
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        for name in names:
            self.create_subscription(
                ImageMessage, f'/review/{name}', lambda m, n=name: self.image(m, n), 1
            )
        self.create_subscription(Odometry, '/ground_truth/odom', self.odom, 10)
        self.create_subscription(
            LaserScan, '/scan', self.laser, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState, '/joint_states', self.joints, qos_profile_sensor_data
        )

    def image(self, msg, name):
        if name in self.images:
            return
        encoding = {'rgb8': 'RGB', 'bgr8': 'BGR'}.get(msg.encoding.lower())
        if not encoding:
            raise ValueError(f'unsupported camera encoding {msg.encoding}')
        Image.frombytes(
            'RGB', (msg.width, msg.height), bytes(msg.data), 'raw', encoding, msg.step
        ).save(self.output / f'{name}.png')
        self.images.add(name)

    def odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = Pose2D(p.x, p.y, yaw)
        self.odom_count += 1
        self.odom_sim = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self.odom_count % 20 == 0:
            self.trace.append([p.x, p.y, yaw])

    def joints(self, msg):
        self.joint_count += bool(msg.name)

    def laser(self, msg):
        self.scan_count += 1
        self.scan = msg
        sim = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        wall = time.monotonic()
        if self.first_sim is None:
            self.first_sim, self.first_wall = sim, wall
        self.last_sim, self.last_wall = sim, wall


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', choices=['facility', 'chair'], default='facility')
    parser.add_argument('--chair-model', choices=['roly', 'legacy'], default='roly')
    parser.add_argument('--route', action='store_true')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--timeout', type=float, default=180)
    parser.add_argument('--physics-step', type=float, default=0.001)
    parser.add_argument('--measure-sim-seconds', type=float, default=8)
    parser.add_argument(
        '--lidar-height', type=float, choices=[0.165, 0.26, 0.45, 0.70], default=0.26
    )
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.route and args.scene != 'facility':
        parser.error('--route requires --scene facility')
    args.output.mkdir(parents=True, exist_ok=True)
    package = Path(get_package_share_directory('cleany_gazebo_sim'))
    layout = load_facility_layout(package / 'config/facility_18f/facility_layout.yaml')
    template = package / 'worlds/cleany_mecanum_fortress.sdf'
    world_path = args.output / 'review.sdf'
    profiles = load_mount_profiles(package / 'config/lidar_mount_profiles.yaml')
    profile = next(
        p
        for p in profiles.values()
        if abs(p.transform.translation[2] + 0.38 - args.lidar_height) < 1e-6
    )
    sensor_config = args.output / 'sensor_tf.yaml'
    write_sensor_tf_config(profile, sensor_config)
    generator = (
        materialize_facility_world
        if args.scene == 'facility'
        else materialize_study_cafe_world
    )
    generator(
        template,
        world_path,
        chair_model=args.chair_model,
        max_step_size=args.physics_step,
        lidar_translation=(0.16, 0.0, args.lidar_height - 0.38),
    )
    root = ET.parse(world_path).getroot()
    world = root.find('world')
    if args.scene == 'facility':
        bounds = layout.raw['floor_bounds']
        a, b = layout.world(bounds[:2]), layout.world(bounds[2:])
        center = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        camera(world, 'top', (*center, 40), (*center, 0), 1600, 1000, 0.9)
        camera(world, 'ground', (-10.5, -6.2, 1.6), (-12.5, 1.5, 1.1), fov=1.25)
        camera(world, 'ceiling', (-12, 0, 1.2), (-12, 0, 2.7), fov=1.2)
        names = ['top', 'ground', 'ceiling']
    else:
        for model in list(world.findall('model')):
            if model.get('name') not in ('ground_plane', 'cleany_mecanum'):
                world.remove(model)
        robot = world.find("model[@name='cleany_mecanum']")
        robot.find('pose').text = f'2 0 .38 0 0 {math.pi}'
        for size in world.findall(
            "model[@name='ground_plane']/link/*/geometry/plane/size"
        ):
            size.text = '8 8'
        if args.chair_model == 'roly':
            add_roly_chair(
                world,
                'review_chair',
                (0.0,) * 6,
                load_roly_config(package / 'config/furniture/roly_p1g210m.yaml'),
                args.output / 'roly_meshes',
            )
        else:
            _add_office_chair(world, 'review_chair', (0.0,) * 6)
        camera(world, 'front', (1.6, 1.5, 1.2), (0, 0, 0.48), fov=0.8)
        camera(world, 'side', (0, 2.3, 0.8), (0, 0, 0.48), fov=0.8)
        camera(world, 'rear', (-1.6, 1.5, 1.2), (0, 0, 0.48), fov=0.8)
        camera(world, 'face', (1.7, 0, 0.60), (0, 0, 0.50), fov=0.95)
        camera(world, 'top', (0, 0, 2.4), (0, 0, 0), fov=0.8)
        names = ['front', 'side', 'rear', 'face', 'top']
    if args.scene == 'chair':
        ground_visual = world.find("model[@name='ground_plane']/link/visual")
        for previous in list(ground_visual.findall('material')):
            ground_visual.remove(previous)
        material = ET.SubElement(ground_visual, 'material')
        ET.SubElement(material, 'ambient').text = '.65 .67 .68 1'
        ET.SubElement(material, 'diffuse').text = '.65 .67 .68 1'
    # GPU lidar observes visual geometry, not contact collision proxies.
    ET.ElementTree(root).write(world_path, encoding='unicode', xml_declaration=True)
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['IGN_PARTITION'] = f'cleany_facility_review_{os.getpid()}'
    os.environ['QT_QPA_PLATFORM'] = 'xcb'
    rclpy.init()
    node = Observer(names, args.output)
    log = (args.output / 'runtime.log').open('w')
    simulator = bridge = None
    reached = []
    try:
        simulator = subprocess.Popen(
            [
                'ros2',
                'launch',
                'cleany_gazebo_sim',
                'gazebo_fortress.launch.py',
                f'world:={world_path}',
                f'headless:={str(not args.gui).lower()}',
                'gui_render_engine:=ogre2',
                f'headless_rendering:={str(not args.gui).lower()}',
                'sensor_profile:=lidar_nav',
                f'sensor_config:={sensor_config}',
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        bridge = subprocess.Popen(
            [
                'ros2',
                'run',
                'ros_gz_bridge',
                'parameter_bridge',
                *[
                    f'/review/{name}@sensor_msgs/msg/Image[ignition.msgs.Image'
                    for name in names
                ],
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        tracker = (
            RouteTracker(
                waypoints_from_flat([v for p in layout.raw['route_xy'] for v in p]),
                RouteLimits(**layout.raw['route_control']),
            )
            if args.route
            else None
        )
        deadline, last_index = time.monotonic() + args.timeout, -1
        report_at = time.monotonic() + 15
        route_complete = False
        last_control_sim = -1.0
        while time.monotonic() < deadline:
            if simulator.poll() is not None:
                raise RuntimeError(f'Gazebo launch exited {simulator.returncode}')
            rclpy.spin_once(node, timeout_sec=0.05)
            if tracker and node.pose and node.odom_sim - last_control_sim >= 0.05:
                last_control_sim = node.odom_sim
                command = tracker.command(node.pose)
                twist = Twist()
                twist.linear.x, twist.angular.z = command.linear_x, command.angular_z
                node.publisher.publish(twist)
                if tracker.waypoint_index != last_index:
                    last_index = tracker.waypoint_index
                    reached.append(last_index)
                    print(f'waypoint {last_index}: {node.pose}', flush=True)
                route_complete = command.completed
            if node.last_wall is not None and time.monotonic() - node.last_wall > 30:
                print('sensor clock stalled for 30 wall seconds', flush=True)
                break
            if time.monotonic() >= report_at:
                print(
                    f'scans={node.scan_count} sim={node.last_sim} pose={node.pose}',
                    flush=True,
                )
                report_at = time.monotonic() + 15
            sim_span = (
                (node.last_sim - node.first_sim) if node.first_sim is not None else 0
            )
            if (
                node.images == set(names)
                and node.scan_count > 5
                and sim_span >= args.measure_sim_seconds
                and (not args.route or route_complete)
            ):
                break
        complete = (
            node.images == set(names)
            and node.scan_count > 5
            and sim_span >= args.measure_sim_seconds
            and (not args.route or route_complete)
        )
        elapsed = node.last_wall - node.first_wall if node.first_wall is not None else 0
        sim_span = node.last_sim - node.first_sim if node.first_sim is not None else 0
        result = {
            'passed': complete,
            'scene': args.scene,
            'chair_model': args.chair_model,
            'physics_step_s': args.physics_step,
            'lidar_height_m': args.lidar_height,
            'images': sorted(node.images),
            'scan_count': node.scan_count,
            'odom_count': node.odom_count,
            'joint_state_count': node.joint_count,
            'sim_span_s': sim_span,
            'wall_span_s': elapsed,
            'measured_rtf': sim_span / elapsed if elapsed else None,
            'scan_sim_hz': (node.scan_count - 1) / sim_span if sim_span else None,
            'route_complete': route_complete if args.route else None,
            'reached_indices': reached,
            'trace': node.trace,
        }
        if node.scan:
            result['scan'] = {
                'angle_min': node.scan.angle_min,
                'angle_increment': node.scan.angle_increment,
                'ranges': [
                    float(v) if math.isfinite(v) else None for v in node.scan.ranges
                ],
            }
        (args.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(
            json.dumps(
                {k: v for k, v in result.items() if k not in ('scan', 'trace')},
                indent=2,
            ),
            flush=True,
        )
        if not complete:
            raise TimeoutError(
                'runtime verification incomplete; see result.json and runtime.log'
            )
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        stop(bridge)
        stop(simulator)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        log.close()


if __name__ == '__main__':
    main()
