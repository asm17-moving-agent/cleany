"""Run observation-only Cartographer map extension beside AMCL/Nav2."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from cartographer_ros_msgs.srv import StartTrajectory, FinishTrajectory, WriteState, GetTrajectoryStates
from ament_index_python.packages import get_package_share_directory


def relative_pose(x, y, yaw, origin):
    dx, dy = x-origin['x'], y-origin['y']
    c, s = math.cos(origin['yaw']), math.sin(origin['yaw'])
    return c*dx+s*dy, -s*dx+c*dy, yaw-origin['yaw']


def write_grid(grid, directory):
    """Write a map-server-compatible image; flip ROS rows into image order."""
    w, h = grid.info.width, grid.info.height
    pixels = bytes(205 if p < 0 else round(254*(100-p)/100) for p in grid.data)
    image = b''.join(pixels[y*w:(y+1)*w] for y in range(h-1, -1, -1))
    tmp = directory/'map.pgm.tmp'
    tmp.write_bytes(f'P5\n{w} {h}\n255\n'.encode()+image)
    tmp.replace(directory/'map.pgm')
    q = grid.info.origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    tmp = directory/'map.yaml.tmp'
    tmp.write_text(f'image: map.pgm\nresolution: {grid.info.resolution}\norigin: [{grid.info.origin.position.x}, {grid.info.origin.position.y}, {yaw}]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    tmp.replace(directory/'map.yaml')


class OdomContinuity:
    """Reject odometry rebases before they reach the extension trajectory."""
    def __init__(self, max_gap=2.0, max_step=.25, max_yaw_step=.35):
        self.limits = max_gap, max_step, max_yaw_step
        self.previous = None

    def check(self, stamp, received, x, y, angle):
        if self.previous is not None:
            old_stamp, old_received, ox, oy, oa = self.previous
            if stamp <= old_stamp:
                return 'Odometry time did not advance'
            if stamp-old_stamp > self.limits[0]:
                return 'Odometry input gap; start a new extension trajectory'
            if math.hypot(x-ox,y-oy) > self.limits[1]:
                return 'Odometry position jumped; possible origin reset'
            if abs(math.atan2(math.sin(angle-oa),math.cos(angle-oa))) > self.limits[2]:
                return 'Odometry heading jumped; possible origin reset'
        self.previous = stamp,received,x,y,angle
        return None


class Session(Node):
    def __init__(self, output, limits=(2.0, .25, .35)):
        super().__init__('map_extension_session')
        self.output = output
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tf_forward = self.create_publisher(TFMessage, "/map_extension/tf", 100)
        self.create_subscription(TFMessage, "/tf", self.tf_forward.publish, 100)
        self.scan_at = self.odom_at = self.pose_at = 0.0
        self.grid = None
        self.guard = OdomContinuity(*limits)
        self.fault = None
        self.odom_out = self.create_publisher(Odometry, "/map_extension/odom", 10)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan_at', time.monotonic()), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.receive_odom, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/map_extension/tracked_pose', lambda m: setattr(self, 'pose_at', time.monotonic()), 10)
        self.create_subscription(OccupancyGrid, '/map_extension/map', lambda m: setattr(self, 'grid', m), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def receive_odom(self, msg):
        if self.fault:
            return
        now = time.monotonic()
        p = msg.pose.pose
        q = p.orientation
        angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.fault = self.guard.check(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, now, p.position.x, p.position.y, angle)
        if self.fault:
            self.get_logger().error(self.fault)
        if not self.fault:
            self.odom_at = now
            self.odom_out.publish(msg)

    def call(self, kind, name, request, timeout=45):
        client = self.create_client(kind, '/map_extension/'+name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError('Service unavailable: '+name)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            result = future.result()
            if result is None or result.status.code != 0:
                raise RuntimeError(f'{name}: {result}')
            return result
        finally:
            self.destroy_client(client)

    def checkpoint(self, label):
        target = self.output/f'{label}.pbstream'
        tmp = self.output/f'.{label}.pbstream'
        req = WriteState.Request()
        req.filename = str(tmp)
        req.include_unfinished_submaps = True
        self.call(WriteState, 'write_state', req)
        tmp.replace(target)
        if self.grid is not None:
            write_grid(self.grid, self.output)
        print(f'Saved {target}', flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--origin-reader', type=Path, required=True)
    parser.add_argument('--odom-max-gap', type=float, default=2.0)
    parser.add_argument('--odom-max-step', type=float, default=.25)
    parser.add_argument('--odom-max-yaw-step', type=float, default=.35)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    seed = args.output/'seed.pbstream'
    shutil.copy2(args.source, seed)
    origin = json.loads(subprocess.check_output([str(args.origin_reader), str(seed)], text=True))
    share = Path(get_package_share_directory('cleany_navigation'))/'config'/'slam'
    config = args.output/'config'
    shutil.copytree(share, config)
    (config/'extension.lua').write_text('''include "cartographer_hardware_2d.lua"
options.publish_to_tf = false
options.publish_tracked_pose = true
options.pose_publish_period_sec = 0.1
options.trajectory_publish_period_sec = 1.0
options.submap_publish_period_sec = 1.0
MAP_BUILDER.num_background_threads = 2
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1
return options
''')
    rclpy.init()
    node = Session(args.output, (args.odom_max_gap,args.odom_max_step,args.odom_max_yaw_step))
    children, logs = [], []
    stopping = False
    trajectory = None
    def stop(signum, frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        deadline = time.monotonic()+20
        pose = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                pose = node.buffer.lookup_transform('map', 'base_link', Time())
                age = node.get_clock().now().nanoseconds*1e-9-(pose.header.stamp.sec+pose.header.stamp.nanosec*1e-9)
                if time.monotonic()-node.scan_at < 1 and time.monotonic()-node.odom_at < 1 and abs(age)<1:
                    break
            except Exception:
                pass
        else:
            raise RuntimeError('Fresh scan, odom and map->base_link required')
        q = pose.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        x, y, angle = relative_pose(pose.transform.translation.x, pose.transform.translation.y, yaw, origin)
        metadata = dict(source=str(args.source), sha256=hashlib.sha256(seed.read_bytes()).hexdigest(), origin=origin, initial_map_pose=[pose.transform.translation.x,pose.transform.translation.y,yaw], initial_relative_pose=[x,y,angle], namespace='/map_extension', publish_to_tf=False, started=time.time())
        (args.output/'session.json').write_text(json.dumps(metadata, indent=2))
        commands = [
          ['cartographer_node', '-configuration_directory', str(config), '-configuration_basename', 'extension.lua', '-load_state_filename', str(seed), '-load_frozen_state=true', '-start_trajectory_with_default_topics=false'],
          ['cartographer_occupancy_grid_node', '-resolution', '0.05', '-publish_period_sec', '2.0']]
        for command in commands:
            log = open(args.output/(command[0]+'.log'), 'w')
            logs.append(log)
            children.append(subprocess.Popen(['ros2','run','cartographer_ros',*command,'--ros-args','-r','__ns:=/map_extension','-r','scan:=/scan','-r','odom:=/map_extension/odom','-r','tf:=/map_extension/tf','-r','/tf:=/map_extension/tf','-r','tf_static:=/tf_static'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        states = node.call(GetTrajectoryStates,'get_trajectory_states',GetTrajectoryStates.Request())
        frozen = [tid for tid, state in zip(states.trajectory_states.trajectory_id, states.trajectory_states.trajectory_state) if state == 2]
        if not frozen:
            raise RuntimeError('No frozen reference trajectory loaded')
        pose = node.buffer.lookup_transform('map', 'base_link', Time())
        q = pose.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        x, y, angle = relative_pose(pose.transform.translation.x, pose.transform.translation.y, yaw, origin)
        metadata['initial_map_pose'] = [pose.transform.translation.x, pose.transform.translation.y, yaw]
        metadata['initial_relative_pose'] = [x, y, angle]
        req = StartTrajectory.Request()
        req.configuration_directory, req.configuration_basename = str(config), 'extension.lua'
        req.use_initial_pose = True
        req.relative_to_trajectory_id = frozen[0]
        req.initial_pose.position.x, req.initial_pose.position.y = x,y
        req.initial_pose.orientation.z, req.initial_pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
        trajectory = node.call(StartTrajectory,'start_trajectory',req).trajectory_id
        metadata['trajectory_id'] = trajectory
        (args.output/'session.json').write_text(json.dumps(metadata, indent=2))
        deadline = time.monotonic()+45
        while node.pose_at == 0 or node.grid is None:
            if node.fault:
                raise RuntimeError(node.fault)
            if any(p.poll() is not None for p in children) or time.monotonic()>deadline:
                raise RuntimeError('Extension did not receive live pose/map')
            rclpy.spin_once(node, timeout_sec=0.2)
        print(f'READY trajectory={trajectory} map=/map_extension/map output={args.output}', flush=True)
        node.checkpoint('latest')
        next_save = time.monotonic()+60
        while not stopping:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.fault or time.monotonic()-node.odom_at > args.odom_max_gap:
                raise RuntimeError(node.fault or 'Odometry input gap')
            if any(p.poll() is not None for p in children):
                raise RuntimeError('Cartographer child exited')
            if time.monotonic() >= next_save:
                node.checkpoint('latest')
                next_save = time.monotonic()+60
    finally:
        if trajectory is not None and all(p.poll() is None for p in children):
            try:
                req = FinishTrajectory.Request()
                req.trajectory_id = trajectory
                node.call(FinishTrajectory,'finish_trajectory',req)
                until = time.monotonic()+3
                while time.monotonic()<until:
                    rclpy.spin_once(node, timeout_sec=0.2)
                node.checkpoint('final')
            except Exception as exc:
                print(f'Final save failed: {exc}', flush=True)
        for p in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in children:
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGTERM)
                p.wait(timeout=10)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
