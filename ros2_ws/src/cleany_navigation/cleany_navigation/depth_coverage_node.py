"""Shadow-only whole-body coverage observer. It cannot publish velocity or pan."""
from __future__ import annotations
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from cleany_navigation.body_geometry import load_body_boxes
from cleany_navigation.depth_clearance import decode_depth_image, rectified_intrinsics
from cleany_navigation.depth_coverage import DepthView, SweptDepthCoverage
from cleany_navigation.voxel_guard import GuardConfig
from cleany_navigation.voxel_guard_node import pose_matrix, twist_vector


class DepthCoverageNode(Node):
    """Measure whether recent depth observations cover the predicted body sweep.

    Subscriptions:
        camera/head/depth/image_raw (sensor_msgs/msg/Image) and
        camera/head/depth/camera_info (sensor_msgs/msg/CameraInfo): Depth and calibration;
            configurable defaults.
        nav2/cmd_vel (geometry_msgs/msg/Twist): Requested motion.
        odom (nav_msgs/msg/Odometry): Measured motion.

    Publishers:
        head_pan/coverage (std_msgs/msg/String): JSON coverage decision and evidence.

    TF:
        Reads sensor and base transforms in fixed_frame, default odom.

    QoS:
        Depth, calibration and odometry use sensor_data.

    Notes:
        Requires a body profile. This observer reports coverage; it does not
        publish a gated velocity command.
    """
    def __init__(self):
        super().__init__('depth_coverage')
        defaults = dict(profile='', allow_simulation_limits=False, fixed_frame='odom', base_frame='base_link',
                        optical_frame='head_camera_depth_optical_frame', input_topic='camera/head/depth/image_raw',
                        camera_info_topic='camera/head/depth/camera_info', source_timeout_s=.5,
                        update_period_s=.2, processing_budget_s=.15, cell_size=.1, max_cells=20000, pose_jump_m=.25, pose_jump_rad=.3,
                        **vars(GuardConfig()))
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        boxes = load_body_boxes(Path(get_package_share_directory('cleany_description')), Path(self.p['profile']),
                               allow_simulation_limits=self.p['allow_simulation_limits'])
        self.coverage = SweptDepthCoverage(boxes, GuardConfig(**{k: self.p[k] for k in vars(GuardConfig())}),
                                          cell_size=self.p['cell_size'], timeout=self.p['source_timeout_s'],
                                          max_cells=self.p['max_cells'], pose_jump_m=self.p['pose_jump_m'],
                                          pose_jump_rad=self.p['pose_jump_rad'])
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer, self)
        self.camera = self.pending = self.odom = None
        self.command = np.zeros(3)
        self.command_wall = self.depth_wall = self.odom_wall = -np.inf
        self.clock_stamp = None
        self.clock_wall = time.monotonic()
        self.state = self.create_publisher(String, 'head_pan/coverage', 10)
        self.create_subscription(CameraInfo, self.p['camera_info_topic'], self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.p['input_topic'], self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Twist, 'nav2/cmd_vel', self.on_command, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_timer(self.p['update_period_s'], self.update, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_info(self, msg):
        self.camera = msg

    def on_depth(self, msg):
        self.pending, self.depth_wall = msg, time.monotonic()

    def on_command(self, msg):
        try:
            self.command = twist_vector(msg)
            self.command_wall = time.monotonic()
        except ValueError:
            self.command_wall = -np.inf

    def on_odom(self, msg):
        self.odom, self.odom_wall = msg, time.monotonic()

    def update(self):
        started = time.monotonic()
        now = self.get_clock().now().nanoseconds/1e9
        report = dict(covered=False, reason='UNAVAILABLE')
        try:
            if self.clock_stamp is not None and now < self.clock_stamp:
                self.coverage.clear(); self.pending = None
            if self.clock_stamp != now:
                self.clock_stamp, self.clock_wall = now, started
            if any(started-t > self.p['source_timeout_s'] for t in
                   (self.command_wall, self.depth_wall, self.odom_wall, self.clock_wall)):
                self.coverage.clear()
                raise ValueError('WALL_OR_CLOCK_TIMEOUT')
            if self.odom is None or self.odom.child_frame_id != self.p['base_frame']:
                raise ValueError('INVALID_ODOMETRY')
            odom_stamp = Time.from_msg(self.odom.header.stamp).nanoseconds/1e9
            if not 0 <= now-odom_stamp <= self.p['source_timeout_s']:
                raise ValueError('STALE_ODOMETRY')
            msg, camera = self.pending, self.camera
            if msg is not None and camera is not None:
                when = Time.from_msg(msg.header.stamp)
                if not 0 <= now-when.nanoseconds/1e9 <= self.p['source_timeout_s']:
                    self.pending = None
                elif self.buffer.can_transform(self.p['fixed_frame'], msg.header.frame_id, when):
                    self.pending = None
                    if (msg.width, msg.height, msg.header.frame_id) != (camera.width, camera.height, camera.header.frame_id) or msg.header.frame_id != self.p['optical_frame']:
                        raise ValueError('CAMERA_MISMATCH')
                    if not np.isfinite(camera.d).all() or any(abs(v) > 1e-9 for v in camera.d):
                        raise ValueError('UNRECTIFIED_DEPTH')
                    intrinsics = rectified_intrinsics(camera.k, camera.p)
                    if intrinsics is None:
                        raise ValueError('INVALID_PROJECTION')
                    pose = self.buffer.lookup_transform(self.p['fixed_frame'], msg.header.frame_id, when)
                    self.coverage.observe(DepthView(when.nanoseconds/1e9,
                        decode_depth_image(msg.data, msg.width, msg.height, msg.step, msg.encoding, msg.is_bigendian),
                        intrinsics, pose_matrix(pose.transform)))
            base = self.buffer.lookup_transform(self.p['fixed_frame'], self.p['base_frame'], Time())
            if not 0 <= now-Time.from_msg(base.header.stamp).nanoseconds/1e9 <= self.p['source_timeout_s']:
                raise ValueError('STALE_BASE_TRANSFORM')
            report = self.coverage.evaluate(now, pose_matrix(base.transform), self.command, twist_vector(self.odom.twist.twist))
        except (ValueError, TransformException) as error:
            report['reason'] = str(error)
        elapsed = time.monotonic()-started
        if elapsed > self.p['processing_budget_s']:
            report.update(covered=False, reason='PROCESSING_OVERRUN')
        report.update(stamp_s=now, sim_s=now, mode='shadow', motion_authority=False, processing_ms=elapsed*1000)
        self.state.publish(String(data=json.dumps(report)))


def main(args=None):
    rclpy.init(args=args)
    node = DepthCoverageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
