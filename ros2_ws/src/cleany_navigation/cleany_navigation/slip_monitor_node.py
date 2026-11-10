"""독립적인 LiDAR 및 바퀴 움직임 관측의 ROS 어댑터입니다."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
import yaml

from .lidar_scan_matcher import LidarScanMatcher
from .slip_estimation import Settings
from .slip_monitor import ObservationHistory, ScanHistory, evaluate_pair
from .slip_context import CommandHistory, json_safe


class SlipMonitor(Node):
    """시각이 있는 움직임 불일치를 발행하며 이동 명령은 보내지 않습니다.

    Subscriptions:
        scan_raw (LaserScan), odom (Odometry): header로 정렬한 측정값.
            sensor_data QoS이며 이름은 파라미터 기본값입니다.
        nav2/cmd_vel, cmd_vel (Twist): 요청 및 최종 속도 컨텍스트.
        map (OccupancyGrid): 기록 위치의 지도 식별 정보. transient_local.
    Publishers:
        navigation/slip (String): JSON 상태와 비교 시각, 5Hz.
    TF:
        스캔 장착 변환과 과거 map -> base_link를 읽으며 발행하지 않습니다.
    Notes:
        백그라운드 정합 작업 하나와 크기 제한 이력을 사용하며 바퀴 사전값을 넣지 않습니다.
        반복 heartbeat는 추가 지속성 근거가 되지 않습니다.
    """

    def __init__(self, **kwargs):
        super().__init__('slip_monitor', **kwargs)
        settings = {k: self.declare_parameter(k, v).value for k, v in asdict(Settings()).items()}
        self.config = Settings(**settings)
        self.history = ScanHistory(self.config)
        self.observations = ObservationHistory(self.config)
        self.base = self.declare_parameter('base_frame', 'base_link').value
        mask_default = str(Path(get_package_share_directory('cleany_navigation'))/'config/lidar_self_mask_cad.yaml')
        self.boxes = yaml.safe_load(Path(self.declare_parameter('mask_file', mask_default).value).read_text())['boxes']
        self.max_age = float(self.declare_parameter('maximum_input_age_s', .5).value)
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.map_revision = None
        self.recording_context = None
        age = float(self.declare_parameter('command_context_max_age_s', .3).value)
        self.commands = {name: CommandHistory(age) for name in ('requested', 'output')}
        self.command_topics = {
            'requested': self.declare_parameter('requested_command_topic', 'nav2/cmd_vel').value,
            'output': self.declare_parameter('output_command_topic', 'cmd_vel').value,
        }
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(String, 'navigation/slip', 1)
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.mount = None
        self.frame = None
        self.reason = 'warming_up'
        self.matcher = LidarScanMatcher(voxel_size_m=self.config.voxel_m, max_iterations=50)
        self.create_subscription(LaserScan, self.declare_parameter('scan_topic', 'scan_raw').value,
                                 self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.declare_parameter('odom_topic', 'odom').value,
                                 self.on_odom, qos_profile_sensor_data)
        for name, topic in self.command_topics.items():
            self.create_subscription(Twist, topic, lambda msg, role=name: self.on_command(role, msg),
                                     qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, self.declare_parameter('map_topic', 'map').value,
                                 self.on_map, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.05, self.work)
        self.create_timer(.2, self.publish)

    def stamp(self, header) -> float:
        return header.stamp.sec+header.stamp.nanosec*1e-9

    def on_command(self, role, message):
        self.commands[role].add(self.get_clock().now().nanoseconds/1e9,
                               (message.linear.x, message.linear.y, message.angular.z))

    def on_map(self, message):
        # 발행 시각이 아닌 기하와 셀을 해시하며 컨텍스트는 제어를 바꾸지 않습니다.
        p, q = message.info.origin.position, message.info.origin.orientation
        metadata = [message.header.frame_id, message.info.width, message.info.height,
                    message.info.resolution, p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        digest = hashlib.sha256(json.dumps(metadata).encode())
        digest.update(bytes(memoryview(message.data)))
        self.map_revision = digest.hexdigest()

    def observation_context(self, observation):
        stamp = observation['t1']
        pose = None
        reason = ''
        try:
            transform = self.buffer.lookup_transform(
                self.map_frame, self.base, Time(nanoseconds=round(stamp*1e9)))
            p, q = transform.transform.translation, transform.transform.rotation
            pose = [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
        except Exception:
            reason = 'historical_transform_unavailable'
        return dict(at=stamp, frame=self.map_frame, pose=pose, pose_reason=reason,
                    map_revision=self.map_revision,
                    command_time_basis='ros_receive_time',
                    commands={role: dict(history.at(stamp), topic=self.command_topics[role])
                              for role, history in self.commands.items()})

    def on_odom(self, message):
        if message.header.frame_id != 'odom' or message.child_frame_id != self.base:
            self.reason = 'odometry_frame_mismatch'
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        self.history.add_odom([self.stamp(message.header), p.x, p.y,
                              math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))])

    def on_scan(self, message):
        try:
            transform = self.buffer.lookup_transform(self.base, message.header.frame_id, Time())
            q, p = transform.transform.rotation, transform.transform.translation
            mount = np.array([p.x, p.y, math.atan2(2*q.w*q.z, 1-2*q.z*q.z)])
            if abs(q.x)+abs(q.y) > 1e-5:
                raise ValueError('non_planar_scan_transform')
            if self.mount is not None and (self.frame != message.header.frame_id or not np.allclose(self.mount, mount)):
                raise ValueError('scan_transform_changed')
            self.mount, self.frame = mount, message.header.frame_id
            ranges = np.array(message.ranges)
            angles = message.angle_min+np.arange(len(ranges))*message.angle_increment
            good = np.isfinite(ranges) & (ranges >= message.range_min) & (ranges <= message.range_max)
            points = np.column_stack((ranges[good]*np.cos(angles[good]), ranges[good]*np.sin(angles[good])))
            self.history.add_scan(self.stamp(message.header)+message.scan_time/2, message.scan_time, points)
        except Exception as error:
            self.reason = str(error)

    def work(self):
        if self.future is not None:
            if not self.future.done():
                return
            try:
                before = self.observations.last_start
                observation = self.observations.update(self.future.result())
                if self.observations.last_start > before:
                    self.recording_context = self.observation_context(observation)
            except Exception as error:
                self.reason = 'matcher_error: '+str(error)
            self.future = None
        if self.mount is None:
            return
        pair = self.history.pair()
        if pair is not None:
            self.future = self.pool.submit(evaluate_pair, *pair, self.mount.copy(), self.boxes, self.config, self.matcher)

    def publish(self):
        observation = self.observations.observation
        keys = ('state', 't0', 't1', 'consecutive', 'kind', 'reason', 'translation_error_m',
                'translation_error_fraction', 'rotation_error_rad', 'wheel_delta', 'lidar_delta')
        result = {k: v for k, v in observation.items() if k in keys}
        result['context'] = self.recording_context
        now = self.get_clock().now().nanoseconds/1e9
        fresh = (self.history.scans and self.history.odom
                 and 0 <= now-self.history.scans[-1][0] <= self.max_age
                 and 0 <= now-self.history.odom[-1][0] <= self.max_age)
        if not fresh:
            result.update(state='unknown', reason='sensor_stale_or_warming: '+self.reason)
        # C++ 소비자가 잘못된 숫자 값을 수락하지 못하게 합니다.
        self.publisher.publish(String(data=json.dumps(json_safe(result), allow_nan=False)))

    def destroy_node(self):
        self.pool.shutdown(wait=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SlipMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
