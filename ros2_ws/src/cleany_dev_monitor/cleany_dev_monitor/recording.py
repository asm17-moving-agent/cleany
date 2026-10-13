"""Recording policy and bounded raw-message handoff; independent of UI leases."""
from collections import deque
from copy import deepcopy
import math
import re
import threading
import time

LABELS = {'motion': '위치·속도·미션', 'paths': '계획·제어 경로', 'costmaps': 'Costmap',
          'bt': 'Nav2 BT', 'safety': '안전 판단', 'logs': '로그',
          'lidar': 'LiDAR 원본', 'point_cloud': 'Depth point cloud 원본',
          'voxel_layer': 'Nav2 voxel layer 원본', 'voxel_world': 'Voxel world·장애물 메모리',
          'camera': '카메라 원본'}
RAW = {'lidar': '/scan', 'point_cloud': '/camera/head/depth/points',
       'voxel_layer': '/local_costmap/voxel_grid', 'voxel_world': '/obstacle_memory/voxels',
       'camera': '/camera/head/color/image_raw'}
TYPES = {'lidar': {'sensor_msgs/msg/LaserScan'}, 'point_cloud': {'sensor_msgs/msg/PointCloud2'},
         'voxel_layer': {'nav2_msgs/msg/VoxelGrid'}, 'voxel_world': {'sensor_msgs/msg/PointCloud2'},
         'camera': {'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'},
         'transforms': {'tf2_msgs/msg/TFMessage'}}


def defaults():
    return {'active': True, 'preset': 'basic', 'enabled': {k: k not in RAW for k in LABELS},
            'hz': {k: 2. for k in RAW}, 'topics': dict(RAW), 'limit_mib': 100}


def validate(value):
    if not isinstance(value, dict) or set(value) != set(defaults()):
        raise ValueError('녹화 설정 형식이 올바르지 않습니다.')
    if type(value['active']) is not bool or value['preset'] not in ('basic', 'obstacles', 'custom'):
        raise ValueError('녹화 상태 또는 프리셋이 올바르지 않습니다.')
    if any(not isinstance(value[key], dict) for key in ('enabled', 'hz', 'topics')):
        raise ValueError('녹화 항목·주기·토픽 형식이 올바르지 않습니다.')
    if set(value['enabled']) != set(LABELS) or any(type(v) is not bool for v in value['enabled'].values()):
        raise ValueError('녹화 항목을 모두 지정해야 합니다.')
    if set(value['hz']) != set(RAW) or set(value['topics']) != set(RAW):
        raise ValueError('원본 녹화 설정이 누락되었습니다.')
    for key in RAW:
        hz = value['hz'][key]
        if type(hz) not in (float, int) or not math.isfinite(hz) or not .1 <= hz <= 10:
            raise ValueError('기록 주기는 0.1~10 Hz 범위입니다.')
        if not isinstance(value['topics'][key], str) or not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_/]*', value['topics'][key]):
            raise ValueError('원본 토픽은 절대 ROS 토픽 이름이어야 합니다.')
    if len(set(value['topics'].values())) != len(RAW):
        raise ValueError('원본 녹화 토픽은 서로 달라야 합니다.')
    if type(value['limit_mib']) is not int or not 10 <= value['limit_mib'] <= 1024:
        raise ValueError('세션 용량은 10~1024 MiB 범위입니다.')
    return deepcopy(value)


def category(row):
    kind, key = row['kind'], row['key']
    if kind in ('config', 'graph', 'recording_policy', 'recording_status') or (kind == 'layer' and key == 'map'):
        return None  # Context is always retained.
    if kind == 'log':
        return 'logs'
    if kind == 'nav2_bt_tree' or 'bt' in key or (row['data'].get('kind') if isinstance(row['data'], dict) else None) == 'nav2_bt':
        return 'bt'
    if key in ('local_costmap', 'global_costmap') or 'costmap' in key:
        return 'costmaps'
    if key in ('path', 'local_path') or key.endswith(('/plan', '/local_plan')):
        return 'paths'
    if key in ('scan', '/scan'):
        return 'lidar'
    if key == 'depth_points' or '/depth/' in key:
        return 'point_cloud'
    if key in ('obstacle_memory', 'lidar_memory', 'depth_memory') or '/obstacle_memory/' in key:
        return 'voxel_world'
    if key in ('reactive_guard', 'voxel_guard', 'collision_state', 'coverage', 'risk_points', 'footprint', 'stop_polygon', 'slow_polygon') or any(s in key for s in ('guard/', 'collision_monitor', '/safety/', 'head_pan/coverage')):
        return 'safety'
    if '/image' in key:
        return 'camera'
    return 'motion'


def allowed(row, settings):
    key = next((k for k,v in settings['topics'].items() if row['key'] == v), None) or category(row)
    return row['kind'] in ('recording_policy','recording_status') or settings['active'] and (key is None or settings['enabled'][key])


class RecordingControls:
    def __init__(self, queue_limit=32*1024**2):
        self.lock = threading.Lock()
        self.settings = defaults()
        self.revision = 0
        self.queue = deque()
        self.queued_bytes = 0
        self.queue_limit = queue_limit
        self.last = {}
        self.stats = {}
        self.available = {}
        self.writer_error = ""

    def configure(self, settings):
        settings = validate(settings)
        with self.lock:
            self.settings = settings
            self.revision += 1
            self.last.clear()
            self.stats = {}

    def topics(self):
        with self.lock:
            if self.writer_error or not self.settings['active']:
                return set()
            topics = {v for k,v in self.settings['topics'].items() if self.settings['enabled'][k]}
            if self.settings['enabled']['paths']:
                topics.add('/local_plan')
            if self.settings['enabled']['voxel_world']:
                topics.update(('/obstacle_memory/grid', '/obstacle_memory/lidar', '/obstacle_memory/depth'))
            if any(self.settings['enabled'][k] for k in RAW):
                topics.update(('/tf', '/tf_static'))
            return topics

    def discover(self, topics):
        with self.lock:
            self.available = dict(topics)

    def reserve(self, topic):
        with self.lock:
            s = self.settings
            key = next((k for k,v in s['topics'].items() if v == topic), None)
            if topic in ('/tf','/tf_static') and any(s['enabled'][k] for k in RAW):
                key = 'transforms'
            if self.writer_error or not key or not s['active'] or key != 'transforms' and not s['enabled'][key]:
                return None
            now = time.monotonic()
            self.stats.setdefault(key, {})['received_at'] = time.time()
            hz = 30 if key == 'transforms' else s['hz'][key]
            if now-self.last.get(topic, -float('inf')) < 1/hz:
                return None
            self.last[topic] = now
            return key, self.revision

    def offer(self, ticket, topic, message_type, payload, ros_time):
        key, revision = ticket
        with self.lock:
            if revision != self.revision:
                return
            stat = self.stats.setdefault(key, {})
            if message_type not in TYPES[key]:
                stat['error'] = '지원하지 않는 ROS 메시지 타입'
                return
            if len(self.queue) >= 128 or len(payload) > 8*1024**2 or self.queued_bytes+len(payload) > self.queue_limit:
                stat['error'] = '원본 크기 또는 저장 대기 한도 초과'
                stat['dropped'] = stat.get('dropped', 0)+1
                return
            self.queue.append((key, topic, message_type, time.time(), ros_time, payload))
            self.queued_bytes += len(payload)

    def drain(self):
        with self.lock:
            batch = list(self.queue)
            self.queue.clear()
            self.queued_bytes = 0
            return batch

    def saved(self, key):
        with self.lock:
            stat = self.stats.setdefault(key, {})
            stat['count'] = stat.get('count', 0)+1

    def view(self, reason=''):
        with self.lock:
            statuses = {}
            s = self.settings
            for key in (*LABELS, 'transforms'):
                stat = dict(self.stats.get(key, {}))
                is_enabled = any(s['enabled'][k] for k in RAW) if key == 'transforms' else s['enabled'][key]
                state = '미녹화' if not is_enabled else '일시정지' if not s['active'] else '저장 중단' if reason else '저장 오류' if stat.get('error') else '기록 대기'
                if state == '기록 대기' and key == 'transforms':
                    state = '토픽 미수신' if not any(self.available.get(t) for t in ('/tf','/tf_static')) else '기록 중' if stat.get('count') else '수신 대기'
                elif state == '기록 대기' and key in RAW:
                    types = set(self.available.get(s['topics'][key], []))
                    state = '토픽 미수신' if not types else '타입 불일치' if not types <= TYPES[key] else '입력 중단' if stat.get('received_at') and time.time()-stat['received_at'] > max(3,2/s['hz'][key]) else '기록 중' if stat.get('count') else '수신 대기'
                elif state == '기록 대기':
                    state = '기록 중' if stat.get('count') else '수신 대기'
                statuses[key] = dict(stat, state=state)
            return {'settings': deepcopy(s), 'labels': LABELS, 'status': statuses, 'reason': reason,
                    'raw_format': 'ROS CDR; full message, sampled in time', 'revision': self.revision}
