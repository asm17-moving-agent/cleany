"""Recording policy and bounded raw-message handoff; independent of UI leases."""
import math
import re
import threading
import time
from collections import deque
from contextlib import contextmanager
from copy import deepcopy
from dataclasses import dataclass

from .sources import (
    SCHEMA,
    raw_maximum_references,
    resolved,
    validate_source_enabled,
    validate_sources,
)

LABELS = {'slam': 'SLAM 서브맵, 스캔, 그래프', 'resources': '호스트 리소스', 'motion': '위치, 속도, 미션', 'paths': '계획, 제어 경로', 'costmaps': 'Costmap',
          'bt': 'Nav2 BT', 'safety': '안전 판단', 'logs': '로그',
          'lidar': 'LiDAR 원본', 'point_cloud': 'Depth point cloud 원본',
          'voxel_layer': 'Nav2 voxel layer 원본', 'voxel_world': 'Voxel world, 장애물 메모리',
          'camera': '카메라 원본'}
RAW = {'lidar': '/scan', 'point_cloud': '/camera/head/depth/points',
       'voxel_layer': '/local_costmap/voxel_grid', 'voxel_world': '/obstacle_memory/voxels',
       'camera': '/camera/head/color/image_raw'}
TYPES = {'lidar': {'sensor_msgs/msg/LaserScan'}, 'point_cloud': {'sensor_msgs/msg/PointCloud2'},
         'voxel_layer': {'nav2_msgs/msg/VoxelGrid'}, 'voxel_world': {'sensor_msgs/msg/PointCloud2'},
         'camera': {'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'},
         'transforms': {'tf2_msgs/msg/TFMessage'}}
TYPES.update(camera_info={'sensor_msgs/msg/CameraInfo'}, clock={'rosgraph_msgs/msg/Clock'})

# Native visualization inputs accompany the corresponding processed records.
# Keep this allowlist separate from heavy, independently selected sensor streams.
CONTEXT = {
    'map_topic': ('costmaps', 'nav_msgs/msg/OccupancyGrid'),
    'local_costmap_topic': ('costmaps', 'nav_msgs/msg/OccupancyGrid'),
    'global_costmap_topic': ('costmaps', 'nav_msgs/msg/OccupancyGrid'),
    'path_topic': ('paths', 'nav_msgs/msg/Path'),
    'local_path_topic': ('paths', 'nav_msgs/msg/Path'),
    'odometry_topic': ('motion', 'nav_msgs/msg/Odometry'),
    'ground_truth_topic': ('motion', 'nav_msgs/msg/Odometry'),
    'amcl_pose_topic': ('motion', 'geometry_msgs/msg/PoseWithCovarianceStamped'),
    'footprint_topic': ('motion', 'geometry_msgs/msg/PolygonStamped'),
}


@dataclass(frozen=True)
class RawMessage:
    category: str
    topic: str
    message_type: str
    received: float
    ros_time: float | None
    payload: bytes
    timestamp_ns: int
    reliable: bool = False
    latched: bool = False


def observation_policies(config):
    """Observation bounds, distinct from raw CDR recording rates."""
    interval = config.get('resource_interval_s', 1.0)
    repeat = config.get('bt_event_repeat_interval_s', 1.0)
    return {
        'slam': {'summary': 'SLAM 레이어 최대 2 Hz, 서브맵 버전별 이미지 보존', 'detail': '서브맵 이미지와 위치를 함께 기록합니다. 버전, 위치 metadata와 이미지는 재생 일관성을 위해 주기 제한 없이 저장합니다.'},
        'resources': {'summary': f'수집 설정 {interval:g}초 ({1 / interval:g} Hz)',
                      'detail': '새 호스트 표본을 저장합니다. 실제 간격에는 수집 소요시간이 더해집니다.'},
        'motion': {'summary': '위치 최대 10 Hz, 주행 요약 5 Hz',
                   'detail': '미션 상태는 발행 주기에 따라 수신합니다. 전이, 결과는 이벤트별로 기록합니다.'},
        'paths': {'summary': '토픽별 최대 10 Hz, 동일 경로 중복 제외',
                  'detail': '경로 수신 시 관측합니다. 동일한 경로 레이어는 다시 저장하지 않습니다.'},
        'costmaps': {'summary': '토픽별 최대 10 Hz, 동일 지도 중복 제외',
                     'detail': 'Costmap 수신 시 관측합니다. 동일한 지도 레이어는 다시 저장하지 않습니다.'},
        'bt': {'summary': '상태 수신 시 갱신, 전이 이벤트 기록',
               'detail': f'동일 상태 반복 이벤트는 {repeat:g}초마다 요약합니다. 실패, ComputePath, FollowPath, Wait 전이는 즉시 기록 대기열에 넣습니다.'},
        'safety': {'summary': '진단 최대 10 Hz, 판단 변경 시 이벤트',
                   'detail': '안전 진단의 최신값과 정지, 감속 판단의 변경 이벤트를 저장합니다.'},
        'logs': {'summary': '/rosout 수신 이벤트별 기록',
                 'detail': '고정 Hz로 표본화하지 않습니다. 관측한 로그를 이벤트별로 저장합니다.'},
    }


def source_rate_defaults():
    return {identity: spec['default_hz'] for key, spec in SCHEMA.items()
            for identity in ([f'{key}:{index}' for index in range(8)] if spec['multiple'] else [key])}


def defaults():
    profile = rate_profiles()['standard']
    return {'active': True, 'preset': 'standard', 'enabled': {k: k not in RAW for k in LABELS},
            'hz': {**{k: 10. for k in LABELS if k != 'logs'}, **profile['hz']}, 'topics': dict(RAW), 'sources': {}, 'source_enabled': {}, 'source_hz': profile['source_hz'], 'rate_defaults_version': 4, 'limit_mib': 100}


def _historical_source_rate_defaults():
    # Freeze the generated v2/v3 values for migration, independent of new defaults.
    rates = {
        'runtime_debug_topic': 2.0,
        'runtime_events_topic': None,
        'runtime_result_topic': None,
        'manipulation_events_topic': None,
        'odometry_topic': 10.0,
        'ground_truth_topic': 10.0,
        'amcl_pose_topic': 5.0,
        'clock_topic': 5.0,
        'velocity_topics': 10.0,
        '_computed_motion': 5.0,
        'path_topic': 2.0,
        'local_path_topic': 5.0,
        'controller_evaluation_topic': 2.0,
        'map_topic': 0.2,
        'local_costmap_topic': 2.0,
        'global_costmap_topic': 1.0,
        'nav2_bt_topic': None,
        'nav2_bt_trace_topic': 1.0,
        'pan_state_topic': 5.0,
        'reactive_guard_topic': 5.0,
        'voxel_guard_topic': 5.0,
        'collision_state_topic': 5.0,
        'coverage_topic': 2.0,
        'scan_topic': 2.0,
        'depth_points_topic': 2.0,
        'risk_points_topic': 2.0,
        'footprint_topic': 2.0,
        'stop_polygon_topic': 1.0,
        'slow_polygon_topic': 1.0,
        'obstacle_memory_topic': 1.0,
        'lidar_memory_topic': 1.0,
        'depth_memory_topic': 1.0,
        'rosout_topic': None,
    }
    return {identity: rate for key, rate in rates.items()
            for identity in ([f'{key}:{index}' for index in range(8)] if key == 'velocity_topics' else [key])}


def _previous_rate_profiles():
    standard = _historical_source_rate_defaults()
    standard['runtime_debug_topic'] = 5.
    low = {key: None if rate is None else {10.: 2., 5.: 1., 2.: .5, 1.: .2, .2: .1}[rate]
           for key, rate in standard.items()}
    precise = {key: None if rate is None else {10.: 10., 5.: 10., 2.: 5., 1.: 2., .2: 1.}[rate]
               for key, rate in standard.items()}
    # These summaries are generated at 2 Hz, and the combined navigation state at 5 Hz.
    for key in ('scan_topic', 'depth_points_topic', 'risk_points_topic', 'controller_evaluation_topic'):
        precise[key] = 2.
    precise['_computed_motion'] = 5.
    return {
        'low': dict(label='저사양', source_hz=low,
                    hz=dict(resources=.2, lidar=1., point_cloud=.5, voxel_layer=.5, voxel_world=.5, camera=.2)),
        'standard': dict(label='기본', source_hz=standard,
                         hz=dict(resources=1., lidar=5., point_cloud=2., voxel_layer=2., voxel_world=2., camera=1.)),
        'precise': dict(label='정밀', source_hz=precise,
                        hz=dict(resources=2., lidar=10., point_cloud=5., voxel_layer=5., voxel_world=5., camera=5.)),
    }


def _version_three_rate_profiles(resource_interval_s=1.):
    standard = _historical_source_rate_defaults()
    precise = {}
    for key, rate in standard.items():
        role = key.split(':')[0]
        observation_max = 2. if role in ('scan_topic', 'depth_points_topic', 'risk_points_topic', 'controller_evaluation_topic') else 10.
        bound = min(SCHEMA[role]['maximum']['hz'], observation_max)
        standard[key] = None if rate is None else min(rate, bound)
        precise[key] = None if rate is None else bound
    low = {key: None if rate is None else min(rate, {10.: 2., 5.: 1., 2.: .5, 1.: .2, .2: .1}.get(rate, max(.1, rate / 5)))
           for key, rate in standard.items()}
    resource_max = 1 / resource_interval_s
    raw_max = {key: reference['hz'] for key, reference in raw_maximum_references().items()}
    profiles = {
        'low': dict(label='저사양', source_hz=low,
                    hz=dict(resources=min(.2, resource_max), lidar=1., point_cloud=.5, voxel_layer=.5, voxel_world=.2, camera=.2)),
        'standard': dict(label='기본', source_hz=standard,
                         hz=dict(resources=min(1., resource_max), lidar=5., point_cloud=2., voxel_layer=2., voxel_world=1., camera=1.)),
        'precise': dict(label='정밀', source_hz=precise,
                        hz=dict(resources=resource_max, **raw_max)),
    }
    for profile in profiles.values():
        for key, bound in raw_max.items():
            profile['hz'][key] = min(profile['hz'][key], bound)
    return profiles


# Rates express useful diagnostic resolution, independently of publisher maxima.
# At 0.2 m/s, 2 Hz poses resolve roughly 10 cm; 10 Hz resolves 2 cm.
# Slow geometry and maps do not gain useful detail from control-loop frequencies.
SEMANTIC_RATES = {
    **{f'slam_{key}_topic': (1., 2., 2.) for key in ('submaps', 'trajectory', 'constraints', 'matched_points')},
    'runtime_debug_topic': (.5, 1., 2.),
    'odometry_topic': (2., 5., 10.), 'ground_truth_topic': (2., 5., 10.),
    'amcl_pose_topic': (1., 2., 5.), 'clock_topic': (1., 2., 5.),
    'velocity_topics': (2., 5., 10.), '_computed_motion': (2., 5., 10.),
    'path_topic': (.5, 1., 2.), 'local_path_topic': (1., 2., 5.),
    'controller_evaluation_topic': (.5, 1., 2.), 'map_topic': (.1, .2, .5),
    'local_costmap_topic': (.5, 1., 2.), 'global_costmap_topic': (.2, .5, 1.),
    'nav2_bt_trace_topic': (1., 2., 4.),
    'pan_state_topic': (1., 2., 5.), 'reactive_guard_topic': (1., 2., 5.),
    'voxel_guard_topic': (1., 2., 5.), 'collision_state_topic': (1., 2., 5.),
    'coverage_topic': (.5, 1., 2.), 'scan_topic': (.5, 1., 2.),
    'depth_points_topic': (.5, 1., 2.), 'risk_points_topic': (.5, 1., 2.),
    'footprint_topic': (.5, 1., 2.), 'stop_polygon_topic': (.2, .5, 1.),
    'slow_polygon_topic': (.2, .5, 1.), 'obstacle_memory_topic': (.2, .5, 1.),
    'lidar_memory_topic': (.2, .5, 1.), 'depth_memory_topic': (.2, .5, 1.),
}


def rate_profiles(resource_interval_s=1.):
    raw_max = raw_maximum_references()
    profiles = {}
    for index, (name, label) in enumerate((('low', '저사양'), ('standard', '기본'), ('precise', '정밀'))):
        sources = {}
        for identity, default in source_rate_defaults().items():
            role = identity.split(':')[0]
            if default is None:
                sources[identity] = None
                continue
            observation_max = 2. if role in ('scan_topic', 'depth_points_topic', 'risk_points_topic', 'controller_evaluation_topic') else 10.
            sources[identity] = min(SEMANTIC_RATES[role][index], SCHEMA[role]['maximum']['hz'], observation_max)
        raw = {key: min(rates[index], raw_max[key]['hz']) for key, rates in {
            'lidar': (1., 3., 5.5), 'point_cloud': (.5, 2., 5.),
            'voxel_layer': (.5, 2., 5.), 'voxel_world': (.2, .5, 1.),
            'camera': (.2, 1., 5.),
        }.items()}
        profiles[name] = dict(label=label, source_hz=sources,
                              hz=dict(resources=min((.2, .5, 1.)[index], 1 / resource_interval_s), **raw))
    return profiles


def validate(value):
    if isinstance(value, dict):
        value = dict(value)
        if value.get('preset') in ('basic', 'obstacles', 'custom'):
            value['preset'] = 'standard'  # Preserve old selection and rates; replace only the legacy label.
        value.setdefault('sources', {})
        value.setdefault('source_enabled', {})
        # Expand legacy group rates once; thereafter each source owns its rate.
        if 'source_hz' not in value:
            legacy_rates = value.get('hz') if isinstance(value.get('hz'), dict) else {}
            value['source_hz'] = {identity: (legacy_rates[SCHEMA[identity.split(':')[0]]['group']]
                                           if legacy_rates.get(SCHEMA[identity.split(':')[0]]['group'], 10.) != 10. else rate)
                                  for identity, rate in source_rate_defaults().items()}
        if 'rate_defaults_version' not in value:
            # Replace the prior uniform defaults; keep differing user rates.
            if isinstance(value['source_hz'], dict):
                value['source_hz'] = {key: source_rate_defaults().get(key, rate) if rate == 10. else rate
                                      for key, rate in value['source_hz'].items()}
            if isinstance(value.get('hz'), dict):
                value['hz'] = {key: defaults()['hz'].get(key, rate) if key in RAW and rate == 2. else rate
                               for key, rate in value['hz'].items()}
            value['rate_defaults_version'] = 2
    if not isinstance(value, dict) or set(value) != set(defaults()):
        raise ValueError('녹화 설정 형식이 올바르지 않습니다.')
    if type(value['rate_defaults_version']) is int and value['rate_defaults_version'] == 2 and value.get('preset') in rate_profiles():
        # Upgrade generated profile values while keeping individually edited rates.
        previous = _previous_rate_profiles()[value['preset']]
        updated = _version_three_rate_profiles()[value['preset']]
        for field in ('source_hz', 'hz'):
            if isinstance(value.get(field), dict):
                value[field] = {key: updated[field][key] if key in previous[field] and rate == previous[field][key] else rate
                                for key, rate in value[field].items()}
        value['rate_defaults_version'] = 3
    if type(value['rate_defaults_version']) is int and value['rate_defaults_version'] == 3 and value.get('preset') in rate_profiles():
        previous = _version_three_rate_profiles()[value['preset']]
        updated = rate_profiles()[value['preset']]
        for field in ('source_hz', 'hz'):
            if isinstance(value.get(field), dict):
                value[field] = {key: updated[field][key] if key in previous[field] and rate == previous[field][key] else rate
                                for key, rate in value[field].items()}
        value['rate_defaults_version'] = 4
    if type(value['rate_defaults_version']) is not int or value['rate_defaults_version'] != 4:
        raise ValueError('지원하지 않는 기록 주기 기본값 버전입니다.')
    if type(value['active']) is not bool or value['preset'] not in rate_profiles():
        raise ValueError('녹화 상태 또는 프리셋이 올바르지 않습니다.')
    if any(not isinstance(value[key], dict) for key in ('enabled', 'hz', 'topics')):
        raise ValueError('녹화 항목, 주기, 토픽 형식이 올바르지 않습니다.')
    value = deepcopy(value)
    value['sources'] = validate_sources(value['sources'])
    value['source_enabled'] = validate_source_enabled(value['source_enabled'])
    if not isinstance(value['source_hz'], dict) or not set(value['source_hz']) <= set(source_rate_defaults()):
        raise ValueError('개별 토픽 기록 주기 형식이 올바르지 않습니다.')
    value['source_hz'] = {**source_rate_defaults(), **value['source_hz']}
    for hz in value['source_hz'].values():
        if hz is not None and (type(hz) not in (float, int) or not math.isfinite(hz) or not .1 <= hz <= 1000):
            raise ValueError('토픽 기록 주기는 0.1~1000 Hz 범위입니다.')
    value['enabled'].setdefault('resources', True)
    value['enabled'].setdefault('slam', True)
    if set(value['hz']) != set(RAW):
        value['hz'].setdefault('slam', 2.)
    if set(value['enabled']) != set(LABELS) or any(type(v) is not bool for v in value['enabled'].values()):
        raise ValueError('녹화 항목을 모두 지정해야 합니다.')
    # Migrate the prior raw-only schema without overwriting its configured rates.
    if set(value['hz']) == set(RAW):
        value['hz'] = {**defaults()['hz'], **value['hz']}
    if set(value['hz']) != set(defaults()['hz']) or set(value['topics']) != set(RAW):
        raise ValueError('녹화 주기 또는 원본 토픽 설정이 누락되었습니다.')
    for hz in value['hz'].values():
        if type(hz) not in (float, int) or not math.isfinite(hz) or not .1 <= hz <= 1000:
            raise ValueError('기록 주기는 0.1~1000 Hz 범위입니다.')
    for key in RAW:
        if not isinstance(value['topics'][key], str) or not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_/]*', value['topics'][key]):
            raise ValueError('원본 토픽은 절대 ROS 토픽 이름이어야 합니다.')
    if len(set(value['topics'].values())) != len(RAW):
        raise ValueError('원본 녹화 토픽은 서로 달라야 합니다.')
    if type(value['limit_mib']) is not int or not 10 <= value['limit_mib'] <= 1024:
        raise ValueError('세션 용량은 10~1024 MiB 범위입니다.')
    return deepcopy(value)


def category(row):
    if 'category' in row:
        return row['category']
    kind, key = row['kind'], row['key']
    if kind in ('config', 'graph', 'recording_policy', 'recording_status') or (kind == 'layer' and key == 'map'):
        return None  # Context is always retained.
    if kind in ('slam', 'slam_texture') or (kind == 'event' and str(row['data'].get('kind', '')).startswith('slam_')):
        return 'slam'
    if kind == 'resources':
        return 'resources'
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
    identity = row.get('source_id')
    if identity:
        if not settings.get('source_enabled', {}).get(identity, True):
            return False
        group = SCHEMA[identity.split(':')[0]]['group']
        if not settings['enabled'][group]:
            return False
    key = category(row) if 'category' in row else next((k for k,v in settings['topics'].items() if row['key'] == v), None) or category(row)
    return row['kind'] in ('recording_policy','recording_status') or settings['active'] and (key is None or settings['enabled'][key])


class RecordingControls:
    def __init__(self, queue_limit=32*1024**2):
        self.lock = threading.Lock()
        self.settings = defaults()
        self.source_config = {}
        self.revision = 0
        self.queue = deque()
        self.queued_bytes = 0
        self.queue_limit = queue_limit
        self.last = {}
        self.stats = {}
        self.stats_generation = 0
        self.available = {}
        self.writer_error = ""
        self.transitioning = False

    @contextmanager
    def transition(self):
        """Seal in-flight callback tickets before draining or rotating a writer."""
        with self.lock:
            self.transitioning = True
            self.revision += 1
        try:
            yield
        finally:
            with self.lock:
                self.transitioning = False

    def configure(self, settings):
        settings = validate(settings)
        with self.lock:
            self.settings = settings
            self.revision += 1
            self.last.clear()
            self.stats = {}
            self.stats_generation += 1

    def camera_info_topic(self):
        topic = self.settings['topics']['camera']
        if topic.endswith('/compressed'):
            topic = topic.rsplit('/', 1)[0]
        return topic.rsplit('/', 1)[0] + '/camera_info'

    def context_topics(self):
        """Called under lock; topic remapping and per-source opt-outs apply."""
        s = self.settings
        sources = {**resolved(self.source_config), **s['sources']}
        topics = {sources[field]: (group, typ, field)
                  for field, (group, typ) in CONTEXT.items()
                  if s['enabled'][group] and s['source_enabled'].get(field, True)}
        if s['enabled']['motion']:
            topics.update({'/joint_states': ('motion', 'sensor_msgs/msg/JointState', None),
                           '/robot_description': ('motion', 'std_msgs/msg/String', None)})
        return topics

    def has_ros_context(self):
        return any(self.settings['enabled'][k] for k in RAW) or bool(self.context_topics())

    def topics(self):
        with self.lock:
            if self.writer_error or not self.settings['active']:
                return set()
            topics = {v for k,v in self.settings['topics'].items() if self.settings['enabled'][k]}
            topics.update(self.context_topics())
            if self.settings['enabled']['voxel_world']:
                topics.update(('/obstacle_memory/grid', '/obstacle_memory/lidar', '/obstacle_memory/depth'))
            if self.has_ros_context():
                topics.update(('/tf', '/tf_static', '/clock'))
            if self.settings['enabled']['camera']:
                topics.add(self.camera_info_topic())
            return topics

    def discover(self, topics):
        with self.lock:
            self.available = dict(topics)

    def reserve(self, topic):
        with self.lock:
            s = self.settings
            key = next((k for k,v in s['topics'].items() if v == topic), None)
            context = self.context_topics().get(topic)
            if context and key is None:
                key = context[0]
            if topic in ('/tf','/tf_static') and self.has_ros_context():
                key = 'transforms'
            if topic == '/clock' and self.has_ros_context():
                key = 'clock'
            if topic == self.camera_info_topic() and s['enabled']['camera']:
                key = 'camera_info'
            if self.transitioning or self.writer_error or not key or not s['active'] or key in RAW and not s['enabled'][key]:
                return None
            now = time.monotonic()
            self.stats.setdefault(key, {})['received_at'] = time.time()
            hz = s['hz'].get(key) if key in RAW else None
            if context and context[2]:
                hz = s['source_hz'].get(context[2])
            if hz is not None and now-self.last.get(topic, -float('inf')) < 1/hz:
                return None
            self.last[topic] = now
            return key, self.revision

    def offer(self, ticket, topic, message_type, payload, ros_time, *, timestamp_ns=None, reliable=False, latched=False):
        key, revision = ticket
        with self.lock:
            if revision != self.revision:
                return
            stat = self.stats.setdefault(key, {})
            context = self.context_topics().get(topic)
            supported = {context[1]} if context and key not in RAW else TYPES.get(key, set())
            if message_type not in supported:
                stat['error'] = '지원하지 않는 ROS 메시지 타입'
                return
            received_ns = time.time_ns()
            message = RawMessage(key, topic, message_type, received_ns / 1e9, ros_time, payload,
                                 received_ns if timestamp_ns is None else timestamp_ns,
                                 reliable, latched or topic == '/tf_static')
            self._enqueue(message)

    def _enqueue(self, message):
        stat = self.stats.setdefault(message.category, {})
        if len(self.queue) >= 128 or len(message.payload) > 8*1024**2 or self.queued_bytes+len(message.payload) > self.queue_limit:
            stat['error'] = '원본 크기 또는 저장 대기 한도 초과'
            stat['dropped'] = stat.get('dropped', 0)+1
            return
        self.queue.append(message)
        self.queued_bytes += len(message.payload)

    def dropped(self, key, count=1):
        with self.lock:
            stat = self.stats.setdefault(key, {})
            stat['dropped'] = stat.get('dropped', 0) + count

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
            for key in (*LABELS, 'transforms', 'clock', 'camera_info'):
                stat = dict(self.stats.get(key, {}))
                is_enabled = (self.has_ros_context() if key in ('transforms', 'clock') else
                              s['enabled']['camera'] if key == 'camera_info' else s['enabled'][key])
                state = '미녹화' if not is_enabled else '녹화 정지' if not s['active'] else '저장 중단' if reason else '저장 오류' if stat.get('error') else '기록 대기'
                if state == '기록 대기' and key in ('transforms', 'clock', 'camera_info'):
                    topics = ('/tf', '/tf_static') if key == 'transforms' else ('/clock',) if key == 'clock' else (self.camera_info_topic(),)
                    state = '토픽 미수신' if not any(self.available.get(t) for t in topics) else '기록 중' if stat.get('count') else '수신 대기'
                elif state == '기록 대기' and key in RAW:
                    types = set(self.available.get(s['topics'][key], []))
                    state = '토픽 미수신' if not types else '타입 불일치' if not types <= TYPES[key] else '입력 중단' if stat.get('received_at') and time.time()-stat['received_at'] > max(3,2/s['hz'][key]) else '기록 중' if stat.get('count') else '수신 대기'
                elif state == '기록 대기':
                    state = '기록 중' if stat.get('count') else '수신 대기'
                statuses[key] = dict(stat, state=state)
            return {'settings': deepcopy(s), 'labels': LABELS, 'status': statuses, 'reason': reason,
                    'raw_format': 'rosbag2/sqlite3; CDR messages sampled in time',
                    'storage': {'raw': 'rosbag2', 'processed': 'sqlite3'},
                    'revision': self.revision, 'stats_generation': self.stats_generation}
