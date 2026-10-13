"""Editable observation routes; independent of ROS and recording rates."""
from copy import deepcopy
import re

FIELDS = {
    'motion': {'runtime_debug_topic': ('미션 상태', '/mission/debug_snapshot'), 'runtime_events_topic': ('미션 이벤트', '/mission/runtime_events'), 'runtime_result_topic': ('미션 결과', '/mission/result'), 'manipulation_events_topic': ('조작 이벤트', '/mock/manipulation/execution_events'), 'odometry_topic': ('휠 오도메트리', '/wheel/odom'), 'ground_truth_topic': ('Ground truth', '/ground_truth/odom'), 'amcl_pose_topic': ('AMCL 위치', '/amcl_pose'), 'clock_topic': ('시뮬레이션 시계', '/clock'), 'velocity_topics': ('속도 체인 (한 줄에 하나, 순서 유지)', ['/nav2/cmd_vel', '/safety_2d/cmd_vel', '/cmd_vel'])},
    'paths': {'path_topic': ('전체 경로', '/plan'), 'local_path_topic': ('지역 경로', '/local_plan'), 'controller_evaluation_topic': ('제어 후보 평가', '/evaluation')},
    'costmaps': {'map_topic': ('정적 지도', '/map'), 'local_costmap_topic': ('Local costmap', '/local_costmap/costmap'), 'global_costmap_topic': ('Global costmap', '/global_costmap/costmap')},
    'bt': {'nav2_bt_topic': ('BT 상태 이벤트', '/behavior_tree_log'), 'nav2_bt_trace_topic': ('BT 트리 구조·상태', '/navigation/bt_trace')},
    'safety': {'pan_state_topic': ('Pan 상태', '/head_pan/state'), 'reactive_guard_topic': ('Reactive guard', '/reactive_guard/state'), 'voxel_guard_topic': ('Voxel guard', '/body_guard_3d/state'), 'collision_state_topic': ('Collision monitor', '/collision_monitor_state'), 'coverage_topic': ('센서 Coverage', '/head_pan/coverage'), 'scan_topic': ('LiDAR 관측 요약', '/scan'), 'depth_points_topic': ('Depth 관측 요약', '/camera/head/depth/obstacle_points'), 'risk_points_topic': ('위험 점 관측 요약', '/body_guard_3d/risk_voxels'), 'footprint_topic': ('Footprint', '/local_costmap/published_footprint'), 'stop_polygon_topic': ('정지 영역', '/safety/stop_zone'), 'slow_polygon_topic': ('감속 영역', '/safety/slow_zone'), 'obstacle_memory_topic': ('장애물 메모리', '/obstacle_memory/grid'), 'lidar_memory_topic': ('LiDAR 메모리', '/obstacle_memory/lidar'), 'depth_memory_topic': ('Depth 메모리', '/obstacle_memory/depth')},
    'slam': {'slam_submaps_topic': ('SLAM 서브맵', '/submap_list'), 'slam_trajectory_topic': ('SLAM 궤적', '/trajectory_node_list'), 'slam_constraints_topic': ('SLAM 제약선', '/constraint_list'), 'slam_matched_points_topic': ('SLAM 매칭 점군', '/scan_matched_points2')},
    'logs': {'rosout_topic': ('ROS 로그', '/rosout')},
}
SCHEMA = {key: dict(group=group, label=label, multiple=isinstance(default, list)) for group, fields in FIELDS.items() for key,(label,default) in fields.items()}


def resolved(config):
    prefix = '/' + config.get('runtime_namespace', '').strip('/') if config.get('runtime_namespace', '').strip('/') else ''
    values = {key: deepcopy(config.get(key, default)) for fields in FIELDS.values() for key,(_,default) in fields.items()}
    for key in ('runtime_debug_topic', 'runtime_events_topic', 'runtime_result_topic'):
        if not config.get(key):
            values[key] = prefix + next(default for fields in FIELDS.values() for k,(_,default) in fields.items() if k == key)
    if not values['manipulation_events_topic'].startswith('/'):
        values['manipulation_events_topic'] = prefix + '/' + values['manipulation_events_topic']
    return values


def validate_sources(value):
    if not isinstance(value, dict) or not set(value) <= (set(SCHEMA) - {'_computed_motion'}):
        raise ValueError('지원하지 않는 기본 관측 토픽 설정입니다.')
    seen = set()
    for key, item in value.items():
        if SCHEMA[key]['multiple']:
            if not isinstance(item, list) or not 1 <= len(item) <= 8:
                raise ValueError('속도 체인은 1~8개 토픽이 필요합니다.')
            topics = item
        else:
            topics = [item]
        for topic in topics:
            if not isinstance(topic, str) or len(topic) > 256 or not re.fullmatch(r'(?:/[A-Za-z_][A-Za-z0-9_]*)+', topic):
                raise ValueError('기본 관측 토픽은 유효한 절대 ROS 토픽 이름이어야 합니다.')
            if topic in seen:
                raise ValueError('서로 다른 기본 관측 역할에 같은 토픽을 지정할 수 없습니다.')
            seen.add(topic)
    return deepcopy(value)


def routes(config):
    c = {**config, **resolved(config)}
    navigation = {c[key+'_topic']: key for key in ('odometry', 'ground_truth', 'amcl_pose', 'reactive_guard', 'voxel_guard', 'coverage', 'nav2_bt', 'controller_evaluation', 'scan', 'depth_points', 'risk_points', 'footprint', 'stop_polygon', 'slow_polygon')}
    navigation.update({c['pan_state_topic']: 'pan', c['collision_state_topic']: 'collision'})
    layers = {c[key+'_topic']: key for key in ('map', 'path', 'local_costmap', 'global_costmap')}
    fixed = set(layers) | {c[key] for key in ('runtime_debug_topic', 'runtime_events_topic', 'runtime_result_topic', 'rosout_topic', 'clock_topic', 'manipulation_events_topic', 'nav2_bt_trace_topic')} | set(c['velocity_topics'])
    fixed.update(c['slam_'+key+'_topic'] for key in ('submaps', 'trajectory', 'constraints', 'matched_points'))
    fixed.update(c[key+'_topic'] for key in ('odometry', 'ground_truth', 'amcl_pose', 'pan_state', 'reactive_guard', 'voxel_guard', 'collision_state', 'coverage', 'nav2_bt'))
    layers.update({c[key+'_topic']: key for key in ('obstacle_memory', 'lidar_memory', 'depth_memory', 'local_path')})
    return c, fixed, layers, navigation

SCHEMA['_computed_motion'] = dict(group='motion', label='주행 요약·TF 위치', multiple=False, internal=True)


def source_roles(config):
    return {topic: (f'{key}:{index}' if isinstance(value,list) else key) for key,value in resolved(config).items() for index,topic in enumerate(value if isinstance(value,list) else [value])}


def validate_source_enabled(value):
    if not isinstance(value, dict):
        raise ValueError('개별 관측 항목의 기록 선택 형식이 올바르지 않습니다.')
    for key, enabled in value.items():
        if type(enabled) is not bool or (key in SCHEMA and SCHEMA[key]['multiple']) or (key not in SCHEMA and not re.fullmatch(r'velocity_topics:[0-7]', key)):
            raise ValueError('지원하지 않는 개별 관측 항목의 기록 선택입니다.')
    return deepcopy(value)


# Recording bounds follow the monitor's observation cadence and payload cost.
# None preserves received event records without an additional time-based filter.
SOURCE_DEFAULT_HZ = {
    'slam_submaps_topic': 2., 'slam_trajectory_topic': 2., 'slam_constraints_topic': 2., 'slam_matched_points_topic': 2.,
    'runtime_debug_topic': 1.0,
    'runtime_events_topic': None,
    'runtime_result_topic': None,
    'manipulation_events_topic': None,
    'odometry_topic': 5.0,
    'ground_truth_topic': 5.0,
    'amcl_pose_topic': 2.0,
    'clock_topic': 2.0,
    'velocity_topics': 5.0,
    '_computed_motion': 5.0,
    'path_topic': 1.0,
    'local_path_topic': 2.0,
    'controller_evaluation_topic': 1.0,
    'map_topic': 0.2,
    'local_costmap_topic': 1.0,
    'global_costmap_topic': 0.5,
    'nav2_bt_topic': None,
    'nav2_bt_trace_topic': 2.0,
    'pan_state_topic': 2.0,
    'reactive_guard_topic': 2.0,
    'voxel_guard_topic': 2.0,
    'collision_state_topic': 2.0,
    'coverage_topic': 1.0,
    'scan_topic': 1.0,
    'depth_points_topic': 1.0,
    'risk_points_topic': 1.0,
    'footprint_topic': 1.0,
    'stop_polygon_topic': 0.5,
    'slow_polygon_topic': 0.5,
    'obstacle_memory_topic': 0.5,
    'lidar_memory_topic': 0.5,
    'depth_memory_topic': 0.5,
    'rosout_topic': None,
}
for key, spec in SCHEMA.items():
    spec['default_hz'] = SOURCE_DEFAULT_HZ[key]



# Publisher reference bounds, separate from recording and UI processing limits.
# Event/input-driven topics have planning estimates, not physical hard limits.
SOURCE_MAX_HZ = {
    **{f'slam_{key}_topic': (5., 'estimate', 'Cartographer 발행 설정 의존; 관측 처리 최대 2 Hz') for key in ('submaps', 'trajectory', 'constraints', 'matched_points')},
    'runtime_debug_topic': (2., 'code', 'mission_manager/node.py: debug snapshot 최소 간격 0.5초'),
    'runtime_events_topic': (100., 'estimate', '이벤트 burst 기준 추정; 고정 발행 상한 없음'),
    'runtime_result_topic': (10., 'estimate', '미션 결과 burst 기준 추정; 완료 시 발행'),
    'manipulation_events_topic': (20., 'estimate', '조작 이벤트 burst 기준 추정'),
    'odometry_topic': (50., 'estimate', 'JointState 입력 의존; 베이스 50 Hz 기준 추정'),
    'ground_truth_topic': (50., 'config', 'Gazebo SDF: odom_publish_frequency=50'),
    'amcl_pose_topic': (10., 'estimate', '이동·스캔 갱신에 의존; 위치 추정 10 Hz 가정'),
    'clock_topic': (1000., 'estimate', '시뮬레이션 clock 전달 주기 추정; 실행 설정에 따라 다름'),
    'velocity_topics': (20., 'config', 'Nav2 controller_frequency=20; guard update_period_s=0.05'),
    '_computed_motion': (10., 'code', 'dev_monitor: TF 위치 0.1초 / 주행 요약 0.2초 타이머'),
    'path_topic': (5., 'estimate', '요청·BT 재계획 의존; expected_planner_frequency=5는 강제 상한이 아님'),
    'local_path_topic': (20., 'config', 'Nav2 controller_frequency=20; 제어 경로 갱신 기준'),
    'controller_evaluation_topic': (20., 'config', 'Nav2 controller_frequency=20; 후보 평가 갱신 기준'),
    'map_topic': (1., 'estimate', '정적 지도·SLAM 지도 변경 전달 기준 추정'),
    'local_costmap_topic': (2., 'config', 'nav2_amcl.yaml: local_costmap.publish_frequency=2'),
    'global_costmap_topic': (1., 'config', 'nav2_amcl.yaml: global_costmap.publish_frequency=1'),
    'nav2_bt_topic': (100., 'estimate', 'BT 전이 burst 기준 추정; 고정 발행 상한 없음'),
    'nav2_bt_trace_topic': (4., 'code', 'bt_trace_node.py: 0.25초 발행 타이머'),
    'pan_state_topic': (20., 'code', 'pan_motion_gate_node.py: 기본 update_period_s=0.05'),
    'reactive_guard_topic': (20., 'code', 'reactive_guard_node.py: 기본 update_period_s=0.05'),
    'voxel_guard_topic': (20., 'config', 'voxel_guard.yaml: update_period_s=0.05'),
    'collision_state_topic': (20., 'estimate', '속도 입력·판단 변경 의존; 제어 루프 20 Hz 기준 추정'),
    'coverage_topic': (5., 'code', 'depth_coverage_node.py: 기본 update_period_s=0.2'),
    'scan_topic': (5.5, 'config', 'Gazebo SDF: LiDAR update_rate=5.5; 요약 처리 상한과 별개'),
    'depth_points_topic': (15., 'config', 'Gazebo SDF: Depth update_rate=15; 점군 처리로 실제 주기는 낮을 수 있음'),
    'risk_points_topic': (20., 'config', 'Voxel guard: update_period_s=0.05; update에서 위험 점 발행'),
    'footprint_topic': (10., 'config', 'nav2_amcl.yaml: local_costmap.update_frequency=10'),
    'stop_polygon_topic': (20., 'estimate', '안전 판단 주기 20 Hz 기준 추정'),
    'slow_polygon_topic': (20., 'estimate', '안전 판단 주기 20 Hz 기준 추정'),
    'obstacle_memory_topic': (1., 'config', 'obstacle_memory.yaml: world_publish_period=1'),
    'lidar_memory_topic': (1., 'config', 'obstacle_memory.yaml: world_publish_period=1'),
    'depth_memory_topic': (1., 'config', 'obstacle_memory.yaml: world_publish_period=1'),
    'rosout_topic': (100., 'estimate', '여러 노드 로그 burst 기준 추정; 고정 발행 상한 없음'),
}


def maximum_reference(value, kind, basis):
    return dict(hz=value, estimated=kind == 'estimate', basis=kind, detail=basis)


for key, spec in SCHEMA.items():
    spec['maximum'] = maximum_reference(*SOURCE_MAX_HZ[key])


def raw_maximum_references():
    return {
        'lidar': maximum_reference(5.5, 'config', 'Gazebo SDF: LiDAR update_rate=5.5'),
        'point_cloud': maximum_reference(15., 'config', 'Gazebo SDF: Depth update_rate=15'),
        'voxel_layer': maximum_reference(10., 'config', 'Nav2 local_costmap.update_frequency=10; voxel layer 갱신 기준'),
        'voxel_world': maximum_reference(1., 'config', 'obstacle_memory.yaml: world_publish_period=1'),
        'camera': maximum_reference(15., 'config', 'Gazebo SDF: RGB camera update_rate=15'),
    }
