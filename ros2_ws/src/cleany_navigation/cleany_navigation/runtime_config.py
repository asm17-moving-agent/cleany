"""Pure parameter composition and Nav2 distribution compatibility."""
from __future__ import annotations

import ast
from copy import deepcopy
import math
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml

# Humble replaces the built-in BT list when plugin_lib_names is supplied.
HUMBLE_BT_LIBRARIES = [
    'nav2_compute_path_to_pose_action_bt_node', 'nav2_compute_path_through_poses_action_bt_node',
    'nav2_follow_path_action_bt_node', 'nav2_spin_action_bt_node', 'nav2_wait_action_bt_node',
    'nav2_back_up_action_bt_node', 'nav2_drive_on_heading_bt_node',
    'nav2_clear_costmap_service_bt_node', 'nav2_is_stuck_condition_bt_node',
    'nav2_goal_reached_condition_bt_node', 'nav2_goal_updated_condition_bt_node',
    'nav2_globally_updated_goal_condition_bt_node', 'nav2_is_path_valid_condition_bt_node',
    'nav2_initial_pose_received_condition_bt_node', 'nav2_reinitialize_global_localization_service_bt_node',
    'nav2_rate_controller_bt_node', 'nav2_distance_controller_bt_node', 'nav2_speed_controller_bt_node',
    'nav2_truncate_path_action_bt_node', 'nav2_truncate_path_local_action_bt_node',
    'nav2_recovery_node_bt_node', 'nav2_pipeline_sequence_bt_node', 'nav2_round_robin_node_bt_node',
    'nav2_navigate_through_poses_action_bt_node', 'nav2_navigate_to_pose_action_bt_node',
    'nav2_remove_passed_goals_action_bt_node', 'nav2_controller_selector_bt_node',
    'nav2_planner_selector_bt_node', 'nav2_goal_checker_selector_bt_node',
]


def load_parameters(path: Path) -> dict:
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict):
        raise ValueError(f'Expected parameter mapping: {path}')
    return document


def merge_parameters(target: dict, overlay: dict) -> None:
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            merge_parameters(target[key], value)
        else:
            target[key] = deepcopy(value)


def set_sim_time(document: dict, use_sim_time: bool) -> None:
    for key, value in document.items():
        if key == 'ros__parameters':
            value['use_sim_time'] = use_sim_time
        elif isinstance(value, dict):
            set_sim_time(value, use_sim_time)


def polygon_points(value) -> list[list[float]]:
    points = ast.literal_eval(value) if isinstance(value, str) else value
    if not isinstance(points, list) or len(points) < 3:
        raise ValueError('A polygon needs at least three vertices')
    if not all(isinstance(p, (list, tuple)) and len(p) == 2
               and all(isinstance(v, (int, float)) and math.isfinite(v) for v in p) for p in points):
        raise ValueError('Expected finite XY polygon vertices')
    return [[float(v) for v in p] for p in points]


def adapt_nav2_parameters(document: dict, distro: str) -> dict:
    """Preserve stop trigger counts and plugin semantics across Humble/Jazzy."""
    if distro not in ('humble', 'jazzy'):
        raise ValueError(f'Unsupported ROS distribution: {distro}')
    result = deepcopy(document)
    # ROS parameter YAML cannot represent nested arrays. Nav2 footprints and
    # Jazzy monitor polygons use serialized XY lists, even if the robot file
    # used a natural YAML list of vertices.
    for name in ('local_costmap', 'global_costmap'):
        params = result.get(name, {}).get(name, {}).get('ros__parameters', {})
        if isinstance(params.get('footprint'), list):
            params['footprint'] = str(polygon_points(params['footprint']))
    monitor = result.get('collision_monitor', {}).get('ros__parameters', {})
    for name in monitor.get('polygons', []):
        if isinstance(monitor[name].get('points'), list):
            monitor[name]['points'] = str(polygon_points(monitor[name]['points']))
    if distro == 'jazzy':
        return result
    def convert(value):
        if isinstance(value, dict):
            return {key: convert(item) for key, item in value.items()}
        if isinstance(value, list):
            return [convert(item) for item in value]
        if isinstance(value, str) and value.startswith(('nav2_navfn_planner::', 'nav2_smac_planner::', 'nav2_behaviors::')):
            return value.replace('::', '/', 1)
        return value
    result = convert(result)
    controller = result.get('controller_server', {}).get('ros__parameters', {})
    if 'progress_checker_plugins' in controller:
        names = controller.pop('progress_checker_plugins')
        if len(names) != 1:
            raise ValueError('Humble supports one progress checker')
        controller['progress_checker_plugin'] = names[0]
    behavior = result.get('behavior_server', {}).get('ros__parameters', {})
    for old, new in [('local_costmap_topic', 'costmap_topic'), ('local_footprint_topic', 'footprint_topic'),
                     ('local_frame', 'global_frame')]:
        if old in behavior:
            behavior[new] = behavior.pop(old)
    for key in ('global_costmap_topic', 'global_footprint_topic'):
        behavior.pop(key, None)
    bt = result.get('bt_navigator', {}).get('ros__parameters', {})
    if 'plugin_lib_names' in bt:
        bt['plugin_lib_names'] = list(dict.fromkeys(HUMBLE_BT_LIBRARIES + bt['plugin_lib_names']))
    for key in ('navigators', 'navigate_to_pose', 'action_server_result_timeout', 'error_code_names'):
        bt.pop(key, None)
    monitor = result.get('collision_monitor', {}).get('ros__parameters', {})
    monitor.pop('enable_stamped_cmd_vel', None)
    for name in monitor.get('polygons', []):
        polygon = monitor[name]
        polygon['points'] = [v for point in polygon_points(polygon['points']) for v in point]
        # Jazzy: points >= min_points; Humble STOP/SLOWDOWN: points > max_points.
        count = polygon.pop('min_points', 3)
        if not isinstance(count, int) or count < 1:
            raise ValueError('min_points must be a positive integer')
        polygon['max_points'] = count - 1
    return result


def adapt_behavior_tree(xml: str, distro: str) -> str:
    """Humble's BT v3 action nodes do not expose Jazzy error-code ports."""
    if distro not in ('humble', 'jazzy'):
        raise ValueError(f'Unsupported ROS distribution: {distro}')
    root = ET.fromstring(xml)
    if distro == 'humble':
        root.attrib.pop('BTCPP_format', None)
        for node in root.iter():
            if node.tag in ('ComputePathToPose', 'FollowPath'):
                node.attrib.pop('error_code_id', None)
    return ET.tostring(root, encoding='unicode')


def validate_robot_parameters(robot: dict, directory: Path, *, use_sim_time: bool) -> None:
    """Require explicit robot geometry/braking; never pick a simulation profile."""
    required = {
        'voxel_guard': ['profile', 'max_speed', 'max_yaw_rate', 'reaction_s',
                        'linear_deceleration', 'angular_deceleration'],
        'obstacle_memory': ['floor_z'],
        'depth_clearance': ['camera_frame', 'optical_frame'],
        'collision_monitor': ['StopZone', 'SlowZone'],
    }
    for node, keys in required.items():
        parameters = robot.get(node, {}).get('ros__parameters', {})
        for key in keys:
            value = parameters.get(key)
            if value is None or value == '':
                raise ValueError(f'robot_params_file must define {node}.{key}')
            if key in ('max_speed', 'max_yaw_rate', 'reaction_s', 'linear_deceleration', 'angular_deceleration'):
                if not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                    raise ValueError(f'Invalid {node}.{key}')
    floor = robot['obstacle_memory']['ros__parameters']['floor_z']
    if not isinstance(floor, (int, float)) or not math.isfinite(floor):
        raise ValueError('Invalid obstacle_memory.floor_z')
    for name in ('local_costmap', 'global_costmap'):
        params = robot.get(name, {}).get(name, {}).get('ros__parameters', {})
        polygon_points(params.get('footprint'))
        obstacle = params.get('obstacle_layer', {})
        for scope in (obstacle, obstacle.get('scan', {})):
            if not isinstance(scope.get('min_obstacle_height'), (int, float)) or not math.isfinite(scope['min_obstacle_height']):
                raise ValueError(f'robot_params_file must define {name} lidar minimum obstacle height')
    for name in ('StopZone', 'SlowZone'):
        polygon_points(robot['collision_monitor']['ros__parameters'][name].get('points'))
    depth = robot.get('local_costmap', {}).get('local_costmap', {}).get('ros__parameters', {}).get('depth_layer', {})
    for key in ('origin_z', 'min_obstacle_height', 'max_obstacle_height'):
        if not isinstance(depth.get(key), (int, float)) or not math.isfinite(depth[key]):
            raise ValueError(f'robot_params_file must define depth_layer.{key}')
    if depth['min_obstacle_height'] >= depth['max_obstacle_height']:
        raise ValueError('Invalid depth obstacle height interval')
    monitor_depth = robot['collision_monitor']['ros__parameters'].get('depth', {})
    for key in ('min_height', 'max_height'):
        if not isinstance(monitor_depth.get(key), (int, float)) or not math.isfinite(monitor_depth[key]):
            raise ValueError(f'robot_params_file must define collision_monitor.depth.{key}')
    if monitor_depth['min_height'] >= monitor_depth['max_height']:
        raise ValueError('Invalid collision monitor depth height interval')
    guard = robot['voxel_guard']['ros__parameters']
    profile = Path(guard['profile'])
    if not profile.is_absolute():
        profile = directory / profile
    body = load_parameters(profile)
    if not isinstance(body.get('park_joints'), dict) or not body['park_joints']:
        raise ValueError('Body profile must contain park_joints')
    if not all(isinstance(value, (int, float)) and math.isfinite(value)
               for value in body['park_joints'].values()):
        raise ValueError('Body park_joints must contain finite joint angles')
    if not use_sim_time and body.get('simulation_park_limits'):
        raise ValueError('Simulation joint limit overrides are forbidden for real-time navigation')
    guard['profile'] = str(profile.resolve())
    projection = robot['depth_clearance']['ros__parameters']
    if not use_sim_time and projection.get('positive_infinity_is_free', False):
        raise ValueError('Simulation depth clearing is forbidden for real-time navigation')
    for node in ('voxel_guard', 'depth_clearance'):
        if not use_sim_time and robot[node]['ros__parameters'].get('allow_simulation_limits', False):
            raise ValueError('Simulation joint limits are forbidden for real-time navigation')


def configure_safety(document: dict, *, pan_enabled: bool = True, memory_enabled: bool = True,
                     voxel_enabled: bool = True, output_directory: str = '') -> dict:
    if voxel_enabled and not memory_enabled:
        raise ValueError('voxel_guard requires obstacle_memory')
    result = deepcopy(document)
    def params(name):
        return result.setdefault(name, {}).setdefault('ros__parameters', {})
    pan = params('pan_motion_gate')
    pan['translation_angle_deg'] = result.get('controller_server', {}).get('ros__parameters', {}).get('FollowPath', {}).get('axis_translation_angle_deg', 0.)
    pan.update(command_topic='nav2/cmd_vel', output_topic='pan_ready/cmd_vel')
    monitor = params('collision_monitor')
    monitor.update(cmd_vel_in_topic='pan_ready/cmd_vel' if pan_enabled else 'nav2/cmd_vel',
                   cmd_vel_out_topic='safety_2d/cmd_vel' if voxel_enabled else 'cmd_vel')
    guard = params('voxel_guard')
    guard.update(command_topic='safety_2d/cmd_vel', output_topic='cmd_vel')
    memory = params('obstacle_memory')
    if voxel_enabled:
        memory.update(publish_3d=True, mark_all_depth_points=True)
    if memory_enabled:
        for name in ('global_costmap', 'local_costmap'):
            costmap = result[name][name]['ros__parameters']
            if 'memory_layer' not in costmap['plugins']:
                costmap['plugins'].insert(1, 'memory_layer')
            costmap['use_maximum'] = True
            costmap['memory_layer'] = dict(plugin='nav2_costmap_2d::StaticLayer',
                map_topic='/obstacle_memory/grid', map_subscribe_transient_local=True,
                subscribe_to_updates=False)
    if output_directory:
        for node in ('obstacle_memory', 'voxel_guard'):
            params(node)['output_directory'] = str(Path(output_directory).resolve()/node)
    return result


def longitudinal_lattice(source: dict) -> dict:
    """Keep Nav2 diff primitives that translate straight or rotate in place."""
    result = deepcopy(source)
    result['primitives'] = [p for p in result['primitives'] if p['arc_length'] == 0.]
    for index, primitive in enumerate(result['primitives']):
        primitive['trajectory_id'] = index
    headings = result['lattice_metadata']['num_of_headings']
    for heading in range(headings):
        candidates = [p for p in result['primitives'] if p['start_angle_index'] == heading]
        if not any(p['straight_length'] > 0 for p in candidates) or not any(
                p['trajectory_length'] == 0 and p['start_angle_index'] != p['end_angle_index']
                for p in candidates):
            raise ValueError(f'Missing longitudinal/rotation primitives for heading {heading}')
    result['lattice_metadata']['number_of_trajectories'] = len(result['primitives'])
    return result
