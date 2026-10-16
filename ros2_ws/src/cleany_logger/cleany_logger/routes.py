"""ROS observation routes using the shared logger source schema."""

from .sources import (
    SCHEMA,
    resolved,
    source_roles,
    validate_source_enabled,
    validate_sources,
    maximum_reference,
    raw_maximum_references,
)


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
