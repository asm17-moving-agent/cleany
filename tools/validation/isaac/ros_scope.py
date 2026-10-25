"""Remap a complete test stack while retaining its node and parameter names."""
import os
import re


def remappings(prefix: str) -> list[tuple[str, str]]:
    if not re.fullmatch(r'env_[0-9]+', prefix):
        raise ValueError('Expected an env_N ROS prefix')
    # Relative endpoints follow the namespace. Humble does not implement '**'
    # remaps, so explicitly scope absolute endpoints used by these adapters.
    names = ['tf', 'tf_static', 'wheel/odom', 'joint_states', 'scan', 'cmd_vel',
             'nav2/cmd_vel', 'collision_monitor_state', 'safety/slow_zone',
             'safety/stop_zone', 'camera/head/depth/points', 'isaac/test/block_base',
             'mission/status', 'mission/result', 'mission/debug_snapshot',
             'mission/offer', 'mission/cancel', 'mission/snapshot',
             'navigate_to_pose/_action/feedback',
             'right_arm_controller/follow_joint_trajectory']
    return [(f'/{name}', f'/{prefix}/{name}') for name in names]


def scoped_ros_args() -> list[str] | None:
    prefix = os.environ.get('CLEANY_ROS_PREFIX')
    if not prefix:
        return None
    args = ['--ros-args', '-r', f'__ns:=/{prefix}']
    for source, destination in remappings(prefix):
        args.extend(['-r', f'{source}:={destination}'])
    return args
