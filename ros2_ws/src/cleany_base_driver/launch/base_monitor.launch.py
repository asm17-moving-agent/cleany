"""Observation-only rqt views for an already-running base driver."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WHEELS = {'fl': 0, 'fr': 1, 'rl': 2, 'rr': 3}


def _nodes(context):
    preset = LaunchConfiguration('preset', default='false').perform(context).lower()
    if preset not in ('true', 'false'):
        raise RuntimeError("preset must be true or false")
    if preset == 'true':
        perspective = (get_package_share_directory('cleany_base_driver')
                       + '/config/base_monitor.perspective')
        return [
            Node(package='rqt_gui', executable='rqt_gui',
                 arguments=['--force-discover', '--perspective-file', perspective],
                 remappings=[('/diagnostics_agg', '/diagnostics')], output='screen'),
        ]
    wheel = LaunchConfiguration('wheel').perform(context).lower()
    if wheel != 'all' and wheel not in WHEELS:
        raise RuntimeError("wheel must be one of fl, fr, rl, rr, all")
    indices = range(4) if wheel == 'all' else (WHEELS[wheel],)
    speed_fields = ('target_rad_s', 'commanded_rad_s', 'velocity_rad_s')
    speed_topics = [f'/base/wheel_state/{field}[{index}]'
                    for index in indices for field in speed_fields]
    pwm_topics = [f'/base/wheel_state/pwm_percent[{index}]' for index in indices]
    return [
        Node(package='rqt_plot', executable='rqt_plot', name='base_speed_plot',
             arguments=['--force-discover', '--empty', *speed_topics], output='screen'),
        Node(package='rqt_plot', executable='rqt_plot', name='base_pwm_plot',
             arguments=['--force-discover', '--empty', *pwm_topics], output='screen'),
        Node(package='rqt_robot_monitor', executable='rqt_robot_monitor',
             name='base_diagnostics_monitor', arguments=['--force-discover'],
             remappings=[('diagnostics_agg', '/diagnostics')], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('wheel', default_value='fl',
                              description='Wheel view: fl, fr, rl, rr, or all'),
        DeclareLaunchArgument('preset', default_value='false',
                              description='Load the saved all-wheel rqt dashboard'),
        OpaqueFunction(function=_nodes),
    ])
