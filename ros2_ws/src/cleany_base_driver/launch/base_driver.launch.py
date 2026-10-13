from pathlib import Path
import yaml
from cleany_base_driver.core import Geometry, Limits
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_profile(config, mock):
    config = Path(config)
    with config.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if data.get('mode') == 'synthetic' and not mock:
        raise RuntimeError('synthetic base configuration requires mock:=true')
    if mock and data.get('mode') != 'synthetic':
        raise RuntimeError('mock mode requires synthetic configuration')
    geo, lim = data.get('geometry', {}), data.get('limits', {})
    required = (geo.get('wheel_radius_m'), geo.get('wheelbase_m'),
                geo.get('wheel_separation_m'), lim.get('linear_x_mps'),
                lim.get('linear_y_mps'), lim.get('angular_z_rad_s'),
                lim.get('wheel_rad_s'), lim.get('command_timeout_s'))
    if any(v is None for v in required):
        raise RuntimeError('real geometry and velocity limits must be explicitly reviewed')
    Geometry(required[0], required[1], required[2])
    Limits(*required[3:])
    protocol = data.get('protocol', {})
    if protocol.get('counts_per_revolution') != 3172 or protocol.get('watchdog_ms') != 250:
        raise RuntimeError('WheelState contract requires scale 3172 and watchdog 250 ms')
    return data, required


def _nodes(context):
    config = LaunchConfiguration('config').perform(context)
    mock = LaunchConfiguration('mock').perform(context).lower() == 'true'
    relay = LaunchConfiguration('relay_odom').perform(context).lower() == 'true'
    data, required = load_profile(config, mock)
    params = {'mock': mock, 'geometry.wheel_radius_m': required[0],
              'geometry.wheelbase_m': required[1], 'geometry.wheel_separation_m': required[2],
              'limits.linear_x_mps': required[3], 'limits.linear_y_mps': required[4],
              'limits.angular_z_rad_s': required[5], 'limits.wheel_rad_s': required[6],
              'limits.command_timeout_s': required[7],
              'counts_per_revolution': data['protocol']['counts_per_revolution']}
    nodes = [Node(package='cleany_base_driver', executable='base_driver_node',
                  name='cleany_base_driver', parameters=[params], output='screen'),
             Node(package='cleany_base_odometry', executable='wheel_odometry_node',
                  name='wheel_odometry', parameters=[{
                      'input_topic': 'joint_states', 'output_topic': 'wheel/odom',
                      'wheel_radius_m': required[0], 'wheelbase_m': required[1],
                      'wheel_separation_m': required[2]}], output='screen')]
    if mock:
        nodes.append(Node(package='cleany_base_driver', executable='mock_mcu_node',
                          name='mock_base_mcu', output='screen'))
    if relay:
        nodes.append(Node(package='cleany_base_driver', executable='odom_relay_node',
                          name='base_odom_relay', output='screen'))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('cleany_base_driver'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config' / 'base_synthetic.yaml')),
        DeclareLaunchArgument('mock', default_value='true'),
        DeclareLaunchArgument('relay_odom', default_value='true'),
        OpaqueFunction(function=_nodes),
    ])
