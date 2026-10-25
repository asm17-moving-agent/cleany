from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = get_package_share_directory('cleany_system_supervisor') + '/config/system_supervisor.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('tof_required', default_value='false'),
        DeclareLaunchArgument('config_file', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='cleany_system_supervisor', executable='system_supervisor',
             parameters=[{'config_file': LaunchConfiguration('config_file'),
                          'tof_required': ParameterValue(LaunchConfiguration('tof_required'), value_type=bool),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}], output='screen'),
    ])
