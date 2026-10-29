from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('cleany_robot_interface')
    model = PathJoinSubstitution([
        FindPackageShare('cleany_description'), 'urdf', 'cleany.urdf.xacro',
    ])
    transforms = [('/tf', '/arm_preview/tf'),
                  ('/tf_static', '/arm_preview/tf_static')]
    return LaunchDescription([
        DeclareLaunchArgument('model_file', default_value=model),
        DeclareLaunchArgument('config_file', default_value=PathJoinSubstitution([
            share, 'config', 'arm_preview.yaml',
        ])),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('head_mast_rear_gap_m', default_value='0.05'),
        DeclareLaunchArgument('include_head_camera', default_value='true'),
        Node(package='cleany_robot_interface', executable='arm_state_reader',
             namespace='arm_preview', output='screen', parameters=[{
                 'config_file': LaunchConfiguration('config_file'),
             }]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='arm_preview', output='screen', parameters=[{
                 'robot_description': ParameterValue(
                     Command(['xacro ', LaunchConfiguration('model_file'),
                              ' include_head_camera:=', LaunchConfiguration('include_head_camera'),
                              ' head_mast_rear_gap_m:=', LaunchConfiguration('head_mast_rear_gap_m')]),
                     value_type=str),
                 'use_sim_time': False,
             }], remappings=transforms + [('/joint_states', '/arm_preview/joint_states')]),
        Node(package='rviz2', executable='rviz2', namespace='arm_preview',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution([share, 'config', 'arm_preview.rviz'])],
             remappings=transforms, output='screen'),
    ])
