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
        # Keep moving shoulders on the fixed CAD motor shafts by default.
        DeclareLaunchArgument('arm_base_spacing_m', default_value=''),
        DeclareLaunchArgument('head_mount_x_offset_m', default_value='0'),
        DeclareLaunchArgument('head_mast_rear_gap_m', default_value='0.05'),
        DeclareLaunchArgument('include_head_camera', default_value='true'),
        DeclareLaunchArgument('show_body_visual', default_value='false'),
        DeclareLaunchArgument('include_desk_plate', default_value='true'),
        DeclareLaunchArgument('desk_plate_thickness_m', default_value='0.005'),
        DeclareLaunchArgument('head_pan_rad', default_value='-0.007669903939428206'),
        DeclareLaunchArgument('head_tilt_rad', default_value='0.7286408742456796'),
        DeclareLaunchArgument('camera_pair_preview_file', default_value=''),
        DeclareLaunchArgument('right_wrist_calibration_file', default_value=PathJoinSubstitution([
            share, 'config', 'right_wrist_camera_extrinsics.yaml',
        ])),
        DeclareLaunchArgument('left_wrist_calibration_file', default_value=PathJoinSubstitution([
            share, 'config', 'left_wrist_camera_extrinsics.yaml',
        ])),
        DeclareLaunchArgument('left_tcp_file', default_value=PathJoinSubstitution([
            share, 'config', 'left_gripper_tcp.yaml',
        ])),
        Node(package='cleany_robot_interface', executable='arm_state_reader',
             namespace='arm_preview', output='screen', parameters=[{
                 'config_file': LaunchConfiguration('config_file'),
             }]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='arm_preview', output='screen', parameters=[{
                 'robot_description': ParameterValue(
                     Command(['xacro ', LaunchConfiguration('model_file'),
                              ' include_head_camera:=', LaunchConfiguration('include_head_camera'),
                              ' head_fixed:=true',
                              ' right_wrist_calibration_file:=', LaunchConfiguration('right_wrist_calibration_file'),
                              ' left_wrist_calibration_file:=', LaunchConfiguration('left_wrist_calibration_file'),
                              ' left_tcp_file:=', LaunchConfiguration('left_tcp_file'),
                              ' show_body_visual:=', LaunchConfiguration('show_body_visual'),
                              ' include_desk_plate:=', LaunchConfiguration('include_desk_plate'),
                              ' desk_plate_thickness_m:=', LaunchConfiguration('desk_plate_thickness_m'),
                              ' head_pan_rad:=', LaunchConfiguration('head_pan_rad'),
                              ' head_tilt_rad:=', LaunchConfiguration('head_tilt_rad'),
                              ' camera_pair_preview_file:=', LaunchConfiguration('camera_pair_preview_file'),
                              ' arm_base_spacing_m:=', LaunchConfiguration('arm_base_spacing_m'),
                              ' head_mast_rear_gap_m:=', LaunchConfiguration('head_mast_rear_gap_m'),
                              ' head_mount_x_offset_m:=', LaunchConfiguration('head_mount_x_offset_m')]),
                     value_type=str),
                 'use_sim_time': False,
             }], remappings=transforms + [('/joint_states', '/arm_preview/joint_states')]),
        Node(package='rviz2', executable='rviz2', namespace='arm_preview',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution([share, 'config', 'arm_preview.rviz'])],
             remappings=transforms, output='screen'),
    ])
