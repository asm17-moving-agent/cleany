"""Read-only motor/tracking subscriber and grasp/lift evidence service."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=PathJoinSubstitution([
            FindPackageShare('cleany_skill_executor'),'config','manipulation_evidence.yaml'])),
        DeclareLaunchArgument('contact_rule_file', default_value=PathJoinSubstitution([
            FindPackageShare('cleany_skill_executor'), 'config', 'gripper_contact_decision.yaml'])),
        Node(package='cleany_skill_executor',executable='manipulation_evidence',
             parameters=[LaunchConfiguration('config'), {
                 'use_sim_time': False,
                 'contact_rule_file': LaunchConfiguration('contact_rule_file'),
             }], output='screen'),
    ])
