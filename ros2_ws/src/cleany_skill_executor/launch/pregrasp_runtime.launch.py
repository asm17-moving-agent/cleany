"""Plan-only runtime using live RGB-D/state and external learned services."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from cleany_skill_executor.motion_runtime import motion_launch_arguments, motion_runtime_prefix, selector_exit_handler


def _setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    backend = value('motion_backend')
    if backend not in ('moveit', 'curobo_ik_moveit'):
        raise ValueError('This sensor OctoMap profile requires MoveIt; cuRobo map conversion is unavailable')
    sim = value('use_sim_time') == 'true'
    skill = Path(get_package_share_directory('cleany_skill_executor'))
    moveit = Path(get_package_share_directory('cleany_moveit_config'))
    selector_parameters = [str(skill / 'config/physical_grasp_selection.yaml'), str(skill / 'config/pregrasp.yaml')]
    if backend == 'curobo_ik_moveit':
        selector_parameters.append(str(skill / 'config/hybrid_motion.yaml'))
    selector = Node(package='cleany_skill_executor', executable='physical_grasp_selection_server', output='screen',
        prefix=motion_runtime_prefix(),
        parameters=[*selector_parameters,
            {'motion_backend': backend, 'use_sim_time': sim,
             'visibility_camera_frame': value('camera_frame')}])
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(moveit / 'launch/physical_move_group.launch.py')),
            launch_arguments={'use_sim_time': value('use_sim_time'),
                              'use_rviz': value('use_rviz'),
                              'robot_description_file': value('robot_description_file'),
                              'include_head_camera': value('include_head_camera'),
                              'enable_depth_octomap': 'true',
                              'allow_trajectory_execution': 'false'}.items()),
        Node(package='cleany_perception', executable='depth_scene_node', output='screen',
             parameters=[{'use_sim_time': sim,
                          'pixel_stride': int(value('scene_pixel_stride')),
                          'depth_image_topic': value('depth_image_topic'),
                          'depth_info_topic': value('depth_info_topic')}]),
        Node(package='cleany_perception', executable='scene_cloud_receipt_node', output='screen',
             parameters=[{'use_sim_time': sim}]),
        selector_exit_handler(selector), selector,
        Node(package='cleany_skill_executor', executable='validate_pick_server', output='screen',
             parameters=[str(skill / 'config/validate_pick.yaml'), str(skill / 'config/pregrasp.yaml'),
                         {'use_sim_time': sim, 'evidence_output_directory': value('evidence_output_directory')}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('repository_root', default_value=str(Path(__file__).resolve().parents[4])),
        *motion_launch_arguments(),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('robot_description_file', default_value=''),
        DeclareLaunchArgument('include_head_camera', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('motion_backend', default_value='moveit', choices=['moveit','curobo_ik_moveit']),
        DeclareLaunchArgument('camera_frame', default_value='head_camera_rgb_optical_frame'),
        DeclareLaunchArgument('depth_image_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('depth_info_topic', default_value='/camera/depth/camera_info'),
        DeclareLaunchArgument('scene_pixel_stride', default_value='4'),
        DeclareLaunchArgument('evidence_output_directory', default_value=''),
        OpaqueFunction(function=_setup),
    ])
