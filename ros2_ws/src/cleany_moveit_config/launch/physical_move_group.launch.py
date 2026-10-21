"""Shared MoveIt server with materialized real URDF and optional sim execution."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from cleany_moveit_config.simulation_collision import ignore_simulation_mast
from cleany_moveit_config.joint_limits import limits_from_model
from cleany_moveit_config.controller_plugin import controller_plugin_environment


def _moveit_config(robot_description_file='', controllers_file='', joint_limits_file=''):
    description = Path(get_package_share_directory('cleany_description')) / 'urdf/cleany.urdf.xacro'
    config = (MoveItConfigsBuilder('cleany', package_name='cleany_moveit_config')
        .robot_description(file_path=str(description), mappings={
            'include_head_camera': LaunchConfiguration('include_head_camera'),
            'include_wheel_joints': 'false'})
        .robot_description_semantic(file_path='config/cleany.srdf.xacro', mappings={
            'include_head_camera': LaunchConfiguration('include_head_camera')})
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path=joint_limits_file or 'config/joint_limits.yaml')
        .trajectory_execution(file_path=controllers_file or 'config/moveit_controllers.yaml',
                              moveit_manage_controllers=False)
        .planning_pipelines(default_planning_pipeline='ompl',
                            pipelines=['ompl', 'pilz_industrial_motion_planner'])
        .planning_scene_monitor().to_moveit_configs())
    if robot_description_file:
        config.robot_description = {'robot_description': Path(robot_description_file).read_text()}
        if not joint_limits_file:
            config.joint_limits = limits_from_model(
                config.joint_limits, config.robot_description['robot_description'])
    return config


def _launch_nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    controllers = value('controllers_file')
    if not controllers and value('enable_gripper_execution') == 'true':
        controllers = 'config/sorting_moveit_controllers.yaml'
    config = _moveit_config(value('robot_description_file'), controllers, value('joint_limits_file'))
    bins = value('simulation_bins_config')
    if bins:
        if value('use_sim_time') != 'true':
            raise ValueError('Simulation bins collision overrides require simulation time')
        fixture = yaml.safe_load(Path(bins).read_text()) or {}
        ignore_mast = fixture.get('simulation_ignore_mast_collision', False)
        if not isinstance(ignore_mast, bool):
            raise ValueError('simulation_ignore_mast_collision must be boolean')
        if ignore_mast:
            config.robot_description_semantic['robot_description_semantic'] = ignore_simulation_mast(
                config.robot_description['robot_description'],
                config.robot_description_semantic['robot_description_semantic'], use_sim_time=True)
    parameters = [config.to_dict()]
    if value('enable_depth_octomap') == 'true':
        parameters += [str(Path(config.package_path) / 'config/depth_octomap.yaml'), {
            'sensors': ['depth_cloud'], 'depth_cloud.sensor_plugin': value('depth_octomap_plugin')}]
    parameters.append({
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'allow_trajectory_execution': ParameterValue(LaunchConfiguration('allow_trajectory_execution'), value_type=bool),
        'publish_robot_description_semantic': True,
        'capabilities': 'cleany/StateValidityBatch',
        'trajectory_execution.allowed_execution_duration_scaling': ParameterValue(
            LaunchConfiguration('allowed_execution_duration_scaling'), value_type=float),
        'trajectory_execution.allowed_goal_duration_margin': ParameterValue(
            LaunchConfiguration('allowed_goal_duration_margin'), value_type=float)})
    move_group = Node(package='moveit_ros_move_group', executable='move_group',
        output='screen', parameters=parameters,
        additional_env=controller_plugin_environment(
            config.trajectory_execution.get('moveit_controller_manager', ''), keep_loaded=True),
        on_exit=Shutdown(reason='MoveIt planning process stopped'))
    rviz = Node(package='rviz2', executable='rviz2', name='moveit_rviz', output='log',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', str(Path(config.package_path) / 'config/moveit.rviz')],
        parameters=[config.robot_description, config.robot_description_semantic,
                    config.robot_description_kinematics, config.joint_limits,
                    config.planning_pipelines, {'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool)}])
    return [move_group, rviz]


def generate_launch_description() -> LaunchDescription:
    defaults = {
        'use_rviz': 'false', 'use_sim_time': 'false',
        'robot_description_file': '', 'controllers_file': '', 'joint_limits_file': '',
        'include_head_camera': 'false', 'enable_depth_octomap': 'false',
        'depth_octomap_plugin': 'cleany_scene_mapping/KnownGeometryOctomapUpdater',
        'simulation_bins_config': '', 'enable_gripper_execution': 'false',
        'allow_trajectory_execution': 'false', 'allowed_execution_duration_scaling': '1.2',
        'allowed_goal_duration_margin': '0.5'}
    return LaunchDescription([*(DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()),
                              OpaqueFunction(function=_launch_nodes)])
