"""공통 Cleany 주행 런타임에 Gazebo 센서와 TF 어댑터를 연결합니다."""
from pathlib import Path
from tempfile import mkdtemp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from cleany_navigation.runtime_config import load_parameters


def _launch(context: LaunchContext) -> list:
    share = Path(get_package_share_directory('cleany_gazebo_sim'))
    navigation = Path(get_package_share_directory('cleany_navigation'))
    def value(name):
        return LaunchConfiguration(name).perform(context)
    safety = load_parameters(Path(value('safety_params_file')))
    camera = safety.pop('depth_camera_tf')
    robot = safety
    base = load_parameters(Path(value('params_file')))
    for name in ('local_costmap', 'global_costmap'):
        layer = base[name][name]['ros__parameters']['obstacle_layer']
        robot[name][name]['ros__parameters']['obstacle_layer'] = {
            'min_obstacle_height': layer['min_obstacle_height'],
            'scan': {'min_obstacle_height': layer['scan']['min_obstacle_height']}}
    # 이 래퍼는 명시적 주차 자세 재정의를 포함한 시뮬레이션 정보만 제공합니다.
    robot['voxel_guard'] = load_parameters(Path(value('voxel_params_file')))['voxel_guard']
    robot['voxel_guard']['ros__parameters'].update(profile=value('body_profile'), allow_simulation_limits=True)
    robot.setdefault('obstacle_memory', {'ros__parameters': {}})['ros__parameters'].update(floor_z=-.38)
    projection = robot.setdefault('depth_clearance', {}).setdefault('ros__parameters', {})
    projection.update(camera_frame='head_camera_depth_frame', optical_frame='head_camera_depth_optical_frame',
                      allow_simulation_limits=True)
    dynamic_pan = 'pan_pivot' in camera
    if dynamic_pan:
        projection['self_filter_profile'] = value('body_profile')
    for key in ('origin_z', 'min_obstacle_height', 'max_obstacle_height'):
        robot['local_costmap']['local_costmap']['ros__parameters']['depth_layer'].setdefault(
            key, safety['local_costmap']['local_costmap']['ros__parameters']['depth_layer'][key])
    directory = Path(mkdtemp(prefix='cleany-gazebo-navigation-'))
    robot_file = directory/'robot.yaml'
    robot_file.write_text(yaml.safe_dump(robot, sort_keys=False))
    adapters = []
    if dynamic_pan:
        adapters.extend([
            Node(package='ros_gz_bridge', executable='parameter_bridge', name='head_pan_bridge', output='screen',
                 arguments=['/model/cleany_mecanum/head_pan/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double'],
                 remappings=[('/model/cleany_mecanum/head_pan/cmd_pos', '/head_pan/command')]),
            Node(package='cleany_gazebo_sim', executable='gazebo_pan_tf_publisher',
                 parameters=[camera, {'use_sim_time': True}], output='screen'),
        ])
    else:
        adapters.append(Node(package='tf2_ros', executable='static_transform_publisher', name='depth_camera_tf',
            parameters=[{'use_sim_time': True}], arguments=[
                '--frame-id', 'base_link', '--child-frame-id', 'head_camera_depth_frame',
                *[item for key, number in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'),
                    camera['translation']+camera['rotation_rpy']) for item in ('--'+key, str(number))]]))
    adapters.append(Node(package='tf2_ros', executable='static_transform_publisher', name='depth_optical_tf',
        parameters=[{'use_sim_time': True}], arguments=[
            '--frame-id', 'head_camera_depth_frame', '--child-frame-id', 'head_camera_depth_optical_frame',
            '--roll', '-1.5707963267948966', '--pitch', '0', '--yaw', '-1.5707963267948966']))
    return [*adapters, IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(navigation/'launch/navigation.launch.py')),
        launch_arguments={
            'map': value('map'), 'robot_params_file': str(robot_file), 'use_sim_time': value('use_sim_time'),
            'autostart': value('autostart'), 'axis_motion': value('axis_motion'),
            'axis_params_file': value('axis_params_file'), 'omni_params_file': value('omni_params_file'), 'params_file': value('params_file'),
            'localization_params_file': value('localization_params_file'),
            # 포함된 launch 사이에는 인자 컨텍스트가 공유됩니다. Gazebo 메타데이터 YAML이
            # 공통 ROS 파라미터 overlay로 전달되지 않게 합니다.
            'safety_params_file': str(navigation/'config/nav2_safety.yaml'),
            'pan_enabled': str(dynamic_pan).lower(), 'memory_enabled': value('memory_enabled'),
            'voxel_enabled': value('voxel_enabled'), 'safety_mode': value('safety_mode'), 'output_directory': value('output_directory'),
            'memory_params_file': value('memory_params_file'),
        }.items())]


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('cleany_gazebo_sim'))
    navigation = Path(get_package_share_directory('cleany_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('axis_motion', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=str(share/'config/nav2_amcl.yaml')),
        DeclareLaunchArgument('localization_params_file', default_value=''),
        DeclareLaunchArgument('safety_params_file', default_value=str(share/'config/nav2_safety.yaml')),
        DeclareLaunchArgument('axis_params_file', default_value=str(navigation/'config/nav2_axis_motion.yaml')),
        DeclareLaunchArgument('omni_params_file', default_value=str(navigation/'config/nav2_omni_motion.yaml')),
        DeclareLaunchArgument('voxel_params_file', default_value=str(navigation/'config/voxel_guard.yaml')),
        DeclareLaunchArgument('memory_params_file', default_value=str(navigation/'config/obstacle_memory.yaml')),
        DeclareLaunchArgument('body_profile', default_value=str(share/'config/cad_frame.yaml')),
        # 반응형 평가에서는 카메라 관측과 주행을 분리합니다.
        DeclareLaunchArgument('safety_mode', default_value='reactive', choices=['serial', 'reactive']),
        DeclareLaunchArgument('memory_enabled', default_value='true'),
        DeclareLaunchArgument('voxel_enabled', default_value='false'),
        DeclareLaunchArgument('output_directory', default_value=''),
        OpaqueFunction(function=_launch),
    ])
