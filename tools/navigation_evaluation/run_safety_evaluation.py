#!/usr/bin/env python3
"""Run an isolated Gazebo/Nav2 safety evaluation; retain evidence on failure."""

from __future__ import annotations

import argparse
import ast
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import rclpy
import yaml

from cleany_gazebo_sim.gazebo_slam_experiment import load_mount_profiles, write_sensor_tf_config
from cleany_gazebo_sim.lidar_noise import load_lidar_noise_profile
from cleany_gazebo_sim.world.generator import materialize_study_cafe_world
from safety_probe import SafetyProbe


def prepare(output: Path, map_yaml: Path, robot_model: str = 'legacy', initial_pose: list[float] | None = None,
            obstacle_memory: bool = False, memory_restore: Path | None = None, guard_3d: bool = False) -> dict:
    if guard_3d and (not obstacle_memory or robot_model != 'cad_frame'):
        raise ValueError('3D guard requires CAD model and obstacle memory')
    share = Path(get_package_share_directory('cleany_gazebo_sim'))
    config = output / 'config'
    config.mkdir()
    for name in ('nav2_amcl.yaml', 'nav2_safety.yaml', 'nav2_safety_evaluation.yaml', 'odometry_error_ideal.yaml', 'nav2_axis_motion.yaml'):
        shutil.copy2(share/'config'/name, config/name)
    shutil.copy2(share/'config/study_cafe/study_cafe_layout.yaml', config/'study_cafe_layout.yaml')
    metadata = yaml.safe_load(map_yaml.read_text())
    image_path = (map_yaml.parent / metadata['image']).resolve()
    shutil.copy2(image_path, output/'map.pgm')
    metadata['image'] = 'map.pgm'
    (output/'map.yaml').write_text(yaml.safe_dump(metadata))
    profile = load_mount_profiles(share/'config/lidar_mount_profiles.yaml')['floor_30cm']
    world = materialize_study_cafe_world(
        share/'worlds/cleany_mecanum_harmonic.sdf', target_path=output/'world.sdf',
        lidar_translation=profile.transform.translation,
        lidar_noise=load_lidar_noise_profile(share/'config/lidar_noise_profiles.yaml', 'measured'),
        robot_model=robot_model,
    )
    odometry_share = Path(get_package_share_directory('cleany_base_odometry'))
    wheel = yaml.safe_load((odometry_share/'config/wheel_odometry.yaml').read_text())
    model_metadata = None
    if robot_model == 'cad_frame':
        shutil.copy2(share/'config/cad_frame.yaml', config/'cad_frame.yaml')
        model_metadata = json.loads(world.with_suffix('.model.json').read_text())
        wheel_params = wheel['wheel_odometry']['ros__parameters']
        for key in ('wheel_radius', 'wheelbase', 'wheel_separation'):
            wheel_params[key+'_m'] = model_metadata[key]
        safety = yaml.safe_load((config/'nav2_safety.yaml').read_text())
        safety['depth_camera_tf'] = model_metadata['depth_camera_tf']
        (config/'nav2_safety.yaml').write_text(yaml.safe_dump(safety, sort_keys=False))
    (config/'wheel_odometry.yaml').write_text(yaml.safe_dump(wheel, sort_keys=False))
    write_sensor_tf_config(profile, config/'sensor_tf.yaml')
    tree = ET.parse(world)
    root = tree.getroot().find('world')
    if initial_pose is not None:
        if len(initial_pose) != 3 or not all(math.isfinite(v) for v in initial_pose):
            raise ValueError('Initial pose must have three finite values')
        spawn = yaml.safe_load((config/'study_cafe_layout.yaml').read_text())['robot']['spawn_pose']
        x, y, yaw = initial_pose
        c, s = math.cos(spawn[5]), math.sin(spawn[5])
        pose = [spawn[0]+c*x-s*y, spawn[1]+s*x+c*y, spawn[2], 0.0, 0.0, spawn[5]+yaw]
        root.find("model[@name='cleany_mecanum']/pose").text = ' '.join(map(str, pose))
    evaluation = yaml.safe_load((config/'nav2_safety_evaluation.yaml').read_text())
    fixtures = evaluation['fixtures']
    for name, fixture in fixtures.items():
        model = ET.SubElement(root, 'model', name='safety_'+name)
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = f'50 50 {fixture["z"]} 0 0 0'
        link = ET.SubElement(model, 'link', name='body')
        for kind in ('collision', 'visual'):
            element = ET.SubElement(link, kind, name=kind)
            box = ET.SubElement(ET.SubElement(element, 'geometry'), 'box')
            ET.SubElement(box, 'size').text = ' '.join(map(str, fixture['size']))
            if kind == 'visual':
                ET.SubElement(ET.SubElement(element, 'material'), 'diffuse').text = '0.9 0.15 0.1 1'
    tree.write(world, encoding='unicode', xml_declaration=True)
    nav = yaml.safe_load((config/'nav2_amcl.yaml').read_text())
    costmap = nav['local_costmap']['local_costmap']['ros__parameters']
    footprint = ast.literal_eval(costmap['footprint'])
    physical_half_size = [max(abs(p[i]) for p in footprint)+costmap['footprint_padding'] for i in (0, 1)]
    if model_metadata is not None:
        low, high = model_metadata['collision_bounds']
        from cleany_gazebo_sim.world.cad_frame import navigation_geometry
        geometry = navigation_geometry(model_metadata)
        safety = yaml.safe_load((config/'nav2_safety.yaml').read_text())
        for name in ('local_costmap', 'global_costmap'):
            layer = safety[name][name]['ros__parameters']
            layer['footprint'] = str(geometry['stop'])
            layer['footprint_padding'] = geometry['planning_padding']
        monitor = safety['collision_monitor']['ros__parameters']
        monitor['StopZone']['points'] = str(geometry['stop'])
        monitor['SlowZone']['points'] = str(geometry['slow'])
        (config/'nav2_safety.yaml').write_text(yaml.safe_dump(safety, sort_keys=False))
        physical_half_size = [v+geometry['planning_padding'] for v in geometry['half_size']]
        model_metadata['navigation_geometry'] = geometry
    if obstacle_memory:
        safety = yaml.safe_load((config/'nav2_safety.yaml').read_text())
        global_map = safety['global_costmap']['global_costmap']['ros__parameters']
        global_map['plugins'] = ['static_layer', 'memory_layer', 'obstacle_layer', 'inflation_layer']
        global_map['use_maximum'] = True
        global_map['memory_layer'] = dict(plugin='nav2_costmap_2d::StaticLayer',
                                        map_topic='/obstacle_memory/grid', map_subscribe_transient_local=True,
                                        subscribe_to_updates=False)
        (config/'nav2_safety.yaml').write_text(yaml.safe_dump(safety, sort_keys=False))
        memory_params = dict(use_sim_time=True, output_directory=str(output/'obstacle_memory'),
                             restore_path=str(memory_restore.resolve()) if memory_restore else '')
        if guard_3d:
            memory_params.update(publish_3d=True, mark_all_depth_points=True, z_resolution=.05, update_period=.2)
        (config/'obstacle_memory.yaml').write_text(yaml.safe_dump({'obstacle_memory': {'ros__parameters': memory_params}}))
    if guard_3d:
        safety = yaml.safe_load((config/'nav2_safety.yaml').read_text())
        safety['collision_monitor']['ros__parameters']['cmd_vel_out_topic'] = '/safety_2d/cmd_vel'
        (config/'nav2_safety.yaml').write_text(yaml.safe_dump(safety, sort_keys=False))
        guard = yaml.safe_load((share/'config/voxel_guard.yaml').read_text())
        guard['voxel_guard']['ros__parameters'].update(profile=str(config/'cad_frame.yaml'),
            output_directory=str(output/'voxel_guard'))
        (config/'voxel_guard.yaml').write_text(yaml.safe_dump(guard))
    scenario = dict(guard_3d=guard_3d, world=root.get('name'), fixtures=fixtures, evaluation=evaluation,
                    obstacle_memory=obstacle_memory,
                    robot_half_size=physical_half_size,
                    robot_center=model_metadata['navigation_geometry']['center'] if model_metadata else [0.0, 0.0],
                    expected_slowdown_ratio=yaml.safe_load((config/'nav2_safety.yaml').read_text())['collision_monitor']['ros__parameters']['SlowZone']['slowdown_ratio'],
                    spawn_pose=yaml.safe_load((config/'study_cafe_layout.yaml').read_text())['robot']['spawn_pose'],
                    robot_model=robot_model, model_metadata=model_metadata, initial_pose=initial_pose or [0.0, 0.0, 0.0],
                    map_source=str(map_yaml), map_sha256=hashlib.sha256(image_path.read_bytes()).hexdigest(),
                    lidar_height_m=0.30, lidar_noise='measured', wheel_stress_level=0,
                    depth_recording='Every eighth point of adapted cloud, at most ~6.7 Hz; Nav2 receives the entire adapted cloud')
    (output/'scenario.json').write_text(json.dumps(scenario, indent=2))
    launch_source = '''from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
def generate_launch_description():
    p = Path(__file__).resolve().parent
    share = Path(get_package_share_directory('cleany_gazebo_sim'))
    memory = [Node(package='cleany_gazebo_sim', executable='obstacle_memory', name='obstacle_memory',
                   parameters=[str(p/'config/obstacle_memory.yaml')], output='screen')] if (p/'config/obstacle_memory.yaml').exists() else []
    guard = [Node(package='cleany_gazebo_sim', executable='voxel_guard', name='voxel_guard',
                  parameters=[str(p/'config/voxel_guard.yaml')], output='screen')] if (p/'config/voxel_guard.yaml').exists() else []
    return LaunchDescription(memory+guard+[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/gazebo_harmonic.launch.py')),
            launch_arguments={'world':str(p/'world.sdf'), 'headless':'true', 'use_sim_time':'true',
                              'sensor_profile':'lidar_depth_nav', 'sensor_config':str(p/'config/sensor_tf.yaml'),
                              'odometry_source':'wheel', 'wheel_odometry_config':str(p/'config/wheel_odometry.yaml'),
                              'odometry_error_config':str(p/'config/odometry_error_ideal.yaml')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/amcl_nav2_safety.launch.py')),
            launch_arguments={'map':str(p/'map.yaml'), 'params_file':str(p/'config/nav2_amcl.yaml'),
                              'safety_params_file':str(p/'config/nav2_safety.yaml'),
                              'axis_params_file':str(p/'config/nav2_axis_motion.yaml')}.items()),
    ])
'''
    (output/'run.launch.py').write_text(launch_source)
    scripts = output/'scripts'
    scripts.mkdir()
    for script in Path(__file__).parent.glob('*.py'):
        shutil.copy2(script, scripts/script.name)
    code = output/'code'
    code.mkdir()
    for name in ('depth_clearance', 'depth_clearance_node', 'navigation_safety_metrics',
                 'obstacle_memory', 'obstacle_memory_node', 'world.cad_frame', 'body_geometry',
                 'point_cloud', 'voxel_guard', 'voxel_guard_node'):
        source = Path(importlib.import_module('cleany_gazebo_sim.'+name).__file__)
        shutil.copy2(source, code/source.name)
    return scenario


def stop(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=25)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='New directory; existing output is never overwritten.')
    parser.add_argument('--mode', choices=('sensors', 'monitor', 'avoid'), default='sensors')
    parser.add_argument('--robot-model', choices=('legacy', 'cad_frame'), default='cad_frame')
    parser.add_argument('--domain-id', type=int, default=202)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['GZ_PARTITION'] = 'cleany_safety_'+uuid.uuid4().hex[:12]
    scenario = prepare(output, args.map.resolve(), robot_model=args.robot_model)
    metadata = dict(mode=args.mode, started=time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                    ros_domain_id=args.domain_id, gz_partition=os.environ['GZ_PARTITION'])
    (output/'experiment.json').write_text(json.dumps(metadata, indent=2))
    launch = recorder = node = None
    result = {'completed': False}
    try:
        with (output/'launch.log').open('w') as launch_log, (output/'recorder.log').open('w') as bag_log:
            launch = subprocess.Popen(['ros2', 'launch', str(output/'run.launch.py')], stdout=launch_log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            topics = ['/scan', '/evaluation/depth_points', '/wheel/odom', '/ground_truth/odom', '/tf', '/tf_static',
                      '/clock', '/amcl_pose', '/cmd_vel', '/nav2/cmd_vel', '/gazebo_cmd_vel',
                      '/collision_monitor_state', '/local_costmap/costmap', '/global_costmap/costmap',
                      '/plan', '/local_plan', '/navigate_to_pose/_action/status']
            recorder = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(output/'bag'), '--topics', *topics],
                                        stdout=bag_log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            node = SafetyProbe(output, scenario)
            node.initialize()
            result = getattr(node, 'run_'+args.mode)()
            result['completed'] = True
    except Exception as error:
        result['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        try:
            if node is not None:
                try:
                    node.finish()
                finally:
                    node.save()
                    node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            try:
                stop(recorder)
            finally:
                stop(launch)
        result['launch_returncode'] = launch.returncode if launch else None
        result['recorder_returncode'] = recorder.returncode if recorder else None
        result['passed'] = bool(result.get('checks')) and all(result['checks'].values())
        (output/'result.json').write_text(json.dumps(result, indent=2))
        printable = {key:value for key,value in result.items() if key != 'feedback'}
        if 'feedback' in result:
            printable['feedback_count'] = len(result['feedback'])
        print(json.dumps({'output':str(output), **printable}), flush=True)
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
