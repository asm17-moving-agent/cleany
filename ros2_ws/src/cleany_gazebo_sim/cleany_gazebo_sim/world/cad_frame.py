"""Adapt the shared CAD description to the existing Gazebo navigation contract."""
from __future__ import annotations

from copy import deepcopy
from itertools import product
from pathlib import Path
import subprocess
import shutil
from tempfile import TemporaryDirectory
from xml.etree import ElementTree as ET

import numpy as np
import yaml


from cleany_gazebo_sim.world.urdf_geometry import rpy, origin, freeze


def collision_bounds(urdf: ET.Element, transforms: dict[str, np.ndarray]) -> list[list[float]]:
    points = []
    for link in urdf.findall('link'):
        for collision in link.findall('collision'):
            geometry = collision.find('geometry')
            shape = next(iter(geometry))
            pose = transforms[link.get('name')] @ origin(collision.find('origin'))
            if shape.tag == 'mesh':
                path = Path(shape.get('filename'))
                raw = path.read_bytes()
                count = int.from_bytes(raw[80:84], 'little')
                if len(raw) != 84+50*count:
                    raise ValueError(f'Expected binary STL collision: {path}')
                dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
                vertices = np.frombuffer(raw, dtype=dtype, offset=84)['vertices'].reshape(-1, 3)
                vertices = vertices * np.fromstring(shape.get('scale', '1 1 1'), sep=' ')
                points.extend(vertices @ pose[:3, :3].T + pose[:3, 3])
                continue
            else:
                if shape.tag == 'box':
                    half = np.fromstring(shape.get('size'), sep=' ')/2
                elif shape.tag == 'sphere':
                    extent = np.full(3, float(shape.get('radius')))
                    points.extend([pose[:3, 3]-extent, pose[:3, 3]+extent])
                    continue
                elif shape.tag == 'cylinder':
                    radius = float(shape.get('radius'))
                    half_length = float(shape.get('length'))/2
                    extent = radius*np.sqrt(pose[:3, 0]**2+pose[:3, 1]**2)+half_length*np.abs(pose[:3, 2])
                    points.extend([pose[:3, 3]-extent, pose[:3, 3]+extent])
                    continue
                else:
                    raise ValueError(f'Unsupported collision shape: {shape.tag}')
                low, high = -half, half
            corners = np.array(list(product(*zip(low, high))))
            pose = transforms[link.get('name')] @ origin(collision.find('origin'))
            points.extend(corners @ pose[:3, :3].T + pose[:3, 3])
    return [np.min(points, axis=0).tolist(), np.max(points, axis=0).tolist()]


def adapt_cad_frame(legacy: ET.Element, description: Path, config: Path) -> tuple[ET.Element, dict]:
    import xacro
    profile = yaml.safe_load(config.read_text())
    urdf = ET.fromstring(xacro.process_file(str(description/'urdf/cleany.urdf.xacro')).toxml())
    for mesh in urdf.findall('.//mesh'):
        filename = mesh.get('filename')
        if not filename.startswith('package://cleany_description/'):
            raise ValueError(f'Unexpected mesh URI: {filename}')
        path = description / filename.removeprefix('package://cleany_description/')
        if not path.is_file():
            raise FileNotFoundError(path)
        mesh.set('filename', str(path.resolve()))
    pan_enabled = profile.get('dynamic_pan', False)
    transforms = freeze(urdf, profile['park_joints'], profile.get('simulation_park_limits'), keep_pan=pan_enabled)
    bounds = collision_bounds(urdf, transforms)
    # Include rendered geometry too: convex collision meshes may be smaller.
    visual_urdf = deepcopy(urdf)
    for link in visual_urdf.findall('link'):
        for element in list(link):
            if element.tag == 'collision':
                link.remove(element)
            elif element.tag == 'visual':
                element.tag = 'collision'
    visual_bounds = collision_bounds(visual_urdf, transforms)
    bounds = [np.minimum(bounds[0], visual_bounds[0]).tolist(),
              np.maximum(bounds[1], visual_bounds[1]).tolist()]
    with TemporaryDirectory(prefix='cleany-cad-') as directory:
        source = Path(directory)/'robot.urdf'
        ET.ElementTree(urdf).write(source, encoding='unicode')
        sdf_cli = shutil.which('gz') or shutil.which('ign')
        if sdf_cli is None:
            raise RuntimeError('CAD conversion requires Gazebo gz sdf or ign sdf')
        converted = subprocess.run([sdf_cli, 'sdf', '-p', str(source)], check=True, capture_output=True, text=True)
    model = ET.fromstring(converted.stdout).find('model')
    if model is None or len(model.findall('link')) != 5 + int(pan_enabled) or len(model.findall('joint')) != 4 + int(pan_enabled):
        raise ValueError('CAD navigation model must have base plus four driven wheels')
    model.set('name', 'cleany_mecanum')
    model.insert(0, deepcopy(legacy.find('pose')))
    base = model.find("link[@name='base_link']")
    for prefix in ('rear_left', 'rear_right', 'front_left', 'front_right'):
        wheel = model.find(f"link[@name='{prefix}_wheel_link']")
        for collision in wheel.findall('collision'):
            wheel.remove(collision)
        contact = deepcopy(legacy.find(f"link[@name='{prefix}_wheel']/collision"))
        geometry = contact.find('geometry')
        geometry.clear()
        cylinder = ET.SubElement(geometry, 'cylinder')
        ET.SubElement(cylinder, 'radius').text = str(profile['wheel_radius'])
        ET.SubElement(cylinder, 'length').text = str(profile['wheel_width'])
        ET.SubElement(contact, 'pose').text = '0 0 0 1.5707963267948966 0 0'
        wheel.append(contact)
    # Keep the established evaluation LiDAR/IMU mounts; head frames come from CAD.
    for frame in legacy.findall('frame'):
        name = frame.get('name')
        if name.startswith(('lidar_', 'imu_', 'left_wrist_camera_', 'right_wrist_camera_')):
            frame = deepcopy(frame)
            for side in ('left', 'right'):
                old, new = side+'_fixed_jaw', side+'_gripper_frame'
                if frame.get('attached_to') == old:
                    frame.set('attached_to', new)
                if frame.find('pose').get('relative_to') == old:
                    frame.find('pose').set('relative_to', new)
            model.append(frame)
    for sensor in legacy.findall('.//sensor'):
        sensor = deepcopy(sensor)
        if pan_enabled and sensor.get('name', '').startswith('head_realsense'):
            model.find("link[@name='head_pan_link']").append(sensor)
        else:
            base.append(sensor)
    for plugin in legacy.findall('plugin'):
        plugin = deepcopy(plugin)
        if plugin.get('name', '').endswith('MecanumDrive'):
            for key in ('wheelbase', 'wheel_separation', 'wheel_radius'):
                plugin.find(key).text = str(profile[key])
        model.append(plugin)
    if pan_enabled:
        fortress = any(p.get('name', '').startswith('ignition::') for p in legacy.findall('plugin'))
        filename = 'ignition-gazebo-joint-position-controller-system' if fortress else 'gz-sim-joint-position-controller-system'
        namespace = 'ignition::gazebo' if fortress else 'gz::sim'
        plugin = ET.SubElement(model, 'plugin', filename=filename,
                               name=namespace+'::systems::JointPositionController')
        for key, value in dict(joint_name='head_pan_joint', initial_position='0',
                use_velocity_commands='true', cmd_max=str(profile.get('pan_speed_rad_s', 1.0)),
                topic='/model/cleany_mecanum/head_pan/cmd_pos').items():
            ET.SubElement(plugin, key).text = value
    camera = transforms['head_camera_depth_frame']
    metadata = dict(source_commit=profile['source_commit'], collision_bounds=bounds,
                    navigation_margins=profile['navigation_margins'],
                    depth_camera_tf=dict(translation=camera[:3, 3].tolist(), rotation_rpy=rpy(camera[:3, :3])),
                    wheelbase=profile['wheelbase'], wheel_separation=profile['wheel_separation'],
                    wheel_radius=profile['wheel_radius'], park_joints=profile['park_joints'],
                    simulation_park_limits=profile.get('simulation_park_limits', {}))
    if pan_enabled:
        metadata['depth_camera_tf']['pan_pivot'] = transforms['head_pan_link'][:3, 3].tolist()
    return model, metadata


def navigation_geometry(model: dict) -> dict:
    """Derive offset polygons from the complete parked robot, not a centered proxy."""
    low, high = np.asarray(model['collision_bounds'], dtype=float)
    if not np.all(np.isfinite([low, high])) or np.any(high <= low):
        raise ValueError('Invalid model bounds')
    margins = model['navigation_margins']
    if any(not np.isfinite(v) or v < 0 for v in margins.values()):
        raise ValueError('Invalid navigation margin')
    if margins['slowdown'] < margins['stop']:
        raise ValueError('Slowdown margin must contain stop margin')
    def polygon(margin):
        x0, y0 = low[:2]-margin
        x1, y1 = high[:2]+margin
        return [[float(x0), float(y0)], [float(x0), float(y1)],
                [float(x1), float(y1)], [float(x1), float(y0)]]
    return dict(physical=polygon(0), stop=polygon(margins['stop']),
                slow=polygon(margins['slowdown']),
                planning_padding=margins['planning_padding'],
                center=((low[:2]+high[:2])/2).tolist(),
                half_size=((high[:2]-low[:2])/2).tolist())
