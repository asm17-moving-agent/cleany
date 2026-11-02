"""Attach Gazebo wrist optics to the shared v8 simulation geometry."""
from __future__ import annotations

import json
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import xacro

from cleany_navigation.urdf_geometry import rpy


def _pose(transform: np.ndarray, parent: str) -> ET.Element:
    pose = ET.Element('pose', relative_to=parent)
    pose.text = ' '.join(map(str, [*transform[:3, 3], *rpy(transform[:3, :3])]))
    return pose


def _replace_legacy_visuals(model: ET.Element, description: Path) -> None:
    """Reuse canonical visuals; legacy navigation retains its collision envelope."""
    urdf = ET.fromstring(xacro.process_file(
        str(description / 'urdf/cleany.urdf.xacro')).toxml())
    base = model.find("link[@name='base_link']")
    for side in ('left', 'right'):
        for visual in base.findall('visual'):
            if visual.get('name', '').startswith(f'{side}_wrist_camera_link_'):
                base.remove(visual)
        jaw = urdf.find(f"link[@name='{side}_gripper_frame']")
        for source in jaw.findall('visual'):
            if not source.get('name', '').startswith(f'{side}_wrist_mount_v8_'):
                continue
            visual = ET.SubElement(base, 'visual', name=source.get('name'))
            origin = source.find('origin')
            ET.SubElement(visual, 'pose', relative_to=f'{side}_fixed_jaw').text = (
                origin.get('xyz') + ' ' + origin.get('rpy'))
            geometry = ET.SubElement(visual, 'geometry')
            shape = source.find('geometry')[0]
            converted = ET.SubElement(geometry, shape.tag)
            if shape.tag == 'mesh':
                path = description / shape.get('filename').removeprefix(
                    'package://cleany_description/')
                ET.SubElement(converted, 'uri').text = str(path.resolve())
                ET.SubElement(converted, 'scale').text = shape.get('scale', '1 1 1')
            elif shape.tag == 'cylinder':
                for key in ('radius', 'length'):
                    ET.SubElement(converted, key).text = shape.get(key)
            else:
                raise ValueError(f'Unsupported wrist visual shape: {shape.tag}')
            material = ET.SubElement(visual, 'material')
            color = source.find('material/color').get('rgba')
            for key in ('ambient', 'diffuse'):
                ET.SubElement(material, key).text = color


def apply_wrist_cameras(
    model: ET.Element, description: Path, *, legacy_visuals: bool = False,
) -> None:
    """Align present sensors and optical frames; do not add excluded CAD arms."""
    registration = json.loads(
        (description / 'config/geometry/wrist_mount_v8.json').read_text())
    jaw_from_cv = np.asarray(registration['legacy_jaw_from_camera_cv_approximate'])
    jaw_from_geometry = np.asarray(registration['legacy_jaw_from_camera_geometry_body'])
    # Gazebo +X forward / +Y left / +Z up, optical +Z forward / +X right / +Y down.
    cv_from_sensor = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
    jaw_from_sensor = jaw_from_cv.copy()
    jaw_from_sensor[:3, :3] = jaw_from_cv[:3, :3] @ cv_from_sensor
    names = {element.get('name') for element in model if element.tag in ('link', 'frame')}
    for side in ('left', 'right'):
        sensor = model.find(f".//sensor[@name='{side}_wrist_rgb']")
        if sensor is None:
            continue
        jaw = f'{side}_fixed_jaw' if legacy_visuals else f'{side}_gripper_frame'
        if jaw not in names:
            raise ValueError(f'Wrist sensor has no jaw frame: {jaw}')
        mount = model.find(f"frame[@name='{side}_wrist_camera_mount']")
        mount.remove(mount.find('pose'))
        mount.append(_pose(jaw_from_geometry, jaw))
        sensor.remove(sensor.find('pose'))
        sensor.insert(0, _pose(jaw_from_sensor, jaw))
        optical = ET.SubElement(model, 'frame',
                                name=sensor.findtext('gz_frame_id'), attached_to=jaw)
        optical.append(_pose(jaw_from_cv, jaw))
    if legacy_visuals:
        _replace_legacy_visuals(model, description)
