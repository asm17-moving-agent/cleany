"""Bind verified CAD geometry to the established motor-coordinate interfaces."""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from cleany_description.cad_coordinates import ATTRIBUTE, CadCoordinates


def require_cad_motion_ready(profile_file: Path, repository_root: Path) -> None:
    """Keep a failed full-opening migration out of the actuator startup path."""
    config = json.loads(profile_file.read_text())
    if config.get('model_variant') != 'freecad_calibrated':
        return
    binding_file = Path(config['cad_binding_file']).expanduser()
    if not binding_file.is_absolute():
        binding_file = repository_root/binding_file
    binding = json.loads(binding_file.read_text())
    validation = binding.get('motion_validation', {})
    if validation.get('ready') is not True:
        raise ValueError('Calibrated CAD physical startup is blocked: '+
                         validation.get('reason', 'full-opening collision verification is incomplete'))


def model_from_binding(source, binding, pose_config, bounds):
    """Reject stale calibration before binding the byte-verified CAD snapshot."""
    if binding.get('schema') != 'cleany.cad_grasp_binding/v1':
        raise ValueError('Unknown calibrated CAD binding schema')
    for path, expected in ((source, binding['source_model_sha256']),
                           (pose_config, binding['pose_config_sha256'])):
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('Calibrated CAD dependency changed: '+str(path))
    pose = json.loads(pose_config.read_text())
    if pose['joint_coordinates'] != binding['joint_coordinates']:
        raise ValueError('Camera and CAD actuator coordinates differ')
    dynamics = binding['joint_dynamics']
    if set(bounds) != set(dynamics):
        raise ValueError('Calibrated CAD requires all fourteen current motor bounds')
    limits = {name: dict(values, lower=bounds[name][0], upper=bounds[name][1])
              for name, values in dynamics.items()}
    return source.read_text(), limits


def rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis/np.linalg.norm(axis)
    x, y, z = axis
    k = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3)+math.sin(angle)*k+(1-math.cos(angle))*(k@k)


def origin_matrix(element: ET.Element | None) -> np.ndarray:
    matrix = np.eye(4)
    if element is not None:
        matrix[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        values = np.fromstring(element.get('rpy', '0 0 0'), sep=' ')
        for axis, angle in reversed(list(zip(np.eye(3), values))):
            matrix[:3, :3] = matrix[:3, :3]@rotation(axis, angle)
    return matrix


def write_origin(element: ET.Element, matrix: np.ndarray) -> None:
    r = matrix[:3, :3]
    pitch = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    if abs(math.cos(pitch)) > 1e-10:
        roll, yaw = math.atan2(r[2, 1], r[2, 2]), math.atan2(r[1, 0], r[0, 0])
    else:
        roll, yaw = 0., math.atan2(-r[0, 1], r[1, 1])
    node = element.find('origin')
    if node is None:
        node = ET.SubElement(element, 'origin')
    node.set('xyz', ' '.join(format(v, '.17g') for v in matrix[:3, 3]))
    node.set('rpy', ' '.join(format(v, '.17g') for v in (roll, pitch, yaw)))


def fixed_alias(root: ET.Element, parent: str, child: str, matrix: np.ndarray) -> None:
    if root.find(f"link[@name='{child}']") is not None:
        raise ValueError('CAD compatibility frame already exists: '+child)
    ET.SubElement(root, 'link', name=child)
    joint = ET.SubElement(root, 'joint', name=child+'_cad_alias', type='fixed')
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=child)
    write_origin(joint, matrix)


def calibrated_cad_model(cad_xml: str, coordinates: dict, limits: dict,
                         linkage_config: dict, tcp_distance_m: float) -> tuple[str, dict]:
    """Preserve CAD surfaces, nonlinear fingers and all physical command radians.

    Coaxial URDF mimic joints encode non-unit affine scales exactly. Axis-frame
    adapters keep the CUDA parser's axis-aligned requirement without rounding
    the CAD axes. Passive finger states are supplied by the shared pin solver.
    """
    closure = CadCoordinates(linkage_config)
    root = ET.fromstring(cad_xml)
    root.set('name', 'cleany')
    root.set(ATTRIBUTE, json.dumps(linkage_config, separators=(',', ':')))
    visual_count, collision_count = len(root.findall('.//visual')), len(root.findall('.//collision'))
    bindings = deepcopy(coordinates)
    for group in closure.config['grippers']:
        endpoints = group.get('endpoint_mapping')
        if endpoints:
            bound = limits[group['source_joint']]
            if (not math.isclose(bound['upper'], endpoints['motor_open_rad'], abs_tol=1e-10)
                    or not math.isclose(bound['lower'], endpoints['motor_closed_rad'], abs_tol=1e-10)):
                raise ValueError('CAD endpoints do not match the current gripper actuator limits')
        bindings[group['linkage']['cam_joint']] = {k:group[k] for k in ('source_joint', 'scale', 'offset_rad')}
    expected = {f'{side}_{suffix}_joint' for side in ('left', 'right') for suffix in
                ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll', 'gripper')}
    expected.update(('head_pan_joint', 'head_tilt_joint'))
    if {b['source_joint'] for b in bindings.values()} != expected:
        raise ValueError('CAD bindings must cover fourteen established actuator coordinates')
    remaining = set(bindings)
    mimic_names = []
    for joint in list(root.findall('joint')):
        old_name = joint.get('name')
        if joint.get('type') == 'fixed':
            continue
        if old_name in closure.dependent_names:
            # These are solved, unactuated coordinates, not invented physical limits.
            joint.set('type', 'continuous')
            if joint.find('limit') is None:
                ET.SubElement(joint, 'limit', effort='0', velocity='0')
            continue
        if old_name not in bindings:
            if 'wheel' not in old_name:
                raise ValueError('Unbound moving CAD joint: '+old_name)
            joint.set('type', 'fixed')
            for tag in ('axis', 'limit', 'dynamics'):
                element = joint.find(tag)
                if element is not None:
                    joint.remove(element)
            continue
        remaining.remove(old_name)
        cfg = bindings[old_name]
        public = cfg['source_joint']
        scale, offset = float(cfg['scale']), float(cfg['offset_rad'])
        if not math.isfinite(scale+offset) or scale == 0:
            raise ValueError('Invalid calibrated CAD affine mapping: '+old_name)
        bound = limits[public]
        if (not all(math.isfinite(float(bound[k])) for k in ('lower', 'upper', 'effort', 'velocity'))
                or bound['lower'] >= bound['upper'] or bound['velocity'] <= 0):
            raise ValueError('Missing operational joint limits: '+public)
        axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
        if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-12:
            raise ValueError('Invalid CAD joint axis')
        axis /= np.linalg.norm(axis)
        reference = np.eye(3)[int(np.argmin(np.abs(axis)))]
        x = np.cross(reference, axis)
        x /= np.linalg.norm(x)
        frame = np.eye(4)
        frame[:3, :3] = np.column_stack((x, np.cross(axis, x), axis))
        offset_matrix = np.eye(4)
        offset_matrix[:3, :3] = rotation(np.array([0., 0., 1.]), offset)
        original_child = joint.find('child').get('link')
        motor_frame = public+'_cad_motor_frame'
        ET.SubElement(root, 'link', name=motor_frame)
        matrix = origin_matrix(joint.find('origin'))@frame@offset_matrix
        joint.set('name', public)
        joint.set('type', 'revolute')
        joint.find('child').set('link', motor_frame)
        sign = int(math.copysign(1., scale))
        joint.find('axis').set('xyz', f'0 0 {sign}')
        write_origin(joint, matrix)
        old_limit = joint.find('limit')
        if old_limit is not None:
            joint.remove(old_limit)
        ET.SubElement(joint, 'limit', **{k:str(v) for k,v in bound.items()})
        multiplier = abs(scale)-1
        final_parent = motor_frame
        if abs(multiplier) > 1e-14:
            final_parent = public+'_cad_scaled_frame'
            ET.SubElement(root, 'link', name=final_parent)
            name = public+'_cad_scale'
            mimic = ET.SubElement(root, 'joint', name=name, type='revolute')
            ET.SubElement(mimic, 'parent', link=motor_frame)
            ET.SubElement(mimic, 'child', link=final_parent)
            ET.SubElement(mimic, 'axis', xyz=f'0 0 {sign}')
            extent = abs(multiplier)*max(abs(bound['lower']), abs(bound['upper']))+1e-12
            ET.SubElement(mimic, 'limit', lower=str(-extent), upper=str(extent),
                          effort=str(bound['effort']), velocity=str(bound['velocity']))
            ET.SubElement(mimic, 'mimic', joint=public, multiplier=str(multiplier), offset='0')
            mimic_names.append(name)
        restoring = ET.SubElement(root, 'joint', name=public+'_cad_body', type='fixed')
        ET.SubElement(restoring, 'parent', link=final_parent)
        ET.SubElement(restoring, 'child', link=original_child)
        write_origin(restoring, np.linalg.inv(frame))
    if remaining:
        raise ValueError('Selected CAD does not contain calibrated joints: '+str(sorted(remaining)))
    if not math.isfinite(tcp_distance_m) or tcp_distance_m <= 0:
        raise ValueError('Approved CAD TCP distance is required')
    for side in ('left', 'right'):
        # The grasp pipeline convention remains local -Y approach, +X closing.
        frame = np.eye(4)
        if side == 'left':
            frame[:3, :3] = rotation(np.array([0., 0., 1.]), math.pi)
        fixed_alias(root, f'cad_{side}_wrist_roll', side+'_gripper_frame', frame)
        tcp = np.eye(4)
        tcp[1, 3] = -tcp_distance_m
        fixed_alias(root, side+'_gripper_frame', side+'_grasp_tcp', tcp)
        aim = np.eye(4)
        aim[1, 3] = -.140
        fixed_alias(root, side+'_grasp_tcp', side+'_pregrasp_aim_tip', aim)
        for old, public in [('shoulder_yaw','shoulder_yaw_link'),('shoulder_pitch','upper_arm_link'),
                            ('elbow_pitch','lower_arm_link'),('wrist_pitch','wrist_pitch_link')]:
            fixed_alias(root, f'cad_{side}_{old}', f'{side}_{public}', np.eye(4))
        fixed_alias(root, f'cad_{side}_gripper_cam', side+'_moving_jaw_link', np.eye(4))
    fixed_alias(root, 'cad_head_pan', 'head_pan_link', np.eye(4))
    fixed_alias(root, 'cad_head_tilt', 'head_tilt_link', np.eye(4))
    fixed_alias(root, 'base_link', 'top_base_link', np.eye(4))
    assert len(root.findall('.//visual')) == visual_count
    assert len(root.findall('.//collision')) == collision_count
    return ET.tostring(root, encoding='unicode'), {
        'model_variant': 'freecad_calibrated', 'actuator_coordinates_changed': False,
        'mimic_joints': mimic_names, 'dependent_joints': list(closure.dependent_names),
        'visual_shapes_retained': visual_count, 'collision_shapes_retained': collision_count,
        'new_collision_envelopes_created': False, 'absolute_accuracy_verified': False,
    }
