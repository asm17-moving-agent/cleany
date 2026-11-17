"""Register rigid accessory geometry through an already calibrated camera frame."""
from copy import deepcopy
import hashlib
import math
import xml.etree.ElementTree as ET

import numpy as np


def _transform(origin: ET.Element | None) -> np.ndarray:
    matrix = np.eye(4)
    if origin is None:
        return matrix
    xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
    if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite([xyz, rpy]).all():
        raise ValueError('Invalid accessory geometry origin')
    cr, cp, cy = np.cos(rpy)
    sr, sp, sy = np.sin(rpy)
    matrix[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                       [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                       [-sp, cp*sr, cp*cr]]
    matrix[:3, 3] = xyz
    return matrix


def _origin(matrix: np.ndarray) -> dict[str, str]:
    rotation = matrix[:3, :3]
    pitch = math.atan2(-rotation[2, 0], math.hypot(rotation[0, 0], rotation[1, 0]))
    if abs(math.cos(pitch)) > 1e-10:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.
    return {'xyz': ' '.join(format(v, '.17g') for v in matrix[:3, 3]),
            'rpy': ' '.join(format(v, '.17g') for v in (roll, pitch, yaw))}


def _rigid_transform(value, name: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-10)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-8)
            or not math.isclose(np.linalg.det(matrix[:3, :3]), 1., abs_tol=1e-8)):
        raise ValueError(name + ' must be a finite rigid transform')
    return matrix


def attach_rigid_accessories(model: str, source: str, registration: dict) -> str:
    """Copy only named rigid parts into their physical parent, preserving collision checks.

    Use either a fixed camera joint or an explicit CAD registration guarded by
    the target parent-joint contract. No joint, camera extrinsic, TCP, padding
    or collision permission changes.
    """
    if hashlib.sha256(source.encode()).hexdigest() != registration['source_model_sha256']:
        raise ValueError('Rigid accessory source model hash mismatch')
    original, cad = ET.fromstring(model), ET.fromstring(source)
    parent_name = registration['target_link']
    parent = original.find(f"link[@name='{parent_name}']")
    if parent is None:
        raise ValueError('Rigid accessory target link missing')
    cad_link = cad.find(f"link[@name='{registration['source_link']}']")
    if cad_link is None:
        raise ValueError('Rigid accessory source link missing')
    if 'target_parent_from_source' in registration:
        contract = registration['target_parent_joint']
        joint = original.find(f"joint[@name='{contract['name']}']")
        expected_origin = ET.Element('origin',
            xyz=' '.join(map(str, contract['origin_xyz_m'])),
            rpy=' '.join(map(str, contract['origin_rpy_rad'])))
        if (joint is None or joint.find('child').get('link') != parent_name
                or joint.find('parent').get('link') != contract['parent_link']
                or joint.find('axis') is None
                or not np.allclose(_transform(joint.find('origin')), _transform(expected_origin),
                                   rtol=0., atol=1e-10)
                or not np.allclose(np.fromstring(joint.find('axis').get('xyz'), sep=' '),
                                   contract['axis'], rtol=0., atol=1e-10)):
            raise ValueError('Rigid accessory target parent-joint contract mismatch')
        transform = _rigid_transform(registration['target_parent_from_source'],
                                     'Rigid accessory CAD registration')
    else:
        camera = original.find(f"joint[@name='{registration['target_camera_joint']}']")
        if (camera is None or camera.get('type') != 'fixed'
                or camera.find('parent').get('link') != parent_name):
            raise ValueError('Rigid accessory target camera must be fixed to its parent link')
        source_from_camera = _rigid_transform(registration['source_parent_from_camera'],
                                              'Rigid accessory camera registration')
        transform = _transform(camera.find('origin')) @ np.linalg.inv(source_from_camera)
    parts = registration['parts']
    if (not isinstance(parts, list) or not parts or len(set(parts)) != len(parts)
            or any(not isinstance(name, str) or not name for name in parts)):
        raise ValueError('Rigid accessory parts must be explicit unique names')
    additions = []
    for name in parts:
        for kind in ('visual', 'collision'):
            shape = cad_link.find(f"{kind}[@name='{name}_{kind}']")
            if shape is None or shape.find('geometry') is None:
                raise ValueError(f'Rigid accessory part missing: {name}/{kind}')
            item = deepcopy(shape)
            item.set('name', 'registered_' + item.get('name'))
            origin = item.find('origin')
            pose = transform @ _transform(origin)
            if origin is None:
                origin = ET.SubElement(item, 'origin')
            origin.attrib.update(_origin(pose))
            additions.append(item)
    replacements = registration.get('replace_shape_prefixes', [])
    if not isinstance(replacements, list) or any(
            not isinstance(prefix, str) or not prefix for prefix in replacements):
        raise ValueError('Accessory replacement prefixes must not be empty')
    for item in list(parent):
        if item.tag in ('visual', 'collision') and any(
                item.get('name', '').startswith(prefix) for prefix in replacements):
            parent.remove(item)
    names = {item.get('name') for item in parent}
    if any(item.get('name') in names for item in additions):
        raise ValueError('Rigid accessory geometry is already registered')
    parent.extend(additions)
    return ET.tostring(original, encoding='unicode')
