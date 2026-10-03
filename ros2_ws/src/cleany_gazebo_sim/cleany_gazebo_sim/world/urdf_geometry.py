"""Shared URDF transforms and optional fixed-joint conversion."""
from __future__ import annotations
from copy import deepcopy
from xml.etree import ElementTree as ET
import numpy as np

def rotation(rpy: list[float]) -> np.ndarray:
    r, p, y = rpy
    cr, cp, cy = np.cos(rpy)
    sr, sp, sy = np.sin(rpy)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def rpy(matrix: np.ndarray) -> list[float]:
    return [float(np.arctan2(matrix[2, 1], matrix[2, 2])),
            float(np.arctan2(-matrix[2, 0], np.hypot(matrix[0, 0], matrix[1, 0]))),
            float(np.arctan2(matrix[1, 0], matrix[0, 0]))]


def origin(element: ET.Element | None) -> np.ndarray:
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
        result[:3, :3] = rotation(list(map(float, element.get('rpy', '0 0 0').split())))
    return result


def freeze(urdf: ET.Element, joints: dict[str, float],
           simulation_limits: dict[str, list[float]] | None = None,
           keep_pan: bool = False) -> dict[str, np.ndarray]:
    """Bake joint positions before sdformat merges fixed-body mass and geometry."""
    simulation_limits = simulation_limits or {}
    if not set(simulation_limits) <= set(joints):
        raise ValueError("Simulation limit override must name a parked joint")
    for bounds in simulation_limits.values():
        if len(bounds) != 2 or not np.isfinite(bounds).all() or bounds[0] > bounds[1]:
            raise ValueError("Invalid simulation joint limits")
    edges = []
    used = set()
    for joint in urdf.findall('joint'):
        name = joint.get('name')
        pose = origin(joint.find('origin'))
        if name in joints:
            angle = joints[name]
            limit = joint.find('limit')
            bounds = simulation_limits.get(name, [float(limit.get('lower', '-inf')), float(limit.get('upper', 'inf'))] if limit is not None else [-np.inf, np.inf])
            if not np.isfinite(angle) or not bounds[0] <= angle <= bounds[1]:
                raise ValueError(f'Park angle outside limits: {name}')
            axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            axis /= np.linalg.norm(axis)
            x, y, z = axis
            cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
            pose[:3, :3] = pose[:3, :3] @ (np.eye(3)+np.sin(angle)*cross+(1-np.cos(angle))*(cross@cross))
            if keep_pan and name == 'head_pan_joint':
                if angle != 0.0:
                    raise ValueError('Dynamic pan must start at zero')
                used.add(name)
                edges.append((joint.find('parent').get('link'), joint.find('child').get('link'), pose))
                continue
            joint.find('origin').set('rpy', ' '.join(map(str, rpy(pose[:3, :3]))))
            joint.set('type', 'fixed')
            for tag in ('axis', 'limit', 'dynamics'):
                for item in joint.findall(tag):
                    joint.remove(item)
            used.add(name)
        elif joint.get('type') != 'fixed' and not name.endswith('_wheel_joint'):
            raise ValueError(f'Unconfigured movable upper-body joint: {name}')
        edges.append((joint.find('parent').get('link'), joint.find('child').get('link'), pose))
    if used != set(joints):
        raise ValueError('Park profile contains unknown joints')
    transforms = {'base_link': np.eye(4)}
    while edges:
        ready = [e for e in edges if e[0] in transforms]
        if not ready:
            raise ValueError('Disconnected URDF tree')
        for parent, child, pose in ready:
            transforms[child] = transforms[parent] @ pose
            edges.remove((parent, child, pose))
    return transforms



def forward_kinematics(urdf: ET.Element, joints: dict[str, float],
                       limit_overrides: dict[str, list[float]] | None = None) -> dict[str, np.ndarray]:
    return freeze(deepcopy(urdf), joints, limit_overrides)
