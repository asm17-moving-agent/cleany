"""Read-only serial-chain FK from the runtime URDF, not simulation geometry."""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET
import numpy as np


def _unit_axis_rotation(axis, angle):
    x, y, z = axis
    sine, cosine = math.sin(angle), math.cos(angle)
    complement = 1.0 - cosine
    return np.array((
        (cosine + x*x*complement, x*y*complement - z*sine, x*z*complement + y*sine),
        (y*x*complement + z*sine, cosine + y*y*complement, y*z*complement - x*sine),
        (z*x*complement - y*sine, z*y*complement + x*sine, cosine + z*z*complement),
    ))


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    unit = tuple(map(float, axis / np.linalg.norm(axis)))
    return _unit_axis_rotation(unit, angle)


@dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    translation: np.ndarray
    rotation: np.ndarray
    axis: np.ndarray
    mimic_joint: str | None = None
    multiplier: float = 1.0
    offset: float = 0.0


class UrdfChain:
    def __init__(self, description: str, root: str, tip: str):
        joints = ET.fromstring(description).findall('joint')
        parents = {j.find('child').get('link'):j for j in joints}
        chain = []
        seen = set()
        while tip != root:
            if tip in seen or tip not in parents:
                raise ValueError('URDF chain does not connect root to tip')
            seen.add(tip)
            joint = parents[tip]
            kind = joint.get('type')
            if kind not in ('fixed','revolute','continuous','prismatic'):
                raise ValueError('Unsupported URDF joint in serial FK chain')
            mimic = joint.find('mimic')
            source = mimic.get('joint') if mimic is not None else None
            multiplier = float(mimic.get('multiplier', '1')) if mimic is not None else 1.
            offset = float(mimic.get('offset', '0')) if mimic is not None else 0.
            if not math.isfinite(multiplier+offset) or (mimic is not None and not source):
                raise ValueError('Invalid URDF mimic coordinate')
            origin = joint.find('origin')
            xyz = np.fromstring(origin.get('xyz','0 0 0') if origin is not None else '0 0 0',sep=' ')
            rpy = np.fromstring(origin.get('rpy','0 0 0') if origin is not None else '0 0 0',sep=' ')
            axis_node = joint.find('axis')
            axis = np.fromstring(axis_node.get('xyz','1 0 0') if axis_node is not None else '1 0 0',sep=' ')
            if any(v.shape != (3,) or not np.isfinite(v).all() for v in (xyz,rpy,axis)) or np.linalg.norm(axis) < 1e-12:
                raise ValueError('Invalid URDF transform or axis')
            rotation = axis_rotation((0,0,1),rpy[2])@axis_rotation((0,1,0),rpy[1])@axis_rotation((1,0,0),rpy[0])
            chain.append(Joint(joint.get('name'),kind,xyz,rotation,axis/np.linalg.norm(axis),
                               source,multiplier,offset))
            tip = joint.find('parent').get('link')
        self.joints = tuple(reversed(chain))
        self._coordinates = {}
        for joint in self.joints:
            if joint.kind == 'fixed':
                continue
            source, multiplier, offset = joint.name, 1., 0.
            if joint.mimic_joint is not None:
                if joint.mimic_joint not in self._coordinates:
                    raise ValueError('URDF mimic must reference an earlier joint in the chain')
                source, factor, bias = self._coordinates[joint.mimic_joint]
                multiplier, offset = factor*joint.multiplier, bias*joint.multiplier+joint.offset
            self._coordinates[joint.name] = source,multiplier,offset
        # URDF axes stay constant across poses of this chain.
        self._joint_axes = {
            joint.name: tuple(map(float, joint.axis / np.linalg.norm(joint.axis)))
            for joint in self.joints if joint.kind in ('revolute', 'continuous')}

    def pose(self, positions):
        p, r = np.zeros(3), np.eye(3)
        for joint in self.joints:
            p = p+r@joint.translation
            r = r@joint.rotation
            if joint.kind != 'fixed':
                source, multiplier, offset = self._coordinates[joint.name]
                value = float(positions[source])*multiplier+offset
                if not math.isfinite(value):
                    raise ValueError('Nonfinite joint position')
                if joint.kind == 'prismatic':
                    p = p+r@joint.axis*value
                else:
                    r = r@_unit_axis_rotation(self._joint_axes[joint.name], value)
        return p,r
