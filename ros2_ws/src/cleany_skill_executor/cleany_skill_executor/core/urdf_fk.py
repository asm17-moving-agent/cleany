"""Read-only serial-chain FK from the runtime URDF, not simulation geometry."""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET
import numpy as np


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x,y,z = axis
    skew = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*skew+(1-math.cos(angle))*(skew@skew)


@dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    translation: np.ndarray
    rotation: np.ndarray
    axis: np.ndarray


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
            if kind not in ('fixed','revolute','continuous','prismatic') or joint.find('mimic') is not None:
                raise ValueError('Unsupported URDF joint in serial FK chain')
            origin = joint.find('origin')
            xyz = np.fromstring(origin.get('xyz','0 0 0') if origin is not None else '0 0 0',sep=' ')
            rpy = np.fromstring(origin.get('rpy','0 0 0') if origin is not None else '0 0 0',sep=' ')
            axis_node = joint.find('axis')
            axis = np.fromstring(axis_node.get('xyz','1 0 0') if axis_node is not None else '1 0 0',sep=' ')
            if any(v.shape != (3,) or not np.isfinite(v).all() for v in (xyz,rpy,axis)) or np.linalg.norm(axis) < 1e-12:
                raise ValueError('Invalid URDF transform or axis')
            rotation = axis_rotation((0,0,1),rpy[2])@axis_rotation((0,1,0),rpy[1])@axis_rotation((1,0,0),rpy[0])
            chain.append(Joint(joint.get('name'),kind,xyz,rotation,axis/np.linalg.norm(axis)))
            tip = joint.find('parent').get('link')
        self.joints = tuple(reversed(chain))

    def pose(self, positions):
        p, r = np.zeros(3), np.eye(3)
        for joint in self.joints:
            p = p+r@joint.translation
            r = r@joint.rotation
            if joint.kind != 'fixed':
                value = float(positions[joint.name])
                if not math.isfinite(value):
                    raise ValueError('Nonfinite joint position')
                if joint.kind == 'prismatic':
                    p = p+r@joint.axis*value
                else:
                    r = r@axis_rotation(joint.axis,value)
        return p,r
