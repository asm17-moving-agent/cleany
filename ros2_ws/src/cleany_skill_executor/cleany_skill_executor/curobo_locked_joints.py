"""Update verified locked-joint transforms without rebuilding a cuRobo robot.

Only the pinned URDF-based, non-mimic layout is eligible. Buffer addresses and
collision geometry remain fixed for captured CUDA graphs.
"""
import xml.etree.ElementTree as ET

import numpy as np

from .core.urdf_fk import UrdfChain


class LockedJointUpdater:
    def __init__(self, xml: str, kinematics, previous: dict[str, float]) -> None:
        import torch
        root = ET.fromstring(xml)
        if any(j.find('mimic') is not None for j in root.findall('joint')):
            raise ValueError('Mimic joints require full cuRobo configuration update')
        joints = {joint.get('name'): joint for joint in root.findall('joint')}
        self.config = kinematics.kinematics_config
        self.names = tuple(previous)
        if (self.config.lock_jointstate is None
                or set(self.config.lock_jointstate.joint_names) != set(self.names)):
            raise ValueError('cuRobo locked-joint layout differs from runtime state')
        self.chains, indices = [], []
        for name in self.names:
            joint = joints[name]
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            self.chains.append(UrdfChain(
                '<robot>' + ET.tostring(joint, encoding='unicode') + '</robot>', parent, child))
            indices.append(self.config.link_name_to_idx_map[child])
        self.indices = self.config.joint_map.new_tensor(indices, dtype=torch.long)
        if not bool((self.config.joint_map_type[self.indices] == -1).all()):
            raise ValueError('cuRobo lock transforms are not fixed joints')
        actual = self.config.fixed_transforms[self.indices].detach().cpu().numpy()
        if not np.allclose(actual, self._matrices(previous), atol=2e-5, rtol=0):
            raise ValueError('cuRobo lock transforms differ from URDF FK')

    def _matrices(self, values: dict[str, float]) -> np.ndarray:
        if set(values) != set(self.names):
            raise ValueError('Locked joint ownership changed')
        matrices = np.repeat(np.eye(4)[None, ...], len(self.names), axis=0)
        for matrix, chain in zip(matrices, self.chains, strict=True):
            position, rotation = chain.pose(values)
            matrix[:3, :3], matrix[:3, 3] = rotation, position
        return matrices

    def update(self, values: dict[str, float], state: dict[str, float]) -> None:
        matrices = self._matrices(values)
        config = self.config
        locked = [values[name] for name in config.lock_jointstate.joint_names]
        retract = [state[name] for name in config.joint_names]
        if not np.isfinite(locked + retract).all():
            raise ValueError('Nonfinite cuRobo joint state')
        config.fixed_transforms.index_copy_(0, self.indices,
            config.fixed_transforms.new_tensor(matrices))
        config.lock_jointstate.position.copy_(
            config.lock_jointstate.position.new_tensor(locked).reshape_as(config.lock_jointstate.position))
        config.cspace.retract_config.copy_(
            config.cspace.retract_config.new_tensor(retract).reshape_as(config.cspace.retract_config))
