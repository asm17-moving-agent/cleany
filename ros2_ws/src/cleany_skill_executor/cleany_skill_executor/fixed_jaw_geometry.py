"""Load fixed-finger collision geometry relative to the runtime URDF TCP."""
from __future__ import annotations

from pathlib import Path
import struct
from typing import Callable, Sequence
import xml.etree.ElementTree as ET

import numpy as np

from .core.fixed_jaw_clearance import Bounds, TargetBox
from .core.physical_grasp_selection import PoseValue
from .core.urdf_fk import UrdfChain, axis_rotation


def resolve_mesh(uri: str) -> Path:
    if uri.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory
        package, relative = uri[len('package://'):].split('/', 1)
        return Path(get_package_share_directory(package)) / relative
    return Path(uri.removeprefix('file://'))


def stl_vertices(path: Path) -> np.ndarray:
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0] if len(data) >= 84 else 0
    if count and len(data) == 84 + 50 * count:
        vertices = [struct.unpack_from('<3f', data, 84 + 50 * triangle + 12 + 12 * vertex)
                    for triangle in range(count) for vertex in range(3)]
    else:
        vertices = [tuple(float(v) for v in line.split()[1:])
                    for line in data.decode('ascii').splitlines()
                    if line.strip().startswith('vertex ')]
    values = np.asarray(vertices, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3 or not len(values) or not np.isfinite(values).all():
        raise ValueError(f'Invalid fixed-jaw STL: {path}')
    return values


def load_fixed_jaw_bounds(description: str, arm: str, mesh_names: Sequence[str],
                          resolver: Callable[[str], Path] = resolve_mesh) -> Bounds:
    if not mesh_names or any(not name or Path(name).name != name for name in mesh_names):
        raise ValueError('Fixed-jaw collision mesh names must be nonempty basenames')
    chain = UrdfChain(description, f'{arm}_gripper_frame', f'{arm}_grasp_tcp')
    if any(joint.kind != 'fixed' for joint in chain.joints):
        raise ValueError('Fixed-jaw TCP transform must contain only fixed joints')
    tcp_position, tcp_rotation = chain.pose({})
    link = ET.fromstring(description).find(f"link[@name='{arm}_gripper_frame']")
    if link is None:
        raise ValueError('Runtime URDF is missing fixed-jaw link')
    found, vertices = set(), []
    for collision in link.findall('collision'):
        mesh = collision.find('geometry/mesh')
        if mesh is None or Path(mesh.get('filename', '')).name not in mesh_names:
            continue
        name = Path(mesh.get('filename')).name
        found.add(name)
        scale = np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
        origin = collision.find('origin')
        xyz = np.fromstring(origin.get('xyz', '0 0 0') if origin is not None else '0 0 0', sep=' ')
        rpy = np.fromstring(origin.get('rpy', '0 0 0') if origin is not None else '0 0 0', sep=' ')
        if (any(v.shape != (3,) or not np.isfinite(v).all() for v in (scale, xyz, rpy))
                or np.any(scale <= 0)):
            raise ValueError('Invalid fixed-jaw mesh transform')
        rotation = (axis_rotation((0, 0, 1), rpy[2]) @ axis_rotation((0, 1, 0), rpy[1])
                    @ axis_rotation((1, 0, 0), rpy[0]))
        in_link = stl_vertices(resolver(mesh.get('filename'))) * scale @ rotation.T + xyz
        vertices.append((in_link - tcp_position) @ tcp_rotation)
    if found != set(mesh_names):
        raise ValueError(f'Runtime URDF is missing fixed-jaw collision meshes: {set(mesh_names) - found}')
    values = np.concatenate(vertices)
    return Bounds(tuple(values.min(axis=0)), tuple(values.max(axis=0)))


def target_box(message) -> TargetBox:
    pose, size = message.obb_pose, message.obb_size
    return TargetBox(PoseValue(
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)),
        (size.x, size.y, size.z))
