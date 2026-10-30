"""Conservative mast cuboids in the planning base frame, derived from the URDF."""
from __future__ import annotations

import xml.etree.ElementTree as ET

import numpy as np

from .collision_sphere_model import mesh_path
from .core.urdf_fk import UrdfChain

MAST_LINK = 'top_base_link'


def _compact_boxes(boxes, tolerance):
    """Remove contained boxes and join overlapping sections with similar widths."""
    boxes = list(boxes)
    changed = True
    while changed:
        changed = False
        for i, (low, high) in enumerate(boxes):
            for j in range(i):
                other_low, other_high = boxes[j]
                contained = ((low >= other_low).all() and (high <= other_high).all()
                             or (other_low >= low).all() and (other_high <= high).all())
                merge = contained
                for axis in range(3):
                    cross = [k for k in range(3) if k != axis]
                    similar = (np.abs(low[cross]-other_low[cross]) <= tolerance).all()
                    similar &= (np.abs(high[cross]-other_high[cross]) <= tolerance).all()
                    overlap = max(low[axis], other_low[axis]) <= min(high[axis], other_high[axis])
                    merge |= similar and overlap
                if merge:
                    boxes[j] = (np.minimum(low, other_low), np.maximum(high, other_high))
                    boxes.pop(i)
                    changed = True
                    break
            if changed:
                break
    return boxes


def mast_cuboids(urdf: str, padding: float, *, resolve=mesh_path) -> dict:
    """Enclose every mast triangle; reject a mast that can move relative to base.

    Small components already inside another box need no additional obstacle.
    Adjacent sections may grow by at most 1 mm per transverse axis per merge.
    Full-robot acceptance still uses the original collision geometry and ACM.
    """
    import trimesh

    link = ET.fromstring(urdf).find(f"link[@name='{MAST_LINK}']")
    if link is None or not link.findall('collision'):
        return {}
    chain = UrdfChain(urdf, 'base_link', MAST_LINK)
    if any(joint.kind != 'fixed' for joint in chain.joints):
        raise ValueError('Mast cuboids require a fixed chain to base_link')
    base_position, base_rotation = chain.pose({})
    transform = np.eye(4)
    transform[:3, :3] = base_rotation
    quaternion = trimesh.transformations.quaternion_from_matrix(transform).tolist()
    boxes = []
    for collision in link.findall('collision'):
        origin = collision.find('origin')
        xyz = np.fromstring(origin.get('xyz', '0 0 0') if origin is not None else '0 0 0', sep=' ')
        rpy = np.fromstring(origin.get('rpy', '0 0 0') if origin is not None else '0 0 0', sep=' ')
        if any(v.shape != (3,) or not np.isfinite(v).all() for v in (xyz, rpy)):
            raise ValueError('Invalid mast collision origin')
        rotation = trimesh.transformations.euler_matrix(*rpy)[:3, :3]
        mesh, box = collision.find('geometry/mesh'), collision.find('geometry/box')
        if mesh is not None:
            geometry = trimesh.load_mesh(resolve(mesh.get('filename')), process=True)
            scale = np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            if scale.shape != (3,) or not np.isfinite(scale).all() or (scale <= 0).any():
                raise ValueError('Invalid mast mesh scale')
            geometry.apply_scale(scale)
            components = geometry.split(only_watertight=False)
            if sum(len(part.faces) for part in components) != len(geometry.faces):
                raise ValueError('Mast mesh partition lost triangles')
        elif box is not None:
            dims = np.fromstring(box.get('size'), sep=' ')
            if dims.shape != (3,) or not np.isfinite(dims).all() or (dims <= 0).any():
                raise ValueError('Invalid mast box dimensions')
            components = [trimesh.creation.box(extents=dims)]
        else:
            raise ValueError('Unsupported mast collision geometry')
        for part in components:
            vertices = part.vertices @ rotation.T + xyz
            if not len(vertices) or not np.isfinite(vertices).all():
                raise ValueError('Invalid mast mesh vertices')
            boxes.append((vertices.min(0)-padding, vertices.max(0)+padding))
    boxes = _compact_boxes(boxes, tolerance=.001)
    return {f'cleany_mast/box/{i}': {
        'dims': (high-low).tolist(),
        'pose': [*(base_rotation @ ((low+high)/2) + base_position).tolist(), *quaternion],
    } for i, (low, high) in enumerate(boxes)}
