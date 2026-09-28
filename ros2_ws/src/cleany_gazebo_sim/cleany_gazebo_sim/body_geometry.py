"""Conservative per-element OBBs for the fixed Gazebo evaluation posture."""
from __future__ import annotations

from itertools import product
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np


@dataclass(frozen=True)
class BodyBox:
    name: str
    center: np.ndarray
    rotation: np.ndarray
    half_size: np.ndarray

    def __post_init__(self) -> None:
        for name, shape in [('center', (3,)), ('rotation', (3, 3)), ('half_size', (3,))]:
            value = np.asarray(getattr(self, name), dtype=float).copy()
            if value.shape != shape or not np.isfinite(value).all():
                raise ValueError(f'Invalid body {name}')
            value.setflags(write=False)
            object.__setattr__(self, name, value)
        if (self.half_size < 0).any() or not np.allclose(self.rotation.T@self.rotation, np.eye(3)) or not np.isclose(np.linalg.det(self.rotation), 1.):
            raise ValueError('Invalid body extent or rotation')

    def distances(self, points: np.ndarray) -> np.ndarray:
        local = (points-self.center)@self.rotation
        return np.linalg.norm(np.maximum(np.abs(local)-self.half_size, 0), axis=1)


def load_body_boxes(description: Path, profile: Path, *, merge: bool = True) -> list[BodyBox]:
    """Use both collision and visual bounds, in their own oriented frames.

    Mesh-local boxes retain concavity between elements, not inside an element.
    The supplied park profile must match the simulator; this is not live FK.
    """
    import xacro
    import yaml
    from cleany_gazebo_sim.world.cad_frame import freeze, origin
    urdf = ET.fromstring(xacro.process_file(str(description/'urdf/cleany.urdf.xacro')).toxml())
    config = yaml.safe_load(profile.read_text())
    transforms = freeze(urdf, config['park_joints'], config.get('simulation_park_limits'))
    result = []
    frame_names: set[str] = set()
    for link in urdf.findall('link'):
        for i, element in enumerate(list(link)):
            if element.tag not in ('collision', 'visual'):
                continue
            shape = next(iter(element.find('geometry')))
            if shape.tag == 'mesh':
                uri = shape.get('filename')
                prefix = 'package://cleany_description/'
                if not uri.startswith(prefix):
                    raise ValueError(f'Unsupported mesh URI: {uri}')
                raw = (description/uri.removeprefix(prefix)).read_bytes()
                count = int.from_bytes(raw[80:84], 'little')
                if len(raw) != 84+50*count:
                    raise ValueError('Expected binary STL')
                dtype = np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
                vertices = np.frombuffer(raw, dtype=dtype, offset=84)['vertices'].reshape(-1,3)
                vertices = vertices*np.fromstring(shape.get('scale','1 1 1'), sep=' ')
                low, high = vertices.min(axis=0), vertices.max(axis=0)
            else:
                if shape.tag == 'box':
                    half = np.fromstring(shape.get('size'), sep=' ')/2
                elif shape.tag == 'sphere':
                    half = np.full(3, float(shape.get('radius')))
                elif shape.tag == 'cylinder':
                    half = np.array([float(shape.get('radius'))]*2+[float(shape.get('length'))/2])
                else:
                    raise ValueError(f'Unsupported shape: {shape.tag}')
                low, high = -half, half
            pose = transforms[link.get('name')]@origin(element.find('origin'))
            center = pose[:3,:3]@((low+high)/2)+pose[:3,3]
            name = f'{link.get("name")}/{element.tag}/{i}'
            result.append(BodyBox(name, center, pose[:3,:3], (high-low)/2))
            if link.get('name') == 'base_link':
                mesh_name = shape.get('filename', '')
                is_rail_mesh = shape.tag == 'mesh' and mesh_name.endswith(('_rail.stl', '_upright.stl'))
                # CAD frame collisions use the same 20 mm extrusion cross-section.
                is_rail_box = shape.tag == 'box' and np.count_nonzero(np.isclose(high-low, .02)) >= 2
                if is_rail_mesh or is_rail_box:
                    frame_names.add(name)
    if not merge:
        return result
    if config.get('dynamic_pan', False):
        pivot = transforms['head_pan_link'][:3, 3]
        signs = np.array(list(product((-1., 1.), repeat=3)))
        expanded = []
        for box in result:
            if box.name.startswith(('head_pan_link/', 'head_tilt_link/', 'head_camera')):
                corners = (signs*box.half_size)@box.rotation.T+box.center
                radius = np.linalg.norm(corners[:, :2]-pivot[:2], axis=1).max()
                low, high = corners[:, 2].min(), corners[:, 2].max()
                box = BodyBox(box.name, np.array([*pivot[:2], (low+high)/2]),
                              np.eye(3), np.array([radius, radius, (high-low)/2]))
            expanded.append(box)
        result = expanded
    if frame_names:
        frame_box = merge_body_boxes([b for b in result if b.name in frame_names],
                                     np.eye(4), 'profile_frame/merged')
        signs = np.array(list(product((-1., 1.), repeat=3)))
        # Interior fixed parts already covered by the frame envelope are redundant.
        for b in result:
            if b.name.startswith('base_link/'):
                corners = (signs*b.half_size)@b.rotation.T+b.center
                if np.all(frame_box.distances(corners) <= 1e-10):
                    frame_names.add(b.name)
        result = [b for b in result if b.name not in frame_names]+[frame_box]
    # All rollers, hub visuals and collision elements of a wheel share one OBB.
    for link_name, pose in transforms.items():
        if not link_name.endswith('_wheel_link'):
            continue
        parts = [b for b in result if b.name.split('/')[0] == link_name]
        if parts:
            result = [b for b in result if b.name.split('/')[0] != link_name]
            result.append(merge_body_boxes(parts, pose, link_name+'/merged'))
    # Both parked arms, their mounting parts and the complete head/pole share
    # one upper-body envelope. Keep the chassis and wheels independent.
    children: dict[str, list[str]] = {}
    for joint in urdf.findall('joint'):
        children.setdefault(joint.find('parent').get('link'), []).append(joint.find('child').get('link'))
    pending = ['left_shoulder_yaw_link', 'right_shoulder_yaw_link', 'top_base_link']
    links: set[str] = set()
    while pending:
        link_name = pending.pop()
        if link_name not in links:
            links.add(link_name)
            pending.extend(children.get(link_name, []))
    upper_names = {b.name for b in result if b.name.split('/')[0] in links}
    frame = next((b for b in result if b.name == 'profile_frame/merged'), None)
    if frame is not None:
        ceiling = frame.center[2]+frame.half_size[2]
        for b in result:
            if b.name.startswith('base_link/'):
                high_z = b.center[2]+np.abs(b.rotation[2])@b.half_size
                if high_z > ceiling+1e-10:
                    upper_names.add(b.name)
    if upper_names:
        parts = [b for b in result if b.name in upper_names]
        result = [b for b in result if b.name not in upper_names]
        result.append(merge_body_boxes(parts, np.eye(4), 'upper_body/merged'))
    return result


def merge_body_boxes(boxes: list[BodyBox], frame: np.ndarray, name: str) -> BodyBox:
    """Enclose every input corner in a single box aligned with the link frame.

    Merge physical bounds before the caller adds its margin, so the margin is
    applied once. This deliberately fills gaps between the merged elements.
    """
    if not boxes:
        raise ValueError('Cannot merge empty body geometry')
    signs = np.array(list(product((-1., 1.), repeat=3)))
    corners = np.concatenate([(signs*b.half_size)@b.rotation.T+b.center for b in boxes])
    local = (corners-frame[:3, 3])@frame[:3, :3]
    low, high = local.min(axis=0), local.max(axis=0)
    return BodyBox(name, frame[:3, :3]@((low+high)/2)+frame[:3, 3],
                   frame[:3, :3], (high-low)/2)


def nearest_body(points: np.ndarray, boxes: list[BodyBox]) -> tuple[np.ndarray, np.ndarray]:
    if not boxes:
        raise ValueError('Body geometry is empty')
    points = np.asarray(points, dtype=float).reshape(-1,3)
    if not np.isfinite(points).all():
        raise ValueError('Non-finite points')
    best = np.full(len(points), np.inf)
    indices = np.zeros(len(points), dtype=int)
    for index, box in enumerate(boxes):
        distance = box.distances(points)
        closer = distance < best
        best[closer], indices[closer] = distance[closer], index
    return best, indices


def swept_hits(points: np.ndarray, boxes: list[BodyBox], velocity: tuple[float,float,float],
               margin: float, reaction_s: float, linear_deceleration: float,
               angular_deceleration: float, step_s: float = .05) -> np.ndarray:
    """Sample a constant-twist conservative stopping horizon.

    Holding full velocity through the braking time overestimates path length
    for proportional braking that preserves the commanded curvature. It does
    not bound arbitrary independent axis braking or wheel slip.
    Sample padding bounds missed motion between poses, including rotation.
    This tests observed surfaces only, not visibility of unobserved space.
    """
    vx, vy, wz = velocity
    if not np.isfinite([vx,vy,wz,margin,reaction_s,linear_deceleration,angular_deceleration,step_s]).all():
        raise ValueError('Non-finite sweep parameters')
    if margin < 0 or reaction_s < 0 or min(linear_deceleration,angular_deceleration,step_s) <= 0:
        raise ValueError('Invalid sweep parameters')
    speed = float(np.hypot(vx,vy))
    horizon = reaction_s+max(speed/linear_deceleration,abs(wz)/angular_deceleration)
    if horizon > 10:
        raise ValueError('Unbounded stopping horizon')
    radius = max(np.linalg.norm(b.center[:2])+np.linalg.norm(b.half_size) for b in boxes)
    padding = (speed+abs(wz)*radius)*step_s/2
    hit = np.zeros(len(points),dtype=bool)
    if speed == 0 and wz == 0:
        return nearest_body(points, boxes)[0] <= margin
    for t in np.linspace(0,horizon,max(2,int(np.ceil(horizon/step_s))+1)):
        angle = wz*t
        c,s = np.cos(angle),np.sin(angle)
        if abs(wz) < 1e-8:
            dx,dy = vx*t,vy*t
        else:
            dx,dy = (vx*s+vy*(c-1))/wz,(vx*(1-c)+vy*s)/wz
        rotation = np.array([[c,-s,0],[s,c,0],[0,0,1]])
        moved = (points-[dx,dy,0])@rotation
        distance,_ = nearest_body(moved,boxes)
        hit |= distance <= margin+padding
    return hit
