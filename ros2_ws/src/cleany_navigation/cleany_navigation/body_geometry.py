"""명시적으로 설정한 주차 자세의 요소별 보수적 OBB입니다."""
from __future__ import annotations

from itertools import product
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

_SIGNS = np.array(list(product((-1., 1.), repeat=3)))


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

    def corners(self) -> np.ndarray:
        return (_SIGNS*self.half_size)@self.rotation.T+self.center

    def distances(self, points: np.ndarray) -> np.ndarray:
        local = (points-self.center)@self.rotation
        return np.linalg.norm(np.maximum(np.abs(local)-self.half_size, 0), axis=1)


def load_body_boxes(description: Path, profile: Path, *, merge: bool = True, allow_simulation_limits: bool = False) -> list[BodyBox]:
    """충돌 경계와 렌더링 경계를 각각의 방향 좌표계로 사용합니다.

    메시 로컬 상자는 요소 사이의 오목한 공간을 유지하지만 요소 내부까지 표현하지는
    않습니다. 입력 주차 프로파일은 백엔드 자세와 같아야 하며 실시간 FK가 아닙니다.
    """
    import xacro
    import yaml
    from cleany_navigation.urdf_geometry import forward_kinematics, origin, apply_simulation_geometry
    urdf = ET.fromstring(xacro.process_file(str(description/'urdf/cleany.urdf.xacro')).toxml())
    config = apply_simulation_geometry(urdf, yaml.safe_load(profile.read_text()), allowed=allow_simulation_limits)
    transforms = forward_kinematics(urdf, config['park_joints'], config.get('simulation_park_limits') if allow_simulation_limits else None)
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
                # CAD 프레임 충돌은 같은 20mm 압출재 단면을 사용합니다.
                is_rail_box = shape.tag == 'box' and np.count_nonzero(np.isclose(high-low, .02)) >= 2
                if is_rail_mesh or is_rail_box:
                    frame_names.add(name)
    if not merge:
        return result
    if config.get('dynamic_pan', False):
        pivot = transforms['head_pan_link'][:3, 3]
        expanded = []
        for box in result:
            if box.name.startswith(('head_pan_link/', 'head_tilt_link/', 'head_camera')):
                corners = box.corners()
                radius = np.linalg.norm(corners[:, :2]-pivot[:2], axis=1).max()
                low, high = corners[:, 2].min(), corners[:, 2].max()
                box = BodyBox(box.name, np.array([*pivot[:2], (low+high)/2]),
                              np.eye(3), np.array([radius, radius, (high-low)/2]))
            expanded.append(box)
        result = expanded
    if frame_names:
        frame_box = merge_body_boxes([b for b in result if b.name in frame_names],
                                     np.eye(4), 'profile_frame/merged')
        # 프레임 외곽에 이미 포함된 내부 고정 부품은 중복이므로 제외합니다.
        for b in result:
            if b.name.startswith('base_link/'):
                if np.all(frame_box.distances(b.corners()) <= 1e-10):
                    frame_names.add(b.name)
        result = [b for b in result if b.name not in frame_names]+[frame_box]
    # 바퀴의 모든 롤러, 허브 외형과 충돌 요소는 OBB 하나를 공유합니다.
    for link_name, pose in transforms.items():
        if not link_name.endswith('_wheel_link'):
            continue
        parts = [b for b in result if b.name.split('/')[0] == link_name]
        if parts:
            result = [b for b in result if b.name.split('/')[0] != link_name]
            result.append(merge_body_boxes(parts, pose, link_name+'/merged'))
    # 주차된 양팔, 장착 부품과 전체 헤드 및 기둥은 상체 외곽 하나로 묶습니다.
    # 차체와 바퀴는 독립적으로 유지합니다.
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
    """모든 입력 모서리를 link 좌표계에 정렬된 단일 상자로 둘러쌉니다.

    호출자가 여유를 더하기 전에 물리 경계를 합쳐 여유를 한 번만 적용합니다.
    합친 요소 사이의 틈은 의도적으로 채웁니다.
    """
    if not boxes:
        raise ValueError('Cannot merge empty body geometry')
    corners = np.concatenate([b.corners() for b in boxes])
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
    """일정 twist를 유지하는 보수적인 정지 예측 구간을 표본화합니다.

    제동 시간 내내 전체 속도를 유지하면 명령 곡률을 유지하는 비례 제동보다
    경로 길이를 크게 잡습니다. 임의의 축별 독립 제동이나 바퀴 미끄럼까지
    제한하지는 않습니다. 표본 여유는 회전을 포함한 자세 사이의 누락 움직임을
    덮습니다. 관측 표면만 검사하며 미관측 공간의 가시성을 확인하지 않습니다.
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
