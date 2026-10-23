"""Render collision assets and copied cuRobo caches without ROS or robot motion.

The input directory contains an expanded current.urdf and geometry-cache/*.npz
copied from Jetson. STL assets are read from the checkout, never regenerated.
Figures use one reference joint pose, not a live joint-state measurement.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import colors, font_manager
from matplotlib.collections import PolyCollection
from matplotlib.patches import Circle
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
import trimesh


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'ros2_ws/src/cleany_skill_executor'))
from optimize_arm_collision_meshes import read_stl
from cleany_skill_executor.core.sphere_reduction import reduce_spheres, sphere_cover_error
from cleany_skill_executor.geometry_cache import _array_digest
from cleany_skill_executor.robot_geometry_cache import load_model

BG, INK, MUTED = '#f8fafc', '#152637', '#5b6b7d'
OLD, NEW, BALL, HULL = '#8594a4', '#078a86', '#dd7d20', '#7961c7'
BEFORE_BALL = '#286ba7'


@dataclass
class Part:
    owner: str
    filename: str
    vertices: np.ndarray
    faces: np.ndarray
    original_vertices: np.ndarray
    original_faces: np.ndarray
    transform: np.ndarray
    support_vertices: np.ndarray | None = None


def placement(element: ET.Element | None) -> np.ndarray:
    result = np.eye(4)
    if element is not None:
        result[:3, :3] = Rotation.from_euler('xyz', np.fromstring(
            element.get('rpy', '0 0 0'), sep=' ')).as_matrix()
        result[:3, 3] = np.fromstring(element.get('xyz', '0 0 0'), sep=' ')
    return result


def moved(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def reference_pose() -> dict[str, float]:
    # Same reference angles as test_head_collision_runtime.state(); display only.
    names = [f'{side}_{joint}_joint' for side in ('left', 'right') for joint in
             ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll', 'gripper')]
    values = [-1.6, 2.87, 3.13, -.4, 1.55, -.2, 1.87, 2.44, 3.08, -1.42, 1.55, -.26]
    return {**dict(zip(names, values)), 'head_pan_joint': 0., 'head_tilt_joint': 0.}


def forward_kinematics(root: ET.Element, pose: dict[str, float]) -> dict[str, np.ndarray]:
    links = {link.get('name') for link in root.findall('link')}
    children = {joint.find('child').get('link') for joint in root.findall('joint')}
    roots = links - children
    assert roots == {'base_link'}, roots
    transforms = {'base_link': np.eye(4)}
    remaining = list(root.findall('joint'))
    while remaining:
        ready = [j for j in remaining if j.find('parent').get('link') in transforms]
        if not ready:
            raise ValueError('Disconnected or cyclic URDF')
        for joint in ready:
            local = placement(joint.find('origin'))
            kind, name = joint.get('type'), joint.get('name')
            q = pose.get(name, 0.)
            mimic = joint.find('mimic')
            if mimic is not None:
                q = pose[mimic.get('joint')] * float(mimic.get('multiplier', 1)) + float(mimic.get('offset', 0))
            if kind in ('revolute', 'continuous', 'prismatic'):
                axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                motion = np.eye(4)
                if kind == 'prismatic':
                    motion[:3, 3] = q * axis
                else:
                    motion[:3, :3] = Rotation.from_rotvec(q * axis / np.linalg.norm(axis)).as_matrix()
                local = local @ motion
            elif kind != 'fixed':
                raise ValueError(f'Unsupported joint: {kind}')
            transforms[joint.find('child').get('link')] = transforms[joint.find('parent').get('link')] @ local
            remaining.remove(joint)
    assert len(transforms) == len(links)
    return transforms


def asset_path(uri: str) -> Path:
    package, relative = uri.removeprefix('package://').split('/', 1)
    path = ROOT / 'ros2_ws/src' / package / relative
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def fitted(path: Path, scale: tuple[float, ...], source: Path) -> tuple[np.ndarray, ...]:
    settings_path = source / 'fitting.json'
    settings = json.loads(settings_path.read_text()) if settings_path.exists() else {'schema': 2}
    schema = settings['schema']
    algorithm = {2: 'convex_hull/subdivide', 3: 'convex_hull/slab_enclosure_v1', 4: 'component_cells_v1'}[schema]
    digest = hashlib.sha256(path.read_bytes())
    digest.update(json.dumps([schema, scale, settings.get('spheres_per_mesh', 8),
                             settings.get('padding_m', .002), '4.5.3/0.7.8', algorithm]).encode())
    cache = source / 'geometry-cache' / (digest.hexdigest() + '.npz')
    with np.load(cache, allow_pickle=False) as stored:
        arrays = tuple(stored[name].copy() for name in ('centers', 'radii', 'vertices'))
        assert str(stored['data_sha256']) == _array_digest(arrays), cache
    return arrays


def load_geometry(source: Path):
    settings_path = source / 'fitting.json'
    settings = json.loads(settings_path.read_text()) if settings_path.exists() else {}
    root = ET.parse(source / 'current.urdf').getroot()
    pose = reference_pose()
    transforms = forward_kinematics(root, pose)
    parts, before, after, reductions = [], {}, {}, {}
    for link in root.findall('link'):
        name = link.get('name')
        centers, radii = [], []
        for collision in link.findall('collision'):
            support_vertices = None
            origin = placement(collision.find('origin'))
            geometry = collision.find('geometry')
            mesh = geometry.find('mesh')
            if mesh is not None:
                path = asset_path(mesh.get('filename'))
                scale = tuple(float(v) for v in mesh.get('scale', '1 1 1').split())
                current = read_stl(path)
                raw_path = Path(str(path).replace('/meshes/collision/', '/meshes/'))
                original = read_stl(raw_path) if raw_path != path else current
                vertices, faces = current.vertices * scale, current.faces
                old_vertices, old_faces = original.vertices * scale, original.faces
                filename = str(path.relative_to(ROOT))
                if name != 'base_link':
                    c, r, support_vertices = fitted(path, scale, source)
                    centers.extend(moved(c, origin))
                    radii.extend(r)
            else:
                box, cylinder, sphere = geometry.find('box'), geometry.find('cylinder'), geometry.find('sphere')
                if box is not None:
                    shape = trimesh.creation.box(extents=np.fromstring(box.get('size'), sep=' '))
                elif cylinder is not None:
                    shape = trimesh.creation.cylinder(radius=float(cylinder.get('radius')),
                                                      height=float(cylinder.get('length')), sections=24)
                elif sphere is not None:
                    shape = trimesh.creation.icosphere(subdivisions=1, radius=float(sphere.get('radius')))
                else:
                    raise ValueError('Unsupported collision geometry')
                vertices, faces = shape.vertices, shape.faces
                old_vertices, old_faces, filename = vertices, faces, 'primitive'
            parts.append(Part(name, filename, vertices, faces, old_vertices, old_faces,
                              transforms[name] @ origin, support_vertices))
        if centers:
            c, r = np.asarray(centers), np.asarray(radii)
            nc, nr = reduce_spheres(c, r, settings.get('budget_per_link', 8))
            error = sphere_cover_error(c, r, nc, nr)
            assert error <= 1e-10
            before[name] = (c, r)
            after[name] = (nc, nr)
            reductions[name] = {'before': len(r), 'after': len(nr), 'cover_error_m': error}

    # Require a checksum-verified full cache matching every reconstructed sphere.
    matches = []
    for path in sorted((source / 'geometry-cache').glob('model-*.npz')):
        if settings.get('full_cache') and path.name != settings['full_cache']:
            continue
        model = SimpleNamespace()
        if not load_model(path, model) or len(model.bodies) != len(parts) or set(model.spheres) != set(after):
            continue
        if any(len(model.spheres[n]) != len(after[n][1]) for n in after):
            continue
        if all(sphere_cover_error(*before[n],
                np.asarray([s['center'] for s in model.spheres[n]]),
                np.asarray([s['radius'] for s in model.spheres[n]])) <= 1e-10 for n in after):
            matches.append((path, model))
    if not matches:
        raise ValueError('No full model cache matches current URDF sphere fits')
    geometry_hashes = {model.sphere_reduction['validation_geometry_sha256'] for _, model in matches}
    if len(geometry_hashes) != 1:
        raise ValueError('Ambiguous validation geometry across matching caches')
    model_path, model = matches[0]
    # Symmetric merge ties can differ after platform-dependent Euler rounding.
    # Display the authoritative cache, whose containment was checked above.
    after = {name: (np.array([s['center'] for s in spheres]), np.array([s['radius'] for s in spheres]))
             for name, spheres in model.spheres.items()}
    for part, body in zip(parts, model.bodies, strict=True):
        assert part.owner == body.owner
        if part.filename != 'primitive':
            part.support_vertices = moved(body.vertices, np.linalg.inv(part.transform) @ transforms[part.owner])
    assert model.sphere_reduction['before'] == sum(item['before'] for item in reductions.values())
    assert model.sphere_reduction['after'] == sum(item['after'] for item in reductions.values())
    return parts, transforms, before, after, model, {
        'pose_rad': pose, 'sphere_reduction': reductions,
        'full_cache': model_path.name, 'validation_geometry_sha256': geometry_hashes.pop(),
        'matching_full_caches': [p.name for p, _ in matches],
    }


def camera(azimuth: float = -55, elevation: float = 22) -> np.ndarray:
    a, e = np.deg2rad([azimuth, elevation])
    direction = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    right = np.array([-np.sin(a), np.cos(a), 0.])
    up = np.cross(direction, right)
    return np.vstack((right, up, direction))


def bounds(points: np.ndarray, view: np.ndarray, pad=.08):
    projected = points @ view.T
    lo, hi = projected[:, :2].min(0), projected[:, :2].max(0)
    size = hi - lo
    return lo - size * pad, hi + size * pad


def axis(ax, limits, title, detail=''):
    lo, hi = limits
    ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), aspect='equal')
    ax.set_axis_off()
    ax.set_title(title, loc='left', color=INK, fontsize=14, fontweight='bold', pad=20)
    if detail:
        ax.text(0, 1.01, detail, transform=ax.transAxes, fontsize=10, color=MUTED, va='bottom')


def surface(ax, entries, view, *, tint=NEW, edges=False, alpha=1., linewidth=.12):
    """Orthographic triangle rendering; no decimation or face subsampling."""
    triangles = np.concatenate([v[f] for v, f in entries])
    normal = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-18)
    light = np.array([-.3, -.5, .81])
    shade = .45 + .55 * np.abs(normal @ light)
    base = np.asarray(colors.to_rgb(tint))
    facecolors = base[None, :] * shade[:, None] + .08 * (1 - shade[:, None])
    projected = triangles @ view.T
    order = np.argsort(projected[:, :, 2].mean(1), kind='stable')
    collection = PolyCollection(projected[order, :, :2], facecolors=facecolors[order],
                                edgecolors=INK if edges else 'none', linewidths=linewidth,
                                alpha=alpha, rasterized=True)
    ax.add_collection(collection)


def sphere_overlay(ax, spheres, view, tint=BALL, fill=.13):
    c, r = spheres
    projected = c @ view.T
    for index in np.argsort(projected[:, 2]):
        ax.add_patch(Circle(projected[index, :2], r[index], facecolor=colors.to_rgba(tint, fill),
                            edgecolor=colors.to_rgba(tint, .9), linewidth=.65))
    ax.scatter(projected[:, 0], projected[:, 1], s=3, c=tint, zorder=4)


def world_parts(parts, original=False, owner=None):
    return [(moved(p.original_vertices if original else p.vertices, p.transform),
             p.original_faces if original else p.faces) for p in parts
            if owner is None or owner(p.owner)]


def world_spheres(spheres, transforms, owner=None):
    selected = [(moved(c, transforms[name]), r) for name, (c, r) in spheres.items()
                if owner is None or owner(name)]
    return np.concatenate([s[0] for s in selected]), np.concatenate([s[1] for s in selected])


def sphere_counts(reductions):
    filters = {'all': lambda n: True, 'arms': lambda n: n.startswith(('left_', 'right_')),
               'right': lambda n: n.startswith('right_'),
               'head': lambda n: not n.startswith(('left_', 'right_'))}
    return {group: {key: sum(value[key] for name, value in reductions.items() if predicate(name))
                    for key in ('before', 'after')} for group, predicate in filters.items()}


def header(fig, title, subtitle):
    fig.set_facecolor(BG)
    fig.text(.035, .955, title, fontsize=24, fontweight='bold', color=INK)
    fig.text(.035, .914, subtitle, fontsize=11, color=MUTED)


def save(fig, output, filename):
    fig.savefig(output / filename, dpi=180, facecolor=BG)
    plt.close(fig)


def render_overview(parts, transforms, before, after, model, report, output):
    view = camera(-58, 18)
    old, new = world_parts(parts, True), world_parts(parts)
    c, r = world_spheres(after, transforms)
    limits = bounds(np.concatenate([v for v, _ in new] + [c - r[:, None], c + r[:, None]]), view, .06)
    fig, axes = plt.subplots(1, 4, figsize=(22, 9))
    fig.subplots_adjust(left=.025, right=.985, bottom=.13, top=.80, wspace=.12)
    header(fig, 'Cleany | 충돌 모델을 같은 자세로 비교',
           '현재 브랜치의 URDF + STL + Jetson cuRobo 캐시 · 정사영 · 비교용 기준 자세 (실시간 자세 아님)')
    mesh = report['current_mesh_counts']
    counts = sphere_counts(report['sphere_reduction'])
    axis(axes[0], limits, '01  원본 CAD 충돌 메시', f"{mesh['original_vertices']:,} 정점 / {mesh['original_triangles']:,} 삼각형")
    surface(axes[0], old, view, tint=OLD)
    axis(axes[1], limits, '02  MoveIt 최적화 메시', f"{mesh['vertices']:,} 정점 / {mesh['triangles']:,} 삼각형")
    surface(axes[1], new, view, tint=NEW, edges=True, linewidth=.08)
    axis(axes[2], limits, '03  cuRobo 제안용 충돌 구',
         f"{len(c)}개 = 양팔 {counts['arms']['after']} + 머리·마스트 {counts['head']['after']}")
    surface(axes[2], new, view, tint='#cbd3dc')
    sphere_overlay(axes[2], (c, r), view)
    axis(axes[3], limits, '04  GPU 검증용 볼록 형상', f'{len(model.bodies)}개 body / {sum(len(b.vertices) for b in model.bodies):,} support 정점')
    hulls = [(moved(b.vertices, transforms[b.owner]), ConvexHull(b.vertices).simplices) for b in model.bodies]
    surface(axes[3], hulls, view, tint=HULL, edges=True, linewidth=.08)
    fig.text(.035, .072, '메시 정점 수는 현재 URDF의 mesh 인스턴스 합계입니다. 구·박스·실린더 primitive는 집계에서 제외합니다.', fontsize=11, color=INK)
    fig.text(.035, .043, '주황 원은 3D 구의 정사영 외곽입니다. 현행 hybrid는 cuRobo로 IK 후보를 만들고, MoveIt으로 전체 장면과 경로를 검사합니다.', fontsize=11, color=MUTED)
    save(fig, output, '01-robot-overview.png')


def render_mesh_details(parts, output):
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.subplots_adjust(left=.04, right=.98, bottom=.12, top=.81, hspace=.48, wspace=.14)
    header(fig, '삼각형과 충돌 정점 | 팔 부품 확대',
           '왼쪽과 가운데는 STL의 모든 삼각형을 표시 · 부품마다 동일 카메라와 동일 축척 · 오른쪽은 cuRobo용 볼록 외피')
    for row, filename in enumerate(('Upper_Arm_5cm.stl', 'Lower_Arm_5cm.stl')):
        part = next(p for p in parts if p.filename.endswith('/collision/' + filename))
        old, new = part.original_vertices, part.vertices
        view = camera(-45, 24)
        limits = bounds(np.vstack((old, new)), view, .1)
        support = part.support_vertices
        hull = ConvexHull(support)
        title = '상완' if row == 0 else '하완'
        labels = [(f'{title} · 원본', f'{len(old):,} 정점 / {len(part.original_faces):,} 삼각형'),
                  (f'{title} · MoveIt 최적화', f'{len(new):,} 정점 / {len(part.faces):,} 삼각형  (정점 {100 * (1-len(new)/len(old)):.1f}% 감소)'),
                  (f'{title} · GPU 볼록 외피', f'{len(support):,} support 정점 / Jetson 캐시')]
        for col in range(3):
            axis(axes[row, col], limits, *labels[col])
        surface(axes[row, 0], [(old, part.original_faces)], view, tint=OLD, edges=True, linewidth=.05)
        surface(axes[row, 1], [(new, part.faces)], view, tint=NEW, edges=True, linewidth=.22)
        surface(axes[row, 2], [(support, hull.simplices)], view, tint=HULL, edges=True, linewidth=.35)
        for col, vertices in ((1, new), (2, support)):
            projected = vertices @ view.T
            axes[row, col].scatter(projected[:, 0], projected[:, 1], c=INK, s=.9 if col == 1 else 3, alpha=.45)
    fig.text(.04, .071, 'MoveIt: 연결부와 포크를 분할한 뒤 평면으로 외접. GPU 외피는 메시 전체를 볼록화해 빈 공간을 메울 수 있습니다.', fontsize=11, color=INK)
    limit = json.loads((ROOT / 'configs/robot/arm_collision_meshes.json').read_text())['max_hull_expansion_m'] * 1000
    fig.text(.04, .045, f'MoveIt 팽창 상한 {limit:g} mm는 각 분할 조각의 볼록 외피 기준입니다. 원본 CAD 표면과의 거리 상한은 아닙니다.', fontsize=11, color=MUTED)
    save(fig, output, '02-mesh-vertices-detail.png')


def render_spheres(parts, transforms, before, after, output):
    fig, axes = plt.subplots(2, 2, figsize=(16, 13))
    fig.subplots_adjust(left=.055, right=.965, bottom=.11, top=.82, wspace=.15, hspace=.42)
    header(fig, 'cuRobo | 같은 링크 안에서 충돌 구 병합',
           f'현재 최적화 메시의 저장된 fitting 결과를 그대로 사용 · 전후 동일 자세 · 링크당 최대 {max(len(r) for c, r in after.values())}개')
    for row, predicate in enumerate((lambda n: n.startswith('right_'), lambda n: n == 'right_gripper_frame')):
        a, b = world_spheres(before, transforms, predicate), world_spheres(after, transforms, predicate)
        entries = world_parts(parts, owner=predicate)
        view = camera(-70, 25) if row == 0 else camera(-20, 25)
        limits = bounds(np.concatenate([v for v, _ in entries] + [a[0] - a[1][:, None], a[0] + a[1][:, None],
                                                                        b[0] - b[1][:, None], b[0] + b[1][:, None]]), view, .04)
        title = '오른팔 전체' if row == 0 else '오른쪽 고정 그리퍼 링크 확대'
        for col, spheres in enumerate((a, b)):
            axis(axes[row, col], limits, f"{title} · {'병합 전' if col == 0 else '병합 후'}", f'{len(spheres[1])}개 충돌 구')
            surface(axes[row, col], entries, view, tint='#c5cdd6')
            sphere_overlay(axes[row, col], spheres, view, tint=BEFORE_BALL if col == 0 else BALL, fill=.13)
    count_before, count_after = sum(len(r) for _, r in before.values()), sum(len(r) for _, r in after.values())
    arm_before = sum(len(r) for n, (_, r) in before.items() if n.startswith(('left_', 'right_')))
    arm_after = sum(len(r) for n, (_, r) in after.items() if n.startswith(('left_', 'right_')))
    fig.text(.055, .068, f'현재 전체 모델: {count_before} → {count_after}개. 양팔: {arm_before} → {arm_after}개. '
             f'머리·마스트: {count_before-arm_before} → {count_after-arm_after}개.', fontsize=12, color=INK)
    fig.text(.055, .042, '병합 구는 원래 구를 포함합니다. 구의 개수는 줄지만 덮는 영역이 커질 수 있습니다. STL과 최종 검증용 형상을 삭제하는 작업은 아닙니다.', fontsize=11, color=MUTED)
    save(fig, output, '03-sphere-reduction.png')


def render_counts(report, output):
    meshes = report['manifest']['meshes']
    fig, axes = plt.subplots(1, 2, figsize=(16, max(8, len(meshes) * .6)), gridspec_kw={'width_ratios': [1.6, 1]})
    fig.subplots_adjust(left=.17, right=.96, bottom=.18, top=.79, wspace=.42)
    header(fig, '어디에서 줄었나 | 정점 수와 충돌 구 수', '파일별 STL 정점 수와 현재 URDF의 링크별 구 수 · 형상 데이터에서 직접 집계')
    names = [n.replace('.stl', '').replace('_', ' ') for n in meshes]
    y = np.arange(len(names))
    old = [m['source_vertices'] for m in meshes.values()]
    new = [m['vertices'] for m in meshes.values()]
    ax = axes[0]
    ax.barh(y+.18, old, height=.31, color=OLD, label='원본')
    ax.barh(y-.18, new, height=.31, color=NEW, label='최적화')
    ax.set_yticks(y, names)
    ax.invert_yaxis()
    ax.set_xscale('log')
    ax.set_xlim(10, 220000)
    ax.set_xlabel('정점 수 (로그 축)')
    ax.set_title(f'경량화 대상 STL {len(meshes)}종', loc='left', pad=16)
    for i, (a, b) in enumerate(zip(old, new)):
        ax.text(a*1.08, i+.18, f'{a:,}', va='center', fontsize=9, color=MUTED)
        ax.text(b*1.08, i-.18, f'{b:,}', va='center', fontsize=9, color=NEW)
    ax.legend(loc='lower right', frameon=False)
    ax = axes[1]
    categories = ['전체 모델', '양팔', '오른팔', '머리·마스트']
    counts = sphere_counts(report['sphere_reduction'])
    a = [counts[n]['before'] for n in ('all', 'arms', 'right', 'head')]
    b = [counts[n]['after'] for n in ('all', 'arms', 'right', 'head')]
    ax.barh(np.arange(4)+.17, a, height=.3, color=OLD, label='병합 전')
    ax.barh(np.arange(4)-.17, b, height=.3, color=BALL, label='병합 후')
    ax.set_yticks(np.arange(4), categories)
    ax.invert_yaxis()
    ax.set_xlim(0, max(a + b) * 1.18)
    ax.set_xlabel('충돌 구 수')
    ax.set_title('현재 모델의 fitting → 링크별 병합', loc='left', pad=16)
    for i, (v, w) in enumerate(zip(a, b)):
        ax.text(v+4, i+.17, str(v), va='center', fontsize=10)
        ax.text(w+4, i-.17, str(w), va='center', fontsize=10)
    ax.legend(loc='lower right', frameon=False)
    for ax in axes:
        ax.set_facecolor(BG)
        ax.spines[['top', 'right']].set_visible(False)
        ax.spines[['left', 'bottom']].set_color('#ccd5df')
        ax.tick_params(length=0)
        ax.grid(axis='x', color='#dce3ea', linewidth=.6)
        ax.set_axisbelow(True)
    fig.text(.035, .095, '이전 10/16 팔 모델의 기록은 256 → 96개입니다. 현재 모델은 그리퍼 형상 변경 등이 반영되어 병합 전 개수가 다릅니다.', fontsize=11, color=INK)
    fig.text(.035, .058, '개수 감소는 계산량을 설명하는 지표입니다. 이 그림은 새로운 속도 측정이나 충돌 안전성 검증 결과가 아닙니다.', fontsize=11, color=MUTED)
    save(fig, output, '04-count-comparison.png')


def sphere_review_html(source: Path, output: Path, model) -> tuple[str, str]:
    """Attach a saved audit only when it matches the displayed geometry."""
    review = json.loads(source.read_text())
    if review['geometry_sha256'] != model.sphere_reduction['validation_geometry_sha256']:
        raise ValueError('Sphere review does not match the displayed collision geometry')
    image_path = source.parent / 'sphere-overreach.png'
    (output / 'sphere-overreach.png').write_bytes(image_path.read_bytes())
    (output / 'sphere-review.json').write_text(json.dumps(review, indent=2, ensure_ascii=False) + '\n')
    labels = [('right_upper_arm_link', '상완'), ('right_lower_arm_link', '하완'),
              ('right_moving_jaw_link', '오른쪽 가동 집게'), ('top_base_link', '마스트')]
    rows = ''.join(f'<tr><td>{label}</td><td>{review["links"][name]["variants"]["current16"]["max_directional_cad_overreach_mm"]:.1f} mm</td></tr>'
                   for name, label in labels)
    runtime = review['runtime_verification']
    valid = {mode: {row['link'] for row in records if row['valid'] and not row['contacts']}
             for mode, records in runtime['moveit'].items()}
    false_positives = sum(row['variants']['current16']['collision']
        and row['link'] in valid['original'] and row['link'] in valid['optimized']
        for row in runtime['curobo']['witnesses'])
    banner = '''<aside class="review-notice"><strong>cuRobo 재검토 결과 반영</strong>
<span>현재 구는 과도하게 부풀어 있습니다. 3D에는 검토한 기존 16개/링크 배치를 표시하며, 구 재배치는 아직 적용하지 않았습니다.</span>
<a href="#sphere-review">비교 이미지와 검증 결과 보기 ↓</a></aside>'''
    section = f'''<section id="sphere-review" class="review-panel">
<h2>cuRobo 충돌 구 재검토</h2>
<p>검토한 메시 링크 {len(review['links'])}개. 기존 구를 링크당 16개로 병합하면서 CAD 주변 빈 공간까지 크게 감싸는 문제가 확인됐습니다.</p>
<div class="review-grid"><div><h3>CAD 외곽보다 더 튀어나온 거리</h3>
<table><thead><tr><th>부위</th><th>측정된 최대 차이</th></tr></thead><tbody>{rows}</tbody></table>
<p class="review-note">648개 방향의 외곽 차이 중 최댓값입니다. 모든 표면 간 거리의 최댓값은 아닙니다.</p></div>
<div><h3>Jetson CUDA와 MoveIt 대조</h3>
<p><strong>선별한 장애물 {false_positives}개에서 충돌 오탐 재현.</strong> 원본 CAD와 최적화 MoveIt 메시에서는 비충돌이지만, 현재 cuRobo 구에서는 충돌로 판정했습니다.</p>
<p>32개/링크 비교안은 이 사례들을 통과했지만, 하완과 마스트의 과대 팽창은 남았습니다. 개수 증가만으로 해결된 상태가 아닙니다.</p>
<p class="review-note">기준 자세에서 검사한 사례이며 전체 계획 성공률이나 속도 측정은 아닙니다. base와 primitive는 제안용 구에서 제외되고 최종 전체 형상 검사에서 확인합니다.</p></div></div>
<a href="sphere-overreach.png"><img src="sphere-overreach.png" alt="원본 CAD, 병합 전 구, 현재 링크당 16개 구의 하완·가동 집게·마스트 비교" loading="lazy"></a>
<p><a href="sphere-overreach.png">비교 이미지 원본</a> · <a href="sphere-review.json">전체 수치와 실행 결과 JSON</a></p>
</section>'''
    return banner, section


def sphere_fit_html(source: Path, output: Path, after, model):
    """Load unapplied fits with their current-model baseline for comparison."""
    comparison = json.loads(source.read_text())
    if comparison['geometry_sha256'] != model.sphere_reduction['validation_geometry_sha256']:
        raise ValueError('Sphere fits do not match displayed geometry')
    if set(comparison['links']) != set(after):
        raise ValueError('Sphere fit comparison is missing links')
    candidates = {}
    with np.load(source.parent / 'fits.npz', allow_pickle=False) as data:
        for name, (centers, radii) in after.items():
            if not (np.array_equal(centers, data[f'{name}__current16__centers'])
                    and np.array_equal(radii, data[f'{name}__current16__radii'])):
                raise ValueError('Sphere fit baseline differs from displayed model')
        for label in ('direct16', 'direct32'):
            candidates[label] = {name: (data[f'{name}__{label}__centers'].copy(),
                                       data[f'{name}__{label}__radii'].copy()) for name in after}
    for filename in ('fits.npz', 'sphere-fit-comparison.png'):
        (output / filename).write_bytes((source.parent / filename).read_bytes())
    (output / 'sphere-fit-comparison.json').write_text(json.dumps(comparison, indent=2) + '\n')
    rows = ''
    for name, label in [('right_upper_arm_link', '상완'), ('right_lower_arm_link', '하완'),
                        ('right_moving_jaw_link', '오른쪽 가동 집게'), ('top_base_link', '마스트')]:
        values = comparison['links'][name]['variants']
        rows += f'<tr><td>{label}</td>' + ''.join(
            f'<td>{values[key]["max_directional_cad_overreach_mm"]:.1f} mm</td>'
            for key in ('current16', 'direct16', 'direct32')) + '</tr>'
    count = sum(len(r) for c, r in after.values())
    samples = sum(row['coverage_samples'] for row in comparison['links'].values())
    runtime_note = ''
    if 'verification' in comparison:
        result = comparison['verification']
        runtime_note = (f'<p>동일한 Jetson CUDA 장애물 검사에서 기존 오탐 '
            f'{result["cuda_false_positives_current16"]}건 → 16개 비교안 '
            f'{result["cuda_false_positives_direct16"]}건. 선별한 네 사례의 결과이며 '
            '전체 환경의 오탐률을 뜻하지 않습니다.</p>')
    section = f'''<section id="sphere-fit" class="review-panel">
<h2>같은 구 개수로 형상에 직접 맞춘 비교안</h2>
<p><strong>현재 {count}개 → 형상 맞춤 {count}개.</strong> 구끼리 병합하지 않고 부품 형상을 분할해 구 중심과 반지름을 다시 계산했습니다. 3D 메뉴에서 현재 배치와 16개/32개 비교안을 전환할 수 있습니다.</p>
<table><thead><tr><th>부위</th><th>현재 16개/링크</th><th>맞춤 16개/링크</th><th>맞춤 32개/링크</th></tr></thead><tbody>{rows}</tbody></table>
<p class="review-note">수치는 648개 방향의 CAD 외곽 초과 거리입니다. 링크 15개의 원본 CAD 정점과 면 표본 {samples:,}개에서 누락 0을 확인했습니다. 분할한 볼록 조각의 포함 관계도 검사했습니다.</p>
{runtime_note}
<p><strong>검토용 비교안이며 로봇 설정에는 미적용입니다.</strong> 구가 차지하는 공간은 줄었지만, 전체 계획 성공률과 실행 속도는 아직 측정하지 않았습니다.</p>
<a href="sphere-fit-comparison.png"><img src="sphere-fit-comparison.png" alt="현재 구와 같은 개수로 다시 맞춘 구, 32개 비교안의 실제 형상 비교" loading="lazy"></a>
<p><a href="sphere-fit-comparison.png">비교 이미지 원본</a> · <a href="sphere-fit-comparison.json">전체 15개 링크 측정 결과</a></p>
</section>'''
    return candidates, section


def interactive(parts, transforms, before, after, model, output, has_baseline=False,
                sphere_review: Path | None = None, sphere_fit: Path | None = None):
    import plotly.graph_objects as go
    from plotly.offline import get_plotlyjs

    def mesh_trace(entries, name, color, opacity=1.):
        vertices, faces, offset = [], [], 0
        for v, f in entries:
            vertices.append(v)
            faces.append(f + offset)
            offset += len(v)
        v, f = np.concatenate(vertices), np.concatenate(faces)
        return go.Mesh3d(x=v[:, 0], y=v[:, 1], z=v[:, 2], i=f[:, 0], j=f[:, 1], k=f[:, 2],
                         name=name, color=color, opacity=opacity, flatshading=True,
                         lighting=dict(ambient=.5, diffuse=.8), hoverinfo='name', visible=False, showlegend=True)

    traces = [mesh_trace(world_parts(parts), 'MoveIt 최적화 메시', NEW)]
    count_before, count_after = sum(len(r) for _, r in before.values()), sum(len(r) for _, r in after.values())
    for sphere_set, name, color in ((before, f'병합 전 충돌 구 ({count_before}개)', BEFORE_BALL),
                                   (after, f'병합 후 충돌 구 ({count_after}개)', BALL)):
        unit = trimesh.creation.icosphere(subdivisions=1 if sphere_set is before else 2)
        c, r = world_spheres(sphere_set, transforms)
        traces.append(mesh_trace([(unit.vertices * radius + center, unit.faces) for center, radius in zip(c, r)], name, color, .65))
    hulls = [(moved(b.vertices, transforms[b.owner]), ConvexHull(b.vertices).simplices) for b in model.bodies]
    traces.append(mesh_trace(hulls, 'GPU 검증용 볼록 형상', HULL, .45))
    v = np.concatenate([vertices for vertices, _ in hulls])
    traces.append(go.Scatter3d(x=v[:, 0], y=v[:, 1], z=v[:, 2], mode='markers',
                               marker=dict(size=2, color=HULL), name='GPU support 정점', visible=False))
    traces[0].visible = True
    modes = [('MoveIt 메시', [True, False, False, False, False]),
             ('병합 전 구 + 메시', [True, True, False, False, False]),
             ('병합 후 구 + 메시', [True, False, True, False, False]),
             ('구 전후 겹쳐 보기', [False, True, True, False, False]),
             ('GPU 볼록 형상 + 정점', [False, False, False, True, True]),
             ('메시 + GPU 정점', [True, False, False, False, True])]
    fit_path = sphere_fit or output / 'sphere-fit-comparison.json'
    fit_section = ''
    if sphere_fit or fit_path.exists():
        candidates, fit_section = sphere_fit_html(fit_path, output, after, model)
        modes = [(label, values + [False, False]) for label, values in modes]
        for index, (label, color) in enumerate((('direct16', '#386de8'), ('direct32', '#8b5cf6'))):
            unit = trimesh.creation.icosphere(subdivisions=2)
            c, r = world_spheres(candidates[label], transforms)
            name = f'형상 맞춤 {label.removeprefix("direct")}개/링크 · 비교안 미적용'
            traces.append(mesh_trace([(unit.vertices * radius + center, unit.faces)
                for center, radius in zip(c, r)], name, color, .65))
            visible = [True, False, False, False, False, index == 0, index == 1]
            modes.append((name + ' + 메시', visible))
    fig = go.Figure(traces)
    fig.update_layout(paper_bgcolor=BG, font=dict(family='Arial, Apple SD Gothic Neo, sans-serif', color=INK),
        margin=dict(l=0, r=0, b=0, t=58), height=760, showlegend=True,
        legend=dict(orientation='h', y=0, x=.02, bgcolor='rgba(248,250,252,.8)'),
        scene=dict(aspectmode='data', camera=dict(eye=dict(x=1.45, y=-1.8, z=.95), projection=dict(type='orthographic')),
                   xaxis_title='X (m)', yaxis_title='Y (m)', zaxis_title='Z (m)', bgcolor=BG),
        updatemenus=[dict(type='dropdown', x=0, y=1.07, xanchor='left', yanchor='top',
                         buttons=[dict(label=label, method='update', args=[dict(visible=values)]) for label, values in modes])])
    body = fig.to_html(full_html=False, include_plotlyjs=False, div_id='collision-view',
                       config=dict(responsive=True, displaylogo=False, modeBarButtonsToRemove=['sendChartToCloud']))
    review_path = sphere_review or output / 'sphere-review.json'
    banner, section = sphere_review_html(review_path, output, model) if sphere_review or review_path.exists() else ('', '')
    if fit_section:
        banner = '''<aside class="review-notice"><strong>형상에 직접 맞춘 구 비교안 추가</strong>
<span>주황 = 현재 배치, 파랑 = 같은 16개/링크 비교안, 보라 = 32개/링크 비교안. 운영 설정에는 아직 적용하지 않았습니다.</span>
<a href="#sphere-fit">최신 비교 이미지와 측정 결과 보기 ↓</a> <a href="#sphere-review">기존 배치의 문제점 보기</a></aside>'''
    html = '''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Cleany 충돌 모델 3D</title><style>body{margin:0;background:#f8fafc;color:#152637;font:15px/1.6 -apple-system,sans-serif}main{max-width:1400px;margin:24px auto;padding:0 20px}h1{font-size:26px;margin:0}p{margin:8px 0;color:#5b6b7d}a{color:#078a86}footer{font-size:13px;padding:12px 0}
.review-notice{margin:18px 0;padding:14px 18px;background:#fff4e5;border-left:4px solid #dd7d20;border-radius:6px;display:grid;gap:4px}
.review-panel{margin:28px 0;padding:24px;border:1px solid #dbe3e9;border-radius:12px;background:white;scroll-margin-top:20px}
.review-panel h2{margin:0 0 8px;font-size:23px}.review-panel h3{font-size:17px;margin:12px 0}.review-grid{display:grid;grid-template-columns:1fr 1.4fr;gap:32px}
.review-panel table{border-collapse:collapse;width:100%;text-align:left}.review-panel th,.review-panel td{padding:8px 12px;border-bottom:1px solid #e4e9ee}.review-panel th{background:#f8fafc}
.review-panel img{display:block;width:100%;height:auto;margin-top:20px}.review-note{font-size:13px;color:#5b6b7d}@media(max-width:700px){.review-grid{grid-template-columns:1fr;gap:8px}.review-panel{padding:16px}}
</style><main>
<h1>Cleany 충돌 모델 3D</h1><p>드래그로 회전 · 스크롤로 확대 · 메뉴로 메시 / 구 / 충돌 정점 전환. 기준 자세이며 실시간 로봇 상태가 아닙니다.</p>'''
    html += banner + '<script>' + get_plotlyjs() + '</script>' + body + fit_section + section
    html += f'<footer>생성된 구 {count_before}개 → 병합 후 사용하는 구 {count_after}개. 구 병합은 원래 구의 영역을 포함합니다.<br>'
    if has_baseline:
        html += '<a href="05-before-after.png">이번 수정 전후 비교 PNG</a> · '
    html += '<a href="06-angular-before-after.png">전체 평면화 전후 비교</a><br>' if (output / '06-angular-before-after.png').exists() else ''
    html += '모든 부품 확대: ' + ' · '.join(
        f'<a href="{path.name}">{index + 1}</a>'
        for index, path in enumerate(sorted(output.glob('all-parts-*.png')))) + '<br>'
    html += '''
주황 구는 IK 제안용 근사입니다. 현재 hybrid의 전체 장면·경로 검증은 MoveIt이 담당합니다. GPU 볼록 형상은 별도 cuRobo 검증 경로의 캐시입니다.<br>
<a href="01-robot-overview.png">전체 비교 PNG</a> · <a href="02-mesh-vertices-detail.png">메시 확대 PNG</a> · <a href="03-sphere-reduction.png">구 병합 PNG</a> · <a href="04-count-comparison.png">개수 비교 PNG</a> · <a href="provenance.json">데이터 근거</a></footer></main></html>'''
    (output / 'collision-inspector.html').write_text(html)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--baseline-cache', type=Path)
    parser.add_argument('--baseline-meshes', type=Path)
    parser.add_argument('--sphere-review', type=Path, help='Saved sphere audit JSON with sphere-overreach.png alongside it')
    parser.add_argument('--sphere-fit', type=Path, help='Offline fit comparison JSON with fits.npz and sphere-fit-comparison.png')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    font = Path('/System/Library/Fonts/AppleSDGothicNeo.ttc')
    if font.is_file():
        font_manager.fontManager.addfont(font)
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=font).get_name()
    plt.rcParams.update({'axes.unicode_minus': False, 'text.color': INK, 'axes.labelcolor': INK, 'xtick.color': MUTED, 'ytick.color': MUTED})
    parts, transforms, before, after, model, report = load_geometry(args.source)
    mesh_parts = [p for p in parts if p.filename != 'primitive']
    report['current_mesh_counts'] = {'instances': len(mesh_parts),
        'original_vertices': sum(len(p.original_vertices) for p in mesh_parts),
        'vertices': sum(len(p.vertices) for p in mesh_parts),
        'original_triangles': sum(len(p.original_faces) for p in mesh_parts),
        'triangles': sum(len(p.faces) for p in mesh_parts)}
    report['sphere_totals'] = {'before': sum(len(r) for _, r in before.values()), 'after': sum(len(r) for _, r in after.values())}
    report['gpu_bodies'] = len(model.bodies)
    report['gpu_support_vertices'] = sum(len(b.vertices) for b in model.bodies)
    manifest_path = ROOT / 'ros2_ws/src/cleany_description/meshes/collision/arm_meshes.json'
    report['manifest'] = json.loads(manifest_path.read_text())
    for name, metadata in report['manifest']['meshes'].items():
        for subdir, key in (('', 'source_sha256'), ('collision/', 'output_sha256')):
            path = manifest_path.parent.parent / subdir / name
            assert hashlib.sha256(path.read_bytes()).hexdigest() == metadata[key]
    acquisition = json.loads((args.source / 'acquisition.json').read_text())
    report.update(environment=platform.platform(), python=sys.version,
        rendering_versions={name: importlib.metadata.version(name) for name in ('numpy', 'scipy', 'matplotlib', 'trimesh', 'plotly')},
        local_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        acquisition=acquisition, source=str(args.source.resolve()),
        urdf_sha256=hashlib.sha256((args.source / 'current.urdf').read_bytes()).hexdigest(),
        comparison='Same current URDF; original collision STL paths substituted only for the baseline.',
        pose_source='test_head_collision_runtime.state reference values; no live joints or actuation.',
        cache_versions={'trimesh': '4.5.3', 'nvidia_curobo': '0.7.8',
            'fitting': json.loads((args.source / 'fitting.json').read_text()) if (args.source / 'fitting.json').exists()
                else {'schema': 2, 'spheres_per_mesh': 8, 'padding_m': .002, 'budget_per_link': 8}},
        validation=[f"All {2 * len(report['manifest']['meshes'])} source/output STL SHA256 digests match the manifest",
                    'Every fitted-cache array checksum verified',
                    'Every pre-merge sphere enclosed by a post-merge sphere',
                    'All spheres match checksum-verified full model cache',
                    'All URDF links have a connected rigid FK transform'],
        failures=acquisition['failures'],
        limitations=['Static reference pose; no live motion or new speed/collision benchmark',
                     'Spheres and primitives are tessellated only for display; cached geometry is unchanged',
                     'STL counts exclude primitive vertices; GPU counts include primitive envelopes'])
    (args.output / 'provenance.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps({k: report[k] for k in ('current_mesh_counts', 'sphere_totals', 'gpu_bodies', 'gpu_support_vertices')}, indent=2), flush=True)
    for function, values in ((render_overview, (parts, transforms, before, after, model, report, args.output)),
                             (render_mesh_details, (parts, args.output)),
                             (render_spheres, (parts, transforms, before, after, args.output)),
                             (render_counts, (report, args.output)),
                             (render_all_meshes, (parts, args.output))):
        function(*values)
        print(function.__name__, 'complete', flush=True)
    if args.baseline_cache and args.baseline_meshes:
        render_fix_comparison(parts, transforms, after, model, args.baseline_cache,
                              args.baseline_meshes, args.output)
    if args.baseline_meshes:
        render_angular_comparison(parts, args.baseline_meshes, args.output)
    interactive(parts, transforms, before, after, model, args.output,
                bool(args.baseline_cache and args.baseline_meshes), args.sphere_review, args.sphere_fit)


def render_all_meshes(parts, output):
    unique = {part.filename: part for part in reversed(parts) if part.filename != 'primitive'}
    selected = list(reversed(list(unique.values())))
    for start in range(0, len(selected), 3):
        rows = selected[start:start + 3]
        fig, axes = plt.subplots(len(rows), 3, figsize=(18, max(8, 4 * len(rows) + 2)), squeeze=False)
        fig.subplots_adjust(left=.04, right=.98, bottom=.06, top=.86, hspace=.45, wspace=.12)
        header(fig, f'전체 충돌 부품 | {start + 1}–{start + len(rows)} / {len(selected)}',
               '원본 CAD → MoveIt 평면 단순화 → GPU 외피 · 같은 행은 동일 카메라와 축척 · 실제 삼각형 표시')
        for row, part in enumerate(rows):
            name = Path(part.filename).name
            support = part.support_vertices
            if support is None:
                # Base-link mesh bodies are available only through the full model.
                raise ValueError(f'Missing GPU geometry: {part.filename}')
            variants = [(part.original_vertices, part.original_faces, OLD, '원본'),
                        (part.vertices, part.faces, NEW, 'MoveIt'),
                        (support, ConvexHull(support).simplices, HULL, 'GPU')]
            view = camera(-45, 24)
            limits = bounds(np.vstack([item[0] for item in variants]), view, .12)
            for col, (vertices, faces, color, label) in enumerate(variants):
                axis(axes[row, col], limits, f'{name} · {label}', f'{len(vertices):,} 정점 / {len(faces):,} 삼각형')
                surface(axes[row, col], [(vertices, faces)], view, tint=color, edges=True, linewidth=.12)
        save(fig, output, f'all-parts-{start // 3 + 1:02d}.png')


def render_angular_comparison(parts, baseline_meshes, output):
    names = [('Upper_Arm_5cm.stl', '상완'), ('Lower_Arm_5cm.stl', '하완'),
             ('Rotation_Pitch.stl', '어깨 회전 연결부'), ('Wrist_Pitch_Roll.stl', '손목 연결부')]
    fig, axes = plt.subplots(4, 2, figsize=(16, 17))
    fig.subplots_adjust(left=.05, right=.97, bottom=.05, top=.85, hspace=.42, wspace=.15)
    header(fig, '전체 평면화 | 이전 적용 상태와 이번 수정', '불필요한 곡면을 평면으로 축소 · 하완 포크와 연결부는 분할 · 같은 행은 동일 축척')
    for row, (filename, label) in enumerate(names):
        part = next(p for p in parts if p.filename.endswith('/collision/' + filename))
        path = baseline_meshes / filename
        old = read_stl(path) if path.exists() else SimpleNamespace(vertices=part.original_vertices, faces=part.original_faces)
        view = camera(-45, 24)
        limits = bounds(np.vstack((old.vertices, part.vertices)), view, .12)
        for col, (vertices, faces, color, suffix) in enumerate([
                (old.vertices, old.faces, OLD, '이전'), (part.vertices, part.faces, NEW, '이번 수정')]):
            axis(axes[row, col], limits, f'{label} · {suffix}', f'{len(vertices):,} 정점 / {len(faces):,} 삼각형')
            surface(axes[row, col], [(vertices, faces)], view, tint=color, edges=True, linewidth=.15)
    save(fig, output, '06-angular-before-after.png')


def render_fix_comparison(parts, transforms, after, model, baseline_cache, baseline_meshes, output):
    baseline = SimpleNamespace()
    assert load_model(baseline_cache, baseline)
    old_spheres = {name: (np.array([s['center'] for s in spheres]),
                           np.array([s['radius'] for s in spheres]))
                   for name, spheres in baseline.spheres.items()}
    fig, axes = plt.subplots(3, 2, figsize=(16, 17))
    fig.subplots_adjust(left=.045, right=.97, bottom=.07, top=.88, hspace=.38, wspace=.15)
    header(fig, '충돌 모델 수정 전 → 수정 후',
           '같은 CAD·자세·축척 · 구의 형상 누락 보정 · 모서리와 GPU 외피 단순화 · 실물 배포 전 검증 결과')
    view = camera(-60, 25)
    predicate = lambda n: n.startswith('right_')
    entries = world_parts(parts, original=True, owner=predicate)
    sphere_sets = [world_spheres(values, transforms, predicate) for values in (old_spheres, after)]
    points = [v for v, _ in entries]
    for c, r in sphere_sets:
        points.extend((c - r[:, None], c + r[:, None]))
    limits = bounds(np.concatenate(points), view, .08)
    for col, spheres in enumerate(sphere_sets):
        axis(axes[0, col], limits, '수정 전 · 표면점 기반 구' if col == 0 else '수정 후 · 부피를 감싸는 구',
             f'오른팔 {len(spheres[1])}개 · ' + ('팔 중간 형상 누락' if col == 0 else '원본 CAD 정점·표면 표본 누락 0'))
        surface(axes[0, col], entries, view, tint='#b5c1cc')
        sphere_overlay(axes[0, col], spheres, view, tint=BEFORE_BALL if col == 0 else BALL, fill=.15)
    name = 'Base_Motor.stl'
    old = read_stl(baseline_meshes / name)
    new = next(p for p in parts if p.filename.endswith('/collision/' + name))
    view = camera(-50, 27)
    limits = bounds(np.vstack((old.vertices, new.vertices)), view, .12)
    for col, (vertices, faces) in enumerate(((old.vertices, old.faces), (new.vertices, new.faces))):
        axis(axes[1, col], limits, f"모터 외피 · {'수정 전' if col == 0 else '수정 후'}",
             f'{len(vertices)} 정점 / {len(faces)} 삼각형')
        surface(axes[1, col], [(vertices, faces)], view, tint=OLD if col == 0 else NEW, edges=True, linewidth=.4)
    owner = 'right_lower_arm_link'
    bodies = [next(b for b in m.bodies if b.owner == owner) for m in (baseline, model)]
    part = next(p for p in parts if p.owner == owner and p.filename.endswith('/Lower_Arm_5cm.stl'))
    mesh_frame = np.linalg.inv(part.transform) @ transforms[owner]
    vertices_in_mesh = [moved(body.vertices, mesh_frame) for body in bodies]
    view = camera(-45, 24)
    limits = bounds(np.concatenate(vertices_in_mesh), view, .12)
    for col, (body, vertices) in enumerate(zip(bodies, vertices_in_mesh)):
        axis(axes[2, col], limits, f"하완 GPU 외피 · {'수정 전' if col == 0 else '수정 후'}",
             f'{len(body.vertices):,} 정점 / {len(body.planes):,} 검사 평면')
        surface(axes[2, col], [(vertices, ConvexHull(vertices).simplices)], view,
                tint=OLD if col == 0 else HULL, edges=True, linewidth=.25)
    fig.text(.045, .035, '외접 상한과 검사 결과는 설정 및 검증 집계를 참고하세요. 개수 감소는 실측 속도 개선율이 아닙니다.', fontsize=11)
    save(fig, output, '05-before-after.png')


if __name__ == '__main__':
    main()
