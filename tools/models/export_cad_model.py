#!/usr/bin/env python3
"""저장된 Cleany 조립체를 공통 ROS 및 MuJoCo 형상으로 내보냅니다.

numpy, scipy와 cadquery-ocp가 필요하며 FreeCAD 문서 코드는 실행하지 않습니다.
CAD 자세는 등록 자세이며 엔코더 보정이 아닙니다. 정적 조립체에서 pincer linkage
닫힘을 구할 수 없으므로 cam 관절은 잠근 상태로 유지합니다.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.TopLoc import TopLoc_Location

from cad_inventory import inventory, read_shape

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from optimize_arm_collision_meshes import enclosing_hull, read_stl, stl_bytes

X = '{http://www.ros.org/wiki/xacro}'
ET.register_namespace('xacro', X[1:-1])
CAD_R = Rotation.from_euler('z', np.pi / 2).as_matrix()
# 기존 차체 정지 높이와 바퀴 접촉 규약을 유지합니다.
CAD_T = np.array([-0.000025, 0.000027, -0.284497])


def values(a):
    return ' '.join(f'{x:.15g}' for x in np.asarray(a).flatten())


def pose(xyz=(0, 0, 0), rotation=None):
    t = np.eye(4)
    t[:3, 3] = xyz
    if rotation is not None:
        t[:3, :3] = rotation
    return t


def origin(joint):
    o = joint.find('origin')
    angles = np.fromstring(o.get('rpy', '0 0 0'), sep=' ')
    snapped = np.round(angles / (np.pi/2)) * (np.pi/2)
    angles = np.where(abs(angles-snapped) < 1e-10, snapped, angles)
    return pose(np.fromstring(o.get('xyz', '0 0 0'), sep=' '),
                Rotation.from_euler('xyz', angles).as_matrix())


def fk(robot, angles):
    transforms = {'base_link': np.eye(4)}
    remaining = list(robot.findall('joint'))
    while remaining:
        ready = [j for j in remaining if j.find('parent').get('link') in transforms]
        if not ready:
            raise ValueError('Disconnected robot tree')
        for j in ready:
            t = origin(j)
            if j.get('type') != 'fixed':
                axis = np.fromstring(j.find('axis').get('xyz'), sep=' ')
                t = t @ pose(rotation=Rotation.from_rotvec(axis * angles.get(j.get('name'), 0)).as_matrix())
            transforms[j.find('child').get('link')] = transforms[j.find('parent').get('link')] @ t
            remaining.remove(j)
    return transforms


def set_origin(joint, transform):
    joint.find('origin').set('xyz', values(transform[:3, 3]))
    joint.find('origin').set('rpy', values(Rotation.from_matrix(transform[:3, :3]).as_euler('xyz')))


def placement(record):
    p = record['placement']
    return pose([float(p['P' + k]) for k in 'xyz'],
                Rotation.from_quat([float(p['Q' + str(i)]) for i in range(4)]).as_matrix())


def cad_point(point):
    return CAD_R @ (np.asarray(point) * .001) + CAD_T


def center(record):
    b = np.asarray(record['bounds_mm'])
    return (b[:3] + b[3:]) / 2


def triangles(shape):
    BRepMesh_IncrementalMesh(shape, .15, False, .3, False).Perform()
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    result = []
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        location = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, location)
        if tri is not None:
            vertices = np.array([list(tri.Node(i).Transformed(location.Transformation()).Coord())
                                 for i in range(1, tri.NbNodes() + 1)])
            for i in range(1, tri.NbTriangles() + 1):
                ids = np.array(tri.Triangle(i).Get()) - 1
                if face.Orientation() == TopAbs_REVERSED:
                    ids = ids[[0, 2, 1]]
                result.append(vertices[ids])
        explorer.Next()
    return np.asarray(result)


def write_stl(path, vertices):
    normals = np.cross(vertices[:, 1] - vertices[:, 0], vertices[:, 2] - vertices[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    keep = lengths > 0
    vertices, normals = vertices[keep], normals[keep] / lengths[keep, None]
    data = np.zeros(len(vertices), dtype=[('n', '<f4', (3,)), ('v', '<f4', (3, 3)), ('a', '<u2')])
    data['n'], data['v'] = normals, vertices
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b'Cleany CAD export'.ljust(80, b'\0') + struct.pack('<I', len(data)) + data.tobytes())


def color(name):
    if name.startswith('Drive_') and name.endswith('_wheel'):
        return [.1, .11, .12, 1]
    if name.startswith('Fomex') and 'Tool' not in name:
        return [.88, .9, .91, 1]
    if 'VirtualDisplay' in name:
        return [.025, .035, .04, 1]
    if any(k in name.lower() for k in ['servo', 'camera', 'motor', 'tube', 'lens']):
        return [.12, .13, .15, 1]
    if any(k in name for k in ['Print', 'Socket', 'Mount', 'Bracket', 'Wrist']):
        return [.2, .36, .55, 1]
    return [.6, .63, .66, 1]


def build_registration(robot, records):
    """기존 팔 각도 부호를 유지하며 혼 쌍에 관절축을 등록합니다."""
    joints = {j.get('name'): j for j in robot.findall('joint')}
    angles = {}
    anchors = {}
    for side, suffix in [('right', ''), ('left', '001')]:
        s = 1 if side == 'left' else -1
        base = records['BasePrint' + suffix]
        # 기존 체인의 베이스 기준면과 16.5mm yaw 원점 규약을 따릅니다.
        yaw_point = center(records['YawDriveHorn' + suffix])
        yaw_point[2] = float(base['placement']['Pz']) + 16.5
        yaw = cad_point(yaw_point)
        joints[f'{side}_shoulder_yaw_joint'].find('origin').set('xyz', values(yaw))
        for key, horn in [('upper_arm_link', 'UpperDriveHorn'), ('lower_arm_link', 'Aligned_LowerDriveHorn'),
                          ('wrist_pitch_link', 'WristDriveHorn')]:
            p = center(records[horn + suffix])
            p[1] = -120
            anchors[f'{side}_{key}'] = cad_point(p)
        tool = center(records[f'{side.title()}Kinematic_CameraTool180_ToolDriveHorn'])
        anchors[f'{side}_gripper_frame'] = cad_point(tool)
        names = [f'{side}_{n}_joint' for n in ['shoulder_pitch', 'elbow_pitch', 'wrist_pitch']]
        def residual(q):
            frames = fk(robot, dict(zip(names, q)))
            return np.r_[
                (frames[f'{side}_lower_arm_link'][:3, 3] - anchors[f'{side}_lower_arm_link']) * 100,
                (frames[f'{side}_wrist_pitch_link'][:3, 3] - anchors[f'{side}_wrist_pitch_link']) * 100,
                frames[f'{side}_gripper_frame'][:3, :3] @ [0, -1, 0] - [0, s, 0]]
        fitted = least_squares(residual, [3.14, 3.14, 0], max_nfev=1000)
        if np.linalg.norm(fitted.fun) > .01:
            raise ValueError(f'CAD arm registration failed: {side}: {fitted.fun}')
        angles.update(dict(zip(names, fitted.x)))
        # 정확한 CAD 회전축으로 반올림 및 바뀐 손목 장착 오프셋을 제거합니다.
        for joint_name, link in zip(names + [f'{side}_wrist_roll_joint'],
                                   [f'{side}_{n}' for n in ['upper_arm_link', 'lower_arm_link', 'wrist_pitch_link', 'gripper_frame']]):
            j = joints[joint_name]
            parent = fk(robot, angles)[j.find('parent').get('link')]
            local = np.linalg.inv(parent) @ np.r_[anchors[link], 1]
            j.find('origin').set('xyz', values(local[:3]))
        # pincer linkage는 고정 CAD snapshot이며 이전 hinged jaw가 아닙니다.
        j = joints[f'{side}_gripper_joint']
        j.find('limit').set('lower', '0')
        j.find('limit').set('upper', '0')
        frames = fk(robot, angles)
        p = center(records[f'{side.title()}Kinematic_CameraTool180_Pinc_CamHorn'])
        t = frames[f'{side}_moving_jaw_link'].copy()
        t[:3, 3] = cad_point(p)
        set_origin(j, np.linalg.inv(frames[f'{side}_gripper_frame']) @ t)
        tips = [center(records[f'{side.title()}Kinematic_CameraTool180_Pinc_Removable_Tip_{n}']) for n in ['L', 'R']]
        tcp = np.linalg.inv(frames[f'{side}_gripper_frame']) @ np.r_[cad_point(np.mean(tips, axis=0)), 1]
        joints[f'{side}_grasp_tcp_joint'].find('origin').set('xyz', values(tcp[:3]))
    # CAD 헤드는 0에서 전방을 향하며 pan 원점은 tilt 축 높이에 있습니다.
    pan = center(records['HeadPanDriveHorn'])
    tilt = center(records['HeadTiltDriveHorn'])
    pan[2] = tilt[2]
    tilt[0] = pan[0]
    set_origin(joints['top_base_joint'], pose())
    set_origin(joints['head_pan_joint'], pose(cad_point(pan)))
    set_origin(joints['head_tilt_joint'], pose(CAD_R @ ((tilt - pan) * .001)))
    # CAD 검증과 동일하게 문서화된 명목 D435 imager 오프셋을 사용합니다.
    camera = np.array([.11867774043964463, .016892537637999085, 1.0217826016167113]) + CAD_T
    set_origin(joints['head_camera_joint'], pose(camera - cad_point(tilt)))
    return angles, anchors


def link_for(name, parents):
    for side in ['Left', 'Right']:
        if side + 'Arm' in parents:
            group = parents[-1].removeprefix(side)
            link = {'Base':'base_link', 'Yaw':side.lower()+'_shoulder_yaw_link',
                    'Upper':side.lower()+'_upper_arm_link', 'Lower':side.lower()+'_lower_arm_link',
                    'Wrist':side.lower()+'_wrist_pitch_link', 'Tool':side.lower()+'_gripper_frame',
                    'CameraComponents':side.lower()+'_gripper_frame'}[group]
            if name.endswith(('_Pinc_Cam', '_Pinc_CamHorn')):
                link = side.lower() + '_moving_jaw_link'
            return link
    if name in ['InstalledMast', 'InstalledMountBracket', 'HeadFixedBracket', 'HeadPanServoCase',
                'HeadPanServoFront', 'HeadPanServoMiddle']:
        return 'top_base_link'
    if name in ['HeadPanBracket', 'HeadPanDriveHorn', 'HeadPanIdlerHorn', 'HeadTiltServoCase',
                'HeadTiltServoFront', 'HeadTiltServoMiddle']:
        return 'head_pan_link'
    if 'HeadRig' in parents:
        return 'head_camera_link' if name.startswith('HeadCamera') else 'head_tilt_link'
    if name.startswith('Drive_') and name.endswith('_wheel'):
        # CAD label은 문서의 로봇 축 X=-Y, Y=X보다 먼저 정해진 이름입니다.
        aliases = {'rear_left':'front_right', 'rear_right':'front_left',
                   'front_left':'rear_right', 'front_right':'rear_left'}
        return aliases[name[6:-6]] + '_wheel_link'
    return 'base_link'


def save_xml(path, root):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='unicode', xml_declaration=True)
    with path.open('a') as f:
        f.write('\n')


def convex_mesh(points):
    points = np.unique(points, axis=0)
    hull = ConvexHull(points)
    faces = hull.simplices.copy()
    triangles = points[faces]
    normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    reverse = np.einsum('ij,ij->i', normals, hull.equations[:, :3]) < 0
    faces[reverse] = faces[reverse][:, [0, 2, 1]]
    normals[reverse] *= -1
    lengths = np.linalg.norm(normals, axis=1)
    if np.any(lengths == 0):
        raise ValueError('Degenerate hull triangle')
    normals /= lengths[:, None]
    # float32 반올림의 가느다란 면까지 포함해 실제 삼각형 평면을 검증합니다.
    for start in range(0, len(normals), 128):
        n = normals[start:start+128]
        d = np.einsum('ij,ij->i', n, points[faces[start:start+128, 0]])
        if np.max(points@n.T-d) > 1e-10:
            raise ValueError('Rounded hull triangle is not supporting')
    from optimize_arm_collision_meshes import Mesh
    return Mesh(points, faces)


def export_collision(visual, destination):
    source = read_stl(visual)
    try:
        mesh, certificate = enclosing_hull(
            source, max_expansion=.00025, margin=.0000001,
            max_iterations=128, planes_per_iteration=16)
        mesh = convex_mesh(mesh.vertices)
    except ValueError:
        # 얇은 입체나 반올림으로 생긴 가느다란 면은 정확한 원본 껍질을 유지합니다.
        mesh = convex_mesh(source.vertices)
        certificate = {'max_expansion_from_component_hull_m': 0.0,
                       'method': 'exact_convex_hull'}
        print('Exact hull retained:', visual.name, flush=True)
    certificate['triangles'] = len(mesh.faces)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(stl_bytes(mesh))
    return certificate


def rebuild_collision(package):
    path = package/'config/geometry/cad_final.json'
    manifest = json.loads(path.read_text())
    for item in manifest['components']:
        destination = package/item['collision_mesh']
        item['collision_certificate'] = export_collision(package/item['mesh'], destination)
        item['collision_sha256'] = hashlib.sha256(destination.read_bytes()).hexdigest()
    path.write_text(json.dumps(manifest, indent=2)+'\n')
    print('Rebuilt enclosing collision hulls; expansion from each convex hull <= 0.25 mm')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--package', type=Path, default=Path('ros2_ws/src/cleany_description'))
    parser.add_argument('--legacy-urdf', type=Path)
    parser.add_argument('--collision-only', action='store_true')
    args = parser.parse_args()
    package = args.package.resolve()
    if args.collision_only:
        rebuild_collision(package)
        return
    if args.legacy_urdf is None:
        parser.error('--legacy-urdf is required for a full export')
    data = inventory(args.source)
    records = {o['name']:o for o in data['objects']}
    robot = ET.parse(args.legacy_urdf).getroot()
    for j in robot.findall('joint'):
        set_origin(j, origin(j))
    angles, anchors = build_registration(robot, records)
    frames = fk(robot, angles)
    links = {l.get('name'):l for l in robot.findall('link')}
    for link in links.values():
        for element in list(link):
            if element.tag in ['visual', 'collision']:
                link.remove(element)
    components = []
    with zipfile.ZipFile(args.source) as archive:
        for record in data['objects']:
            name = record['name']
            link_name = link_for(name, record['parents'])
            shape = read_shape(archive, record['brep'])
            shapes = [shape]
            if name == 'RPLidarA1M8':
                explorer = TopExp_Explorer(shape, TopAbs_SOLID)
                shapes = []
                while explorer.More():
                    shapes.append(explorer.Current())
                    explorer.Next()
            for part, shape in enumerate(shapes):
                raw = triangles(shape)
                if not len(raw):
                    raise ValueError(f'No triangles: {name}')
                # MuJoCo STL decoder는 파일당 삼각형을 최대 200,000개 받습니다.
                for chunk_start in range(0, len(raw), 190000):
                    chunk = raw[chunk_start:chunk_start+190000]
                    key = name if len(shapes) == 1 and len(raw) <= 190000 else f'{name}_{part}_{chunk_start}'
                    world = chunk * .001 @ CAD_R.T + CAD_T
                    t = frames[link_name]
                    local = (world - t[:3, 3]) @ t[:3, :3]
                    mesh = Path('meshes/cad_final') / (key + '.stl')
                    write_stl(package / mesh, local)
                    collision = Path('meshes/cad_final/collision') / (key + '.stl')
                    certificate = export_collision(package/mesh, package/collision)
                    uri = 'package://cleany_description/' + str(mesh)
                    for tag in ['visual', 'collision']:
                        element = ET.SubElement(links[link_name], tag, name='cad_' + key + '_' + tag)
                        ET.SubElement(element, 'origin', xyz='0 0 0', rpy='0 0 0')
                        ET.SubElement(ET.SubElement(element, 'geometry'), 'mesh', filename=uri if tag == 'visual' else 'package://cleany_description/' + str(collision), scale='1 1 1')
                        if tag == 'visual':
                            ET.SubElement(ET.SubElement(element, 'material', name='cad_' + key), 'color', rgba=values(color(name)))
                    components.append(dict(name=name, link=link_name, mesh=str(mesh), brep_sha256=record['sha256'],
                                           mesh_sha256=hashlib.sha256((package/mesh).read_bytes()).hexdigest(),
                                           collision_mesh=str(collision), collision_sha256=hashlib.sha256((package/collision).read_bytes()).hexdigest(),
                                           cad_bounds_mm=record['bounds_mm'], collision_certificate=certificate))
    # 카메라 광학 등록은 이전 v8 데이터가 아닌 렌즈 축에서 구합니다.
    cameras = {}
    for side in ['left', 'right']:
        lens = records[side.title() + 'Camera_Lens']
        p = placement(lens)
        with zipfile.ZipFile(args.source) as archive:
            glass = triangles(read_shape(archive, records[side.title()+'Camera_Glass']['brep'])).reshape(-1,3)
        glass_local = (glass-p[:3,3]) @ p[:3,:3]
        # 모델링한 전면 유리 바로 바깥의 명목 렌더링 원점입니다.
        lens_center = p @ np.array([0, 0, glass_local[:,2].max()+.1, 1])
        cv = pose(cad_point(lens_center[:3]), CAD_R @ p[:3, :3])
        cameras[side] = (np.linalg.inv(frames[side + '_gripper_frame']) @ cv).tolist()
    manifest = dict(schema_version=1, profile='cad_final', source_name=args.source.name,
                    source_sha256=data['source_sha256'], cad_to_base=pose(CAD_T, CAD_R).tolist(),
                    registration_angles_rad=angles, anchors_base_m={k:v.tolist() for k,v in anchors.items()},
                    wrist_cameras=cameras, components=components,
                    boundary='Layout-draft CAD; inherited mass/inertia estimates; pincer linkage locked. Not a measured hardware calibration.')
    path = package/'config/geometry/cad_final.json'
    path.write_text(json.dumps(manifest, indent=2) + '\n')
    write_xacro(package, robot)
    write_mjcf(package, robot, cameras)
    write_wrist_config(package.parent/'cleany_mujoco_sim/config/sorting_wrist_cameras.yaml', cameras, data['source_sha256'])
    print(f'Exported {len(components)} components; CAD registration angles: {angles}')


def write_xacro(package, robot):
    root = ET.Element('robot')
    for macro_name, include_head in [('cleany_robot_geometry',False), ('cleany_head_camera',True)]:
        params = 'calibration_file:=\'\'' if include_head else 'include_wheel_joints:=false head_mast_rear_gap_m:=0.05'
        macro = ET.SubElement(root, X+'macro', name=macro_name, params=params)
        if include_head:
            guard = ET.SubElement(macro, X+'if', value='${bool(calibration_file)}')
            guard.text = "${xacro.fatal('cad_final requires new camera calibration; use model_profile:=jetson_20261009 for the old measured assembly')}"
        for element in robot:
            if element.tag not in ['link', 'joint']:
                continue
            name = element.get('name')
            is_head = name.startswith('head_')
            if is_head != include_head:
                continue
            e = deepcopy(element)
            if e.tag == 'joint' and name.endswith('_wheel_joint'):
                e.set('type', "${'continuous' if include_wheel_joints else 'fixed'}")
            macro.append(e)
    save_xml(package/'urdf/cad_final/geometry.xacro', root)


def write_wrist_config(path, cameras, source_sha):
    """시뮬레이션 TF publisher에도 같은 명목 CAD 등록을 적용합니다."""
    lines = ['# 명목 CAD 렌즈 원점이며 측정된 손-눈 보정값이 아닙니다.',
             'source: simulation_nominal_cad', 'cad_sha256: ' + source_sha]
    for side, matrix in cameras.items():
        t = np.asarray(matrix)
        q = Rotation.from_matrix(t[:3, :3]).as_quat()
        lines += [side + ':', '  parent_frame: ' + side + '_gripper_frame',
                  '  child_frame: ' + side + '_wrist_rgb_optical_frame',
                  '  translation_m: ' + json.dumps(t[:3, 3].tolist()),
                  '  quaternion_xyzw: ' + json.dumps(q.tolist())]
    path.write_text('\n'.join(lines) + '\n')


def write_mjcf(package, robot, cameras):
    old = ET.parse(package/'mjcf/cleany_jetson_20261009.xml').getroot()
    root = ET.Element('mujoco', model='cleany_cad_final')
    root.append(deepcopy(old.find('compiler')))
    asset = ET.SubElement(root, 'asset')
    root.append(deepcopy(old.find('default')))
    # 공통 기본값은 롤러 접촉 외형에 이 재질들을 계속 참조합니다.
    for m in old.findall('asset/material'):
        asset.append(deepcopy(m))
    world = ET.SubElement(root, 'worldbody')
    world.append(deepcopy(old.find('worldbody/light')))
    chassis = ET.SubElement(world, 'body', name='chassis', pos='0 0 0.38')
    ET.SubElement(chassis, 'freejoint')
    bodies = {'base_link':chassis}
    links = {l.get('name'):l for l in robot.findall('link')}
    mapping = {'base_link':'chassis'}
    for side, suffix in [('left',''), ('right','_2')]:
        for link, body in [('shoulder_yaw_link','Rotation_Pitch'), ('upper_arm_link','Upper_Arm'),
                           ('lower_arm_link','Lower_Arm'), ('wrist_pitch_link','Wrist_Pitch_Roll'),
                           ('gripper_frame','Fixed_Jaw'), ('moving_jaw_link','Moving_Jaw')]:
            mapping[side+'_'+link] = body+suffix
    pending = list(robot.findall('joint'))
    while pending:
        for j in pending[:]:
            parent, child = j.find('parent').get('link'), j.find('child').get('link')
            if parent not in bodies:
                continue
            if child.endswith('_wheel_link'):
                name = child.removesuffix('_link')
                wheel = deepcopy(old.find(f'.//body[@name="{name}"]'))
                # 검증된 롤러 접촉 모델과 총질량 500g을 유지합니다.
                for geom in wheel.findall('geom'):
                    wheel.remove(geom)
                bodies[parent].append(wheel)
                bodies[child] = wheel
            else:
                t = origin(j)
                q = Rotation.from_matrix(t[:3,:3]).as_quat()
                body = ET.SubElement(bodies[parent], 'body', name=mapping.get(child,child),
                                     pos=values(t[:3,3]), quat=values(q[[3,0,1,2]]))
                bodies[child] = body
                if j.get('type') != 'fixed':
                    limit = j.find('limit')
                    limits = [float(limit.get(k)) for k in ['lower','upper']]
                    if limits == [0,0]:
                        limits = [-1e-9,1e-9]
                    cls = 'sts3250' if any(k in j.get('name') for k in ['shoulder_pitch','elbow_pitch']) else 'sts3215'
                    ET.SubElement(body,'joint',name=j.get('name'),type='hinge',axis=j.find('axis').get('xyz'),
                                  range=values(limits),attrib={'class':cls})
            pending.remove(j)
    for link_name, body in bodies.items():
        link = links[link_name]
        inertial = link.find('inertial')
        if inertial is not None and not link_name.endswith('_wheel_link'):
            a = inertial.find('inertia').attrib
            ET.SubElement(body, 'inertial', mass=inertial.find('mass').get('value'),
                          pos=inertial.find('origin').get('xyz'),
                          fullinertia=' '.join(a[k] for k in ['ixx','iyy','izz','ixy','ixz','iyz']))
        for visual in link.findall('visual'):
            meshfile = visual.find('geometry/mesh').get('filename').split('/meshes/')[1]
            meshname = 'cad_' + Path(meshfile).stem
            ET.SubElement(asset, 'mesh', name=meshname, file=meshfile)
            rotation = np.eye(3)
            if link_name.endswith('_wheel_link'):
                # 바퀴 body 축은 +Z이며 URDF 축은 +Y입니다.
                rotation = Rotation.from_euler('x', np.pi/2).as_matrix()
            q = Rotation.from_matrix(rotation).as_quat()[[3,0,1,2]]
            ET.SubElement(body, 'geom', name=visual.get('name'), type='mesh', mesh=meshname,
                          quat=values(q), rgba=visual.find('material/color').get('rgba'), attrib={'class':'visual'})
            if not link_name.endswith('_wheel_link'):
                collision_name = 'collision_' + meshname
                ET.SubElement(asset, 'mesh', name=collision_name, file='cad_final/collision/' + Path(meshfile).name)
                geom = ET.SubElement(body, 'geom', name=visual.get('name').replace('_visual','_collision'),
                                     type='mesh', mesh=collision_name, attrib={'class':'collision'})
                if any(meshname.endswith('Kinematic_CameraTool180_Pinc_' + part) for part in
                       ['Removable_Tip_L', 'Tip_Support_L', 'Removable_Tip_R', 'Tip_Support_R', 'Distal_Shell_R']):
                    # 시뮬레이션 접촉 재질을 유지하며 측정된 계수는 아닙니다.
                    geom.set('friction', '3 .2 .1')
                    geom.set('condim', '6')
        if link_name.endswith('_grasp_tcp'):
            ET.SubElement(body, 'site', name=link_name, size='.005', rgba='0 0.8 1 0.4')
    for side, matrix in cameras.items():
        t = np.asarray(matrix)
        q = Rotation.from_matrix(t[:3,:3]).as_quat()[[3,0,1,2]]
        cam = ET.SubElement(bodies[side+'_gripper_frame'],'body',name=side+'_wrist_rgb_optical_frame',pos=values(t[:3,3]),quat=values(q))
        ET.SubElement(cam,'site',name=side+'_wrist_rgb_optical_frame',size='.005',rgba='1 0 0 0')
        ET.SubElement(cam,'camera',name=side+'_wrist_rgb',quat='0 1 0 0',fovy='93')
    for kind, fovy in [('rgb','42'), ('depth','40')]:
        name = f'head_camera_{kind}_optical_frame'
        ET.SubElement(bodies[name],'site',name=name,size='.005',rgba='0 1 0 0')
        ET.SubElement(bodies[name],'camera',name='head_realsense_'+kind,quat='0 1 0 0',fovy=fovy)
    ET.SubElement(chassis,'site',name='lidar_site',pos='0.18 0 0.08',size='.02',rgba='0 1 0 .5')
    contact = ET.SubElement(root,'contact')
    # 여전히 존재하는 body의 제외만 유지합니다. 시뮬레이션 근사입니다.
    names = {b.get('name') for b in root.findall('.//body')}
    for e in old.findall('contact/exclude'):
        if e.get('body1') in names and e.get('body2') in names:
            contact.append(deepcopy(e))
    actuators = deepcopy(old.find('actuator'))
    for a in actuators:
        if a.get('joint','').endswith('_gripper_joint'):
            a.set('ctrlrange','-1e-9 1e-9')
    root.append(actuators)
    save_xml(package/'mjcf/cleany.xml', root)


if __name__ == '__main__':
    main()
