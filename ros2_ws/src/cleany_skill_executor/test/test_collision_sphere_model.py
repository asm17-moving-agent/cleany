from dataclasses import replace
import json
from types import SimpleNamespace

import pytest

from cleany_skill_executor import collision_sphere_model as fitted
from cleany_skill_executor.curobo_model import CuRoboConfig
from cleany_skill_executor.robot_geometry_cache import model_cache_path


@pytest.fixture
def certified_model(tmp_path, monkeypatch):
    mesh = tmp_path / 'mesh.stl'
    mesh.write_bytes(b'certified geometry')
    xml = '<robot><link name="arm"><collision><geometry><mesh filename="package://robot/mesh.stl"/></geometry></collision></link></robot>'
    resolve = lambda _: mesh
    signature = fitted.collision_signatures(xml, {'arm'}, resolve)['arm']
    path = tmp_path / 'spheres.json'
    asset = {'schema': 2, 'model': 'joint_simplified', 'joint_covers': [], 'padding_m': .0005,
             'coverage_certificate_sha256': 'certificate',
             'links': {'arm': {'collision_sha256': signature,
                       'spheres': [{'center': [.01, 0., 0.], 'radius': .02}]},
                       'head': {'collision_sha256': 'unused-head-profile', 'spheres': []}}}
    path.write_text(json.dumps(asset))
    monkeypatch.setattr(fitted, 'MODEL_PATH', path)
    model = SimpleNamespace(xml=xml, config=CuRoboConfig(), ignore={'arm': []},
        spheres={'arm': [{'center': [0., 0., 0.], 'radius': .1}]}, bodies=['complete-body'],
        sphere_reduction={'before': 64, 'after': 16,
            'links': {'arm': {'before': 64, 'after': 16, 'cover_error_m': 0.}},
            'validation_geometry_sha256': 'unchanged-body-geometry'})
    return model, mesh, path, resolve


def test_certified_subset_replaces_proposal_only_and_records_source(certified_model):
    model, _, _, resolve = certified_model
    fitted.apply_sphere_model(model, resolve=resolve)
    assert model.spheres['arm'] == [{'center': [.01, 0., 0.], 'radius': .02}]
    assert model.bodies == ['complete-body']
    assert model.sphere_reduction['after'] == 1
    assert model.sphere_reduction['method'] == 'joint_simplified'
    assert model.sphere_reduction['validation_geometry_sha256'] == 'unchanged-body-geometry'
    assert 'cover_error_m' not in model.sphere_reduction['links']['arm']


@pytest.mark.parametrize('change', ['mesh', 'scale', 'origin', 'primitive', 'padding'])
def test_changed_geometry_or_padding_cannot_reuse_certified_cover(certified_model, change):
    model, mesh, _, resolve = certified_model
    if change == 'mesh':
        mesh.write_bytes(b'different geometry')
    elif change == 'scale':
        model.xml = model.xml.replace('/mesh.stl"', '/mesh.stl" scale="2 1 1"')
    elif change == 'origin':
        model.xml = model.xml.replace('<collision>', '<collision><origin xyz="0.01 0 0"/>')
    elif change == 'primitive':
        model.xml = model.xml.replace('</link>', '<collision><geometry><box size=".01 .02 .03"/></geometry></collision></link>')
    else:
        model.config = replace(model.config, sphere_surface_radius_m=.002)
    before = model.spheres
    with pytest.raises(ValueError, match='mismatch|padding'):
        fitted.apply_sphere_model(model, resolve=resolve)
    assert model.spheres is before


def test_asset_contents_invalidate_complete_geometry_cache(certified_model, monkeypatch, tmp_path):
    model, mesh, path, _ = certified_model
    monkeypatch.setattr('importlib.metadata.version', lambda _: 'test-v1')
    xml = model.xml.replace('package://robot/mesh.stl', str(mesh))
    config = replace(model.config, geometry_cache_directory=str(tmp_path))
    first = model_cache_path(xml, '<robot/>', config)
    data = json.loads(path.read_text())
    data['links']['arm']['spheres'][0]['radius'] = .03
    path.write_text(json.dumps(data))
    assert first != model_cache_path(xml, '<robot/>', config)
    assert first != model_cache_path(xml, '<robot/>', replace(config, sphere_model='generated'))


def test_generated_mode_is_explicit_and_does_not_load_asset(certified_model):
    model, _, path, resolve = certified_model
    model.config = replace(model.config, sphere_model='generated')
    path.unlink()
    fitted.apply_sphere_model(model, resolve=resolve)
    assert model.spheres['arm'][0]['radius'] == .1


def test_invalid_sphere_asset_and_unknown_model_are_rejected(certified_model):
    model, _, path, resolve = certified_model
    data = json.loads(path.read_text())
    data['links']['arm']['spheres'][0]['radius'] = -1.
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='Invalid certified'):
        fitted.apply_sphere_model(model, resolve=resolve)
    with pytest.raises(ValueError, match='Unknown collision sphere'):
        CuRoboConfig(sphere_model='unknown')


def test_joint_ball_stays_at_child_origin_for_every_angle():
    import numpy as np
    import xml.etree.ElementTree as ET
    from cleany_skill_executor.core.urdf_fk import UrdfChain

    xml = '''<robot><link name="parent"/><link name="child"/>
      <joint name="joint" type="revolute"><parent link="parent"/><child link="child"/>
        <origin xyz=".1 .2 .3" rpy=".3 -.4 .7"/><axis xyz="1 0 0"/>
      </joint></robot>'''
    joint = ET.fromstring(xml).find('joint')
    asset = {'links': {'parent': {'spheres': [{'center': [.1, .2, .3], 'radius': .04}]}},
        'joint_covers': [{'joint': 'joint', 'parent': 'parent', 'child': 'child',
            'sphere_index': 0, 'radius_m': .04, 'joint_sha256': fitted.joint_signature(joint)}]}
    fitted.validate_joint_covers(xml, asset)
    chain = UrdfChain(xml, 'parent', 'child')
    points = np.array([[.01, .02, .015], [-.03, .01, 0.]])
    for angle in np.linspace(-np.pi, np.pi, 41):
        p, r = chain.pose({'joint': angle})
        np.testing.assert_allclose(np.linalg.norm(points @ r.T + p - [.1, .2, .3], axis=1),
                                   np.linalg.norm(points, axis=1), atol=1e-12)
    for modified in (xml.replace('.1 .2 .3', '.1 .2 .31'),
                     xml.replace('xyz="1 0 0"', 'xyz="0 1 0"')):
        with pytest.raises(ValueError, match='joint cover geometry mismatch'):
            fitted.validate_joint_covers(modified, asset)
    asset['links']['parent']['spheres'][0]['center'][0] += .01
    with pytest.raises(ValueError, match='not centered'):
        fitted.validate_joint_covers(xml, asset)


def test_production_asset_has_separate_shoulder_and_elbow_housing_balls():
    data = json.loads(fitted.MODEL_PATH.read_text())
    assert data['model'] == 'joint_simplified'
    assert 'top_base_link' not in data['links']
    assert sum(len(entry['spheres']) for entry in data['links'].values()) == 118
    assert data['joint_covers'] == []
    assert {cover['link'] for cover in data['rigid_covers']} == {
        f'{side}_{link}' for side in ('left', 'right')
        for link in ('shoulder_yaw_link', 'upper_arm_link', 'lower_arm_link', 'gripper_frame')} | {
            'head_pan_link', 'head_tilt_link'}
    assert len(data['rigid_covers']) == 21
    assert {(c['region'], c['sphere_index']) for c in data['rigid_covers']
            if c['link'].startswith('head_')} == {
                ('head_pan', 0), ('head_tilt', 0), ('head_camera', 1)}
    for side in ('left', 'right'):
        assert len(data['links'][f'{side}_shoulder_yaw_link']['spheres']) == 1
        for link in ('upper_arm_link', 'lower_arm_link'):
            covers = [c for c in data['rigid_covers'] if c['link'] == f'{side}_{link}'
                      and c['region'].startswith('elbow_')]
            assert {c['sphere_index'] for c in covers} == {0, 1}
            assert {c['sphere_count'] for c in covers} == {2}
        covers = [c for c in data['rigid_covers'] if c['link'] == f'{side}_upper_arm_link'
                  and c['region'] == 'shoulder_upper_housing']
        assert {c['sphere_index'] for c in covers} == {2, 3}
        assert len(data['links'][f'{side}_upper_arm_link']['spheres']) == 6


@pytest.mark.parametrize('side', ['left', 'right'])
def test_separate_elbow_balls_cover_their_housings_and_follow_the_owning_link(side):
    import numpy as np
    from pathlib import Path
    import trimesh
    import xml.etree.ElementTree as ET
    xacro = pytest.importorskip('xacro')
    packages = pytest.importorskip('ament_index_python.packages')
    from cleany_skill_executor.core.urdf_fk import UrdfChain

    description = Path(packages.get_package_share_directory('cleany_description'))
    xml = xacro.process_file(str(description/'urdf/cleany.urdf.xacro'),
        mappings={'optimized_arm_collision_meshes': 'true'}).toxml()
    root = ET.fromstring(xml)
    data = json.loads(fitted.MODEL_PATH.read_text())

    def components(name, filename=None):
        link = root.find(f"link[@name='{name}']")
        for collision in link.findall('collision'):
            mesh, origin = collision.find('geometry/mesh'), collision.find('origin')
            if filename and not mesh.get('filename').endswith('/'+filename):
                continue
            geometry = trimesh.load_mesh(fitted.mesh_path(mesh.get('filename')), process=True)
            geometry.apply_scale(np.fromstring(mesh.get('scale', '1 1 1'), sep=' '))
            rotation = trimesh.transformations.euler_matrix(*np.fromstring(origin.get('rpy'), sep=' '))[:3, :3]
            for part in geometry.split(only_watertight=False):
                points = np.vstack((part.vertices, part.triangles_center,
                                    part.vertices[part.edges_unique].mean(axis=1)))
                yield points @ rotation.T + np.fromstring(origin.get('xyz'), sep=' ')

    yaw = f'{side}_shoulder_yaw_link'
    sphere = data['links'][yaw]['spheres'][0]
    vertices = np.concatenate(list(components(yaw)))
    assert np.linalg.norm(vertices-sphere['center'], axis=1).max() <= sphere['radius']-data['padding_m']

    parent, child = f'{side}_upper_arm_link', f'{side}_lower_arm_link'
    upper_balls = data['links'][parent]['spheres'][:2]
    lower_balls = data['links'][child]['spheres'][:2]
    upper_c = np.array([s['center'] for s in upper_balls])
    lower_c = np.array([s['center'] for s in lower_balls])
    upper_r = np.array([s['radius'] for s in upper_balls])
    lower_r = np.array([s['radius'] for s in lower_balls])
    assert upper_r.min() < .027 and upper_r.max() < .037
    assert lower_r.min() < .025 and lower_r.max() < .037
    # Include complete sockets and the necks that overlap the carbon rod.
    upper = [v for v in components(parent, 'Upper_Arm_5cm.stl') if v[:, 1].min() > .1]
    lower = [v for v in components(child, 'Lower_Arm_5cm.stl') if v[:, 2].max() < .075]
    assert len(upper) == 1 and len(lower) == 3
    upper_midline = (upper[0][:, 0].min()+upper[0][:, 0].max())/2
    neck = next(v for v in lower if v[:, 2].min() > .03)
    lower_midline = (neck[:, 0].min()+neck[:, 0].max())/2
    assert np.abs(upper_c[:, 0]-upper_midline).max() < .00025
    assert np.abs(lower_c[:, 0]-lower_midline).max() < .00025
    lower = np.concatenate(lower)
    assert lower[:, 2].min() < -.011 and lower[:, 2].max() > .066
    upper = np.concatenate(upper + list(components(parent, 'Upper_Arm_Motor.stl')))
    assert (np.linalg.norm(upper[:, None]-upper_c, axis=2)-upper_r).min(1).max() <= -data['padding_m']
    chain = UrdfChain(xml, parent, child)
    centers = []
    for angle in np.linspace(-.22, 3.14, 41):
        position, rotation = chain.pose({f'{side}_elbow_pitch_joint': angle})
        points = lower @ rotation.T + position
        center = lower_c @ rotation.T + position
        centers.append(center)
        assert (np.linalg.norm(points[:, None]-center, axis=2)-lower_r).min(1).max() <= -data['padding_m']
    assert np.linalg.norm(centers[0]-centers[-1]) > .04


@pytest.mark.parametrize('side', ['left', 'right'])
def test_shoulder_upper_pair_covers_housing_through_yaw_and_pitch(side):
    import numpy as np
    from pathlib import Path
    import trimesh
    import xml.etree.ElementTree as ET
    xacro = pytest.importorskip('xacro')
    packages = pytest.importorskip('ament_index_python.packages')
    from cleany_skill_executor.core.urdf_fk import UrdfChain

    description = Path(packages.get_package_share_directory('cleany_description'))
    xml = xacro.process_file(str(description/'urdf/cleany.urdf.xacro'),
        mappings={'optimized_arm_collision_meshes': 'true'}).toxml()
    root = ET.fromstring(xml)
    name = f'{side}_upper_arm_link'
    data = json.loads(fitted.MODEL_PATH.read_text())
    spheres = data['links'][name]['spheres'][2:4]
    centers = np.array([s['center'] for s in spheres])
    radii = np.array([s['radius'] for s in spheres])
    assert radii.max() < .040 and radii.min() < .029
    assert np.abs(centers[:, 0]).max() < .001
    parts = []
    for collision in root.find(f"link[@name='{name}']").findall('collision'):
        mesh, origin = collision.find('geometry/mesh'), collision.find('origin')
        if not mesh.get('filename').endswith('/Upper_Arm_5cm.stl'):
            continue
        geometry = trimesh.load_mesh(fitted.mesh_path(mesh.get('filename')), process=True)
        geometry.apply_scale(np.fromstring(mesh.get('scale', '1 1 1'), sep=' '))
        rotation = trimesh.transformations.euler_matrix(*np.fromstring(origin.get('rpy'), sep=' '))[:3, :3]
        for part in geometry.split(only_watertight=False):
            points = np.vstack((part.vertices, part.triangles_center,
                                part.vertices[part.edges_unique].mean(axis=1)))
            points = points @ rotation.T + np.fromstring(origin.get('xyz'), sep=' ')
            if points[:, 1].max() < .07:
                parts.append(points)
    assert len(parts) == 1 and parts[0][:, 1].min() < -.011
    chain = UrdfChain(xml, 'base_link', name)
    for yaw in np.linspace(-2.16, 2.16, 7):
        for pitch in np.linspace(-.22, 3.37, 7):
            p, r = chain.pose({f'{side}_shoulder_yaw_joint': yaw,
                               f'{side}_shoulder_pitch_joint': pitch})
            points, transformed = parts[0] @ r.T+p, centers @ r.T+p
            assert (np.linalg.norm(points[:, None]-transformed, axis=2)-radii).min(1).max() <= -data['padding_m']


def test_three_head_balls_cover_pan_tilt_and_camera_on_their_owning_links():
    import numpy as np
    from pathlib import Path
    import trimesh
    import xml.etree.ElementTree as ET
    xacro = pytest.importorskip('xacro')
    packages = pytest.importorskip('ament_index_python.packages')
    from cleany_skill_executor.core.urdf_fk import UrdfChain

    description = Path(packages.get_package_share_directory('cleany_description'))
    xml = xacro.process_file(str(description/'urdf/cleany.urdf.xacro'), mappings={
        'optimized_arm_collision_meshes': 'true', 'include_head_camera': 'true',
        'head_fixed': 'false'}).toxml()
    root = ET.fromstring(xml)
    data = json.loads(fitted.MODEL_PATH.read_text())
    assert len(data['links']['head_pan_link']['spheres']) == 1
    assert len(data['links']['head_tilt_link']['spheres']) == 2
    # The camera mesh belongs to the tilt link; the optical frame has no body.
    assert root.find("link[@name='head_camera_link']/collision") is None
    expected = {'head_pan_link': {'tophead1.stl': 0, 'tophead4.stl': 0},
                'head_tilt_link': {'tophead5.stl': 0, 'tophead6.stl': 1}}
    for name, filenames in expected.items():
        chain = UrdfChain(xml, 'base_link', name)
        collisions = root.find(f"link[@name='{name}']").findall('collision')
        assert len(collisions) == len(filenames)
        for collision in collisions:
            mesh, origin = collision.find('geometry/mesh'), collision.find('origin')
            index = filenames[Path(mesh.get('filename')).name]
            sphere = data['links'][name]['spheres'][index]
            geometry = trimesh.load_mesh(fitted.mesh_path(mesh.get('filename')), process=True)
            geometry.apply_scale(np.fromstring(mesh.get('scale', '1 1 1'), sep=' '))
            rotation = trimesh.transformations.euler_matrix(*np.fromstring(origin.get('rpy'), sep=' '))[:3, :3]
            vertices = geometry.vertices @ rotation.T + np.fromstring(origin.get('xyz'), sep=' ')
            for pan in np.linspace(-1.57, 1.57, 7):
                for tilt in np.linspace(-.5, .7, 7):
                    p, r = chain.pose({'head_pan_joint': pan, 'head_tilt_joint': tilt})
                    points, center = vertices @ r.T + p, r @ sphere['center'] + p
                    assert np.linalg.norm(points-center, axis=1).max() <= sphere['radius']-data['padding_m']


@pytest.mark.parametrize('side', ['left', 'right'])
def test_wrist_mount_pair_encloses_primitives_and_follows_pitch_and_roll(side):
    import numpy as np
    from pathlib import Path
    import trimesh
    import xml.etree.ElementTree as ET
    xacro = pytest.importorskip('xacro')
    packages = pytest.importorskip('ament_index_python.packages')
    from cleany_skill_executor.core.urdf_fk import UrdfChain

    description = Path(packages.get_package_share_directory('cleany_description'))
    xml = xacro.process_file(str(description/'urdf/cleany.urdf.xacro'),
        mappings={'optimized_arm_collision_meshes': 'true'}).toxml()
    root = ET.fromstring(xml)
    name = f'{side}_gripper_frame'
    data = json.loads(fitted.MODEL_PATH.read_text())
    covers = [c for c in data['rigid_covers']
              if c['link'] == name and c['region'] == 'wrist_camera_mount']
    assert len(covers) == 2 and all(c['sphere_count'] == 2 for c in covers)
    assert {c['part_index'] for c in covers} == {0, 1}
    assert len(data['links'][name]['spheres']) == 15
    spheres = [data['links'][name]['spheres'][c['sphere_index']] for c in covers]
    local_centers = np.array([s['center'] for s in spheres])
    radii = np.array([s['radius'] for s in spheres])
    assert .026 < radii.min() < .027 and .029 < radii.max() < .031
    points, kinds = [], []
    for collision in root.find(f"link[@name='{name}']").findall('collision'):
        if 'wrist_mount_v8_' not in collision.get('name', ''):
            continue
        origin = collision.find('origin')
        position = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
        rotation = trimesh.transformations.euler_matrix(
            *np.fromstring(origin.get('rpy', '0 0 0'), sep=' '))[:3, :3]
        local = (local_centers-position) @ rotation
        box, cylinder = collision.find('geometry/box'), collision.find('geometry/cylinder')
        if box is not None:
            size = np.fromstring(box.get('size'), sep=' ')
            from itertools import product
            sample = np.array(list(product(*[np.linspace(-v/2, v/2, 5) for v in size])))
            points.append(sample @ rotation.T+position)
            kinds.append('box')
        else:
            assert cylinder is not None
            # Exact maximum distance to the complete cylinder, not a polygon.
            farthest = np.hypot(np.linalg.norm(local[:, :2], axis=1)+float(cylinder.get('radius')),
                               np.abs(local[:, 2])+float(cylinder.get('length'))/2)
            assert (farthest-radii).min() <= -data['padding_m']
            kinds.append('cylinder')
    assert kinds.count('box') == 14 and kinds.count('cylinder') == 1
    points = np.concatenate(points)
    chain = UrdfChain(xml, 'base_link', name)
    centers = []
    for pitch in np.linspace(-1.5, 1.5, 7):
        for roll in np.linspace(-3., 3., 7):
            state = {joint.get('name'): 0. for joint in root.findall('joint')
                     if joint.get('type') in ('revolute', 'continuous', 'prismatic')}
            state.update({f'{side}_wrist_pitch_joint': pitch, f'{side}_wrist_roll_joint': roll})
            p, r = chain.pose(state)
            transformed = local_centers @ r.T+p
            centers.append(transformed)
            world = points @ r.T+p
            assert (np.linalg.norm(world[:, None]-transformed, axis=2)-radii).min(1).max() <= -data['padding_m']
            p_open, r_open = chain.pose({**state, f'{side}_gripper_joint': -.4})
            np.testing.assert_allclose(local_centers @ r_open.T+p_open, transformed, atol=1e-12)
    assert np.ptp(centers, axis=0).max() > .05
