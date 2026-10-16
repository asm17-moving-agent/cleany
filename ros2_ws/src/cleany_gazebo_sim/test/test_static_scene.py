"""Geometry, state and frame boundaries for opt-in static scene grouping."""
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import pytest

from cleany_gazebo_sim.world.static_scene import GROUP_NAME, materialize_static_scene


def model(name='chair', extra='', pose='1 2 3 0.2 -0.4 0.7'):
    return f'''<model name="{name}"><static>true</static><pose>{pose}</pose>{extra}
    <link name="{name}_link"><pose>0.4 0.5 0.6 -0.3 0.1 0.2</pose>
    <collision name="shape"><pose>0.1 0.2 0.3 0.4 0.2 -0.1</pose>
    <geometry><box><size>1 2 3</size></box></geometry>
    <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision>
    <visual name="view"><geometry><mesh><uri>file:///unchanged.dae</uri><scale>2 3 4</scale></mesh></geometry>
    <material><ambient>0.1 0.2 0.3 1</ambient></material></visual></link></model>'''


def source(tmp_path: Path, content: str) -> Path:
    path = tmp_path / 'world.sdf'
    path.write_text(f'<sdf version="1.7"><world name="test">{content}</world></sdf>')
    return path


def rotation(pose):
    # Independent coordinate composition using elementary axis rotations.
    x, y, z, r, p, a = map(float, pose.split())
    cr, sr, cp, sp, ca, sa = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(a), np.sin(a)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[ca, -sa, 0], [sa, ca, 0], [0, 0, 1]])
    result = np.eye(4)
    result[:3, :3] = rz @ ry @ rx
    result[:3, 3] = [x, y, z]
    return result


@pytest.mark.parametrize('pose', ['1 2 3 0.2 -0.4 0.7', '0 0 0 0 1.5707963267948966 0.5'])
def test_preserves_world_shapes_properties_and_dynamic_model(tmp_path, pose):
    robot = '<model name="robot"><link name="base"><sensor name="camera" type="camera"/></link></model>'
    path = source(tmp_path, model(pose=pose) + model('desk') + robot)
    before = path.read_bytes()
    original = ET.parse(path).find('./world')
    target = materialize_static_scene(path)
    world = ET.parse(target).find('./world')
    assert path.read_bytes() == before
    assert target != path
    assert ET.tostring(world.find("model[@name='robot']")) == ET.tostring(original.find("model[@name='robot']"))
    link = world.find(f"model[@name='{GROUP_NAME}']/link")
    assert len(link.findall('collision')) == 2
    assert len(link.findall('visual')) == 2
    for old, new in zip(original.findall("model[@name='chair']/link/*")[1:] + original.findall("model[@name='desk']/link/*")[1:], list(link)):
        old_model = original.find("model[@name='chair']") if new.get('name').startswith('static_0_') else original.find("model[@name='desk']")
        expected = rotation(old_model.findtext('pose')) @ rotation(old_model.findtext('link/pose')) @ rotation(old.findtext('pose', '0 0 0 0 0 0'))
        np.testing.assert_allclose(rotation(new.findtext('pose')), expected, atol=1e-12, rtol=0)
        for tag in ['geometry', 'surface', 'material']:
            if old.find(tag) is not None:
                assert ET.tostring(old.find(tag)) == ET.tostring(new.find(tag))


@pytest.mark.parametrize('extra', ['<plugin name="p" filename="p.so"/>', '<frame name="f"/>', '<joint name="j" type="fixed"/>', '<include><uri>model://other</uri></include>'])
def test_stateful_and_framed_models_remain_unchanged(tmp_path, extra):
    path = source(tmp_path, model(extra=extra))
    assert materialize_static_scene(path) == path


@pytest.mark.parametrize('reference', ['<frame name="f" attached_to="chair"/>', '<model name="other"><pose relative_to="chair::chair_link">0 0 0 0 0 0</pose></model>'])
def test_external_frame_references_remain_valid(tmp_path, reference):
    path = source(tmp_path, model() + reference)
    assert materialize_static_scene(path) == path


def test_directional_friction_and_explicit_pose_are_not_reparented(tmp_path):
    content = model().replace('<mu>0.8</mu>', '<fdir1>1 0 0</fdir1><mu>0.8</mu>')
    assert materialize_static_scene(source(tmp_path, content)) == tmp_path / 'world.sdf'
    content = model().replace('<mu2>0.8</mu2>', '<mu2>0.2</mu2>')
    assert materialize_static_scene(source(tmp_path, content)) == tmp_path / 'world.sdf'
    content = model().replace('<pose>', '<pose relative_to="world">', 1)
    assert materialize_static_scene(source(tmp_path, content)) == tmp_path / 'world.sdf'


@pytest.mark.parametrize('content', [model(pose='nan 0 0 0 0 0'), model(pose='0 0'), model(GROUP_NAME)])
def test_invalid_geometry_and_reserved_name_fail_closed(tmp_path, content):
    with pytest.raises(ValueError):
        materialize_static_scene(source(tmp_path, content))


def test_empty_pose_uses_sdf_identity_default(tmp_path):
    content = model().replace('<pose>1 2 3 0.2 -0.4 0.7</pose>', '<pose/>')
    target = materialize_static_scene(source(tmp_path, content))
    world = ET.parse(target).find('./world')
    geometry = world.find(f"model[@name='{GROUP_NAME}']/link/collision")
    expected = rotation('0.4 0.5 0.6 -0.3 0.1 0.2') @ rotation('0.1 0.2 0.3 0.4 0.2 -0.1')
    np.testing.assert_allclose(rotation(geometry.findtext('pose')), expected, atol=1e-12, rtol=0)
