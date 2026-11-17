"""Rigid camera housings follow calibration without relaxing scene collisions."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from cleany_robot_interface.rigid_accessories import attach_rigid_accessories, _transform


SOURCE = '''<robot name="cad"><link name="cad_wrist">
<visual name="Camera_visual"><origin xyz="0.01 0.02 0.03" rpy="0 0 0"/>
<geometry><box size="0.04 0.05 0.01"/></geometry></visual>
<collision name="Camera_collision"><origin xyz="0.01 0.02 0.03" rpy="0 0 0"/>
<geometry><box size="0.04 0.05 0.01"/></geometry></collision>
</link></robot>'''
MODEL = '''<robot name="physical"><link name="wrist">
<collision name="arm"><geometry><box size="0.02 0.03 0.04"/></geometry></collision>
<collision name="old_mount_case"><geometry><box size="1 1 1"/></geometry></collision>
</link><link name="camera"/><link name="tcp"/>
<joint name="camera_fixed" type="fixed"><parent link="wrist"/><child link="camera"/>
<origin xyz="0.04 -0.05 0.06" rpy="0.3 -0.2 0.4"/></joint>
<joint name="tcp_fixed" type="fixed"><parent link="wrist"/><child link="tcp"/>
<origin xyz="0 -0.119775 0" rpy="0 0 0"/></joint></robot>'''


def configuration():
    source_pose = ET.fromstring('<origin xyz="0.002 0.003 -0.004" rpy="0.2 0.4 -0.3"/>')
    return {'source_model_sha256': hashlib.sha256(SOURCE.encode()).hexdigest(),
            'source_link': 'cad_wrist', 'source_parent_from_camera': _transform(source_pose).tolist(),
            'target_link': 'wrist', 'target_camera_joint': 'camera_fixed',
            'parts': ['Camera'], 'replace_shape_prefixes': ['old_mount_']}


def test_registered_rigid_shapes_share_measured_camera_transform():
    config = configuration()
    original = ET.fromstring(MODEL)
    result = ET.fromstring(attach_rigid_accessories(MODEL, SOURCE, config))
    actual = _transform(result.find("./link/collision[@name='registered_Camera_collision']/origin"))
    source_pose = _transform(ET.fromstring(SOURCE).find('./link/collision/origin'))
    expected = _transform(original.find("joint[@name='camera_fixed']/origin")) @ np.linalg.inv(
        config['source_parent_from_camera']) @ source_pose
    np.testing.assert_allclose(actual, expected, atol=1e-14)
    assert [e.get('name') for e in result.findall('./link/collision')] == ['arm', 'registered_Camera_collision']
    assert [ET.tostring(e) for e in result.findall('joint')] == [ET.tostring(e) for e in original.findall('joint')]
    assert result.find('.//disable_collisions') is None


@pytest.mark.parametrize('change', ['hash', 'missing_part', 'wrong_parent', 'nonrigid', 'duplicate'])
def test_registration_fails_closed(change):
    config = configuration()
    model = MODEL
    if change == 'hash': config['source_model_sha256'] = 'wrong'
    if change == 'missing_part': config['parts'] = ['missing']
    if change == 'wrong_parent': config['target_link'] = 'tcp'
    if change == 'nonrigid': config['source_parent_from_camera'][0][0] = 3.
    if change == 'duplicate': config['parts'] *= 2
    with pytest.raises(ValueError):
        attach_rigid_accessories(model, SOURCE, config)


def test_repeated_registration_is_rejected():
    config = configuration()
    once = attach_rigid_accessories(MODEL, SOURCE, config)
    with pytest.raises(ValueError, match='already registered'):
        attach_rigid_accessories(once, SOURCE, config)


def test_deployed_registration_uses_only_rigid_parts_and_existing_cad_visuals():
    root = Path(__file__).resolve().parents[4]
    config = json.loads((root/'configs/robot/left_rigid_wrist_geometry.json').read_text())
    source = (root/config['source_model']).read_text()
    assert hashlib.sha256(source.encode()).hexdigest() == config['source_model_sha256']
    model = ET.fromstring(source)
    assert len(config['parts']) == 23
    assert len(model.findall('./link/collision')) == len(config['parts'])
    assert all('gripper' not in name.lower() and 'Rod_' not in name for name in config['parts'])
    assert any('Top_Plate' in name for name in config['parts'])
    for shape in model.findall('./link/collision'):
        assert '/meshes/collision/rigid_wrist/' in shape.find('geometry/mesh').get('filename')
    for shape in model.findall('./link/visual'):
        assert '/meshes/rigid_wrist_source/' in shape.find('geometry/mesh').get('filename')


def test_explicit_registration_keeps_dynamic_optical_tf_out_of_geometry():
    model = MODEL.replace('</robot>', '<link name="base"/>'
        '<joint name="wrist_parent" type="revolute"><parent link="base"/><child link="wrist"/>'
        '<origin xyz="0 .02 0" rpy="0 0 0"/><axis xyz="0 0 1"/>'
        '<limit lower="-2" upper="2" effort="1" velocity="1"/></joint></robot>')
    tree = ET.fromstring(model)
    tree.remove(tree.find("joint[@name='camera_fixed']"))
    model = ET.tostring(tree, encoding='unicode')
    config = configuration()
    config.pop('target_camera_joint')
    config['target_parent_from_source'] = np.eye(4).tolist()
    config['target_parent_from_source'][0][3] = .015
    config['target_parent_joint'] = {'name': 'wrist_parent', 'parent_link': 'base',
        'origin_xyz_m': [0., .02, 0.], 'origin_rpy_rad': [0., 0., 0.], 'axis': [0., 0., 1.]}
    result = ET.fromstring(attach_rigid_accessories(model, SOURCE, config))
    actual = _transform(result.find("./link/collision[@name='registered_Camera_collision']/origin"))
    np.testing.assert_allclose(actual[:3, 3], [.025, .02, .03], atol=1e-14)
    assert [ET.tostring(j) for j in result.findall('joint')] == [
        ET.tostring(j) for j in tree.findall('joint')]
    config['target_parent_joint']['origin_xyz_m'][0] = .01
    with pytest.raises(ValueError, match='parent-joint contract'):
        attach_rigid_accessories(model, SOURCE, config)
