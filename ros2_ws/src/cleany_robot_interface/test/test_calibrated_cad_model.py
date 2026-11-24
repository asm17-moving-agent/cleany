"""Reject incomplete commissioning without changing the existing model path."""
import hashlib
import json
import xml.etree.ElementTree as ET

import pytest

from cleany_robot_interface.state_profile import apply_collision_mesh_overrides, share_world_collision_geometry
from cleany_robot_interface.calibrated_cad_model import model_from_binding, require_cad_motion_ready


def test_failed_cad_opening_is_rejected_before_actuator_startup(tmp_path):
    binding = tmp_path/'binding.json'
    binding.write_text(json.dumps({'motion_validation':{'ready':False,'reason':'drive-rod interference'}}))
    profile = tmp_path/'profile.json'
    profile.write_text(json.dumps({'model_variant':'freecad_calibrated','cad_binding_file':'binding.json'}))
    with pytest.raises(ValueError,match='drive-rod interference'):
        require_cad_motion_ready(profile,tmp_path)
    assert sorted(p.name for p in tmp_path.iterdir()) == ['binding.json','profile.json']
    profile.write_text(json.dumps({'robot_model':'legacy.urdf.xacro'}))
    require_cad_motion_ready(profile,tmp_path)


def test_stale_camera_or_geometry_cannot_reuse_calibration(tmp_path):
    source = tmp_path/'cad.urdf'
    pose = tmp_path/'pose.json'
    source.write_text('<robot/>')
    pose.write_text(json.dumps({'joint_coordinates':{}}))
    binding = {'schema':'cleany.cad_grasp_binding/v1',
               'source_model_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
               'pose_config_sha256':hashlib.sha256(pose.read_bytes()).hexdigest(),
               'joint_coordinates':{},'joint_dynamics':{}}
    source.write_text('<robot name="changed"/>')
    with pytest.raises(ValueError,match='dependency changed'):
        model_from_binding(source,binding,pose,{})
    source.write_text('<robot/>')
    pose.write_text(json.dumps({'joint_coordinates':{'changed':1}}))
    with pytest.raises(ValueError,match='dependency changed'):
        model_from_binding(source,binding,pose,{})


def test_collision_mesh_override_is_scoped_and_content_addressed(tmp_path):
    directory = tmp_path / 'ros2_ws/src/cleany_description/meshes/cad_runtime_collision'
    directory.mkdir(parents=True)
    prefix = 'package://cleany_description/meshes/cad_runtime_collision/'
    original = '1' * 64
    digest = hashlib.sha256(b'new geometry').hexdigest()
    target = directory / (digest + '.glb')
    target.write_bytes(b'new geometry')
    shape = f'<geometry><mesh filename="{prefix}{original}.glb"/></geometry>'
    xml = f'<robot><link><visual>{shape}</visual><collision>{shape}</collision></link></robot>'
    tree = ET.fromstring(xml)
    assert apply_collision_mesh_overrides(tree, {original: digest}, tmp_path) == {target}
    assert tree.find('.//visual/geometry/mesh').get('filename') == prefix + original + '.glb'
    assert tree.find('.//collision/geometry/mesh').get('filename') == prefix + digest + '.glb'
    with pytest.raises(ValueError, match='missing source'):
        apply_collision_mesh_overrides(tree, {original: digest}, tmp_path)
    target.write_bytes(b'different geometry')
    with pytest.raises(ValueError, match='digest mismatch'):
        apply_collision_mesh_overrides(ET.fromstring(xml), {original: digest}, tmp_path)


def test_share_world_collision_geometry_preserves_solids_and_unselected_links():
    shapes = ('<visual name="skin"/><collision name="common"/>'
              '<collision name="cad_world__cad_enclosure__tube"><geometry/></collision>'
              '<collision name="cad_self__tube"/>')
    tree = ET.fromstring(f'<robot><link name="arm">{shapes}</link><link name="drive">{shapes}</link></robot>')
    arm, drive = tree.findall('link')
    original_drive = ET.tostring(drive)
    share_world_collision_geometry(tree, ['arm'])
    assert [node.get('name') for node in arm] == ['skin', 'common', 'cad_enclosure__tube']
    assert arm.find('collision/geometry') is not None
    assert ET.tostring(drive) == original_drive
    with pytest.raises(ValueError, match='both CAD collision representations'):
        share_world_collision_geometry(tree, ['arm'])
    with pytest.raises(ValueError, match='Unknown CAD collision link'):
        share_world_collision_geometry(tree, ['missing'])
