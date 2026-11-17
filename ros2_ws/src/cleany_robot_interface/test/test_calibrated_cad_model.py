"""Reject incomplete commissioning without changing the existing model path."""
import hashlib
import json

import pytest

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
