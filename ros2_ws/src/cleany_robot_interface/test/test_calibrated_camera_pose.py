import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from cleany_robot_interface.calibrated_camera_pose import CameraPoseModel, remove_camera_frames


def test_camera_pose_uses_calibration_joint_coordinates(tmp_path):
    model = tmp_path / 'robot.urdf'
    model.write_text('''<robot name="test"><link name="base_link"/><link name="wrist"/>
      <joint name="cad_joint" type="revolute"><parent link="base_link"/><child link="wrist"/>
      <origin xyz="1 2 3" rpy="0 0 0"/><axis xyz="0 0 1"/></joint></robot>''')
    mount = np.eye(4)
    mount[0, 3] = .1
    data = {'model_file': str(model), 'model_sha256': hashlib.sha256(model.read_bytes()).hexdigest(),
            'joint_coordinates': {'cad_joint': {'source_joint': 'operational_joint', 'scale': -2.,
                                               'offset_rad': np.pi / 2}},
            'cameras': {'left': {'parent_frame': 'wrist', 'parent_T_camera': mount.tolist(),
                                 'optical_frames': ['left_optical']}}}
    config = tmp_path / 'camera.json'
    config.write_text(json.dumps(data))
    pose = CameraPoseModel(config)
    np.testing.assert_allclose(pose.poses({'operational_joint': 0})['left'][:3, 3], [1, 2.1, 3])
    np.testing.assert_allclose(pose.poses({'operational_joint': np.pi / 4})['left'][:3, 3], [1.1, 2, 3])
    assert pose.poses({}) == {}
    with pytest.raises(ValueError, match='Nonfinite'):
        pose.poses({'operational_joint': float('nan')})
    model.write_text(model.read_text() + '\n')
    with pytest.raises(ValueError, match='hash'):
        CameraPoseModel(config)


def test_external_camera_owner_removes_only_optical_branches():
    model = '''<robot><link name="base_link"/><link name="arm"/><link name="optical"/>
      <link name="rgb_alias"/><joint name="arm_joint" type="revolute">
      <parent link="base_link"/><child link="arm"/></joint>
      <joint name="camera_joint" type="fixed"><parent link="arm"/><child link="optical"/></joint>
      <joint name="alias" type="fixed"><parent link="optical"/><child link="rgb_alias"/></joint></robot>'''
    root = ET.fromstring(remove_camera_frames(model, {'optical'}))
    assert {x.get('name') for x in root.findall('link')} == {'base_link', 'arm'}
    assert root.find("joint[@name='arm_joint']").get('type') == 'revolute'
    with pytest.raises(ValueError, match='movable'):
        remove_camera_frames(model, {'arm'})


def test_state_profile_selects_new_camera_without_legacy_extrinsics(tmp_path, monkeypatch):
    from cleany_robot_interface.state_profile import prepare_state_profile
    import cleany_robot_interface.state_profile as state_profile

    tree = ET.Element('robot', name='synthetic')
    ET.SubElement(tree, 'link', name='base_link')
    joints = [f'{side}_{suffix}_joint' for side in ('left', 'right')
              for suffix in ('shoulder_yaw', 'shoulder_pitch', 'elbow_pitch',
                             'wrist_pitch', 'wrist_roll', 'gripper')]
    for name in [*joints, 'head_pan_joint', 'head_tilt_joint']:
        ET.SubElement(tree, 'link', name=name + '_link')
        joint = ET.SubElement(tree, 'joint', name=name, type='revolute')
        ET.SubElement(joint, 'parent', link='base_link')
        ET.SubElement(joint, 'child', link=name + '_link')
    ET.SubElement(tree, 'link', name='head_camera_rgb_optical_frame')
    joint = ET.SubElement(tree, 'joint', name='old_optical', type='fixed')
    ET.SubElement(joint, 'parent', link='base_link')
    ET.SubElement(joint, 'child', link='head_camera_rgb_optical_frame')
    model = tmp_path / 'synthetic.urdf'
    model.write_text(ET.tostring(tree, encoding='unicode'))
    pose = tmp_path / 'camera-pose.json'
    pose.write_text(json.dumps({
        'model_file': str(model),
        'model_sha256': hashlib.sha256(model.read_bytes()).hexdigest(),
        'joint_coordinates': {},
        'cameras': {'head': {'parent_frame': 'base_link',
                            'parent_T_camera': np.eye(4).tolist(),
                            'optical_frames': ['head_camera_rgb_optical_frame']}}}))
    for filename, data in [('arm.json', {}), ('head.json', {'camera_serial': 'synthetic'}),
                           ('head-mapping.json', {})]:
        (tmp_path / filename).write_text(json.dumps(data))
    config = tmp_path / 'profile.json'
    config.write_text(json.dumps({
        'arm_config': 'arm.json', 'head_config': 'head.json',
        'head_joint_mapping': 'head-mapping.json', 'robot_model': str(model),
        'camera_pose_config': str(pose),
        'xacro_args': {'head_fixed': 'false', 'include_wheel_joints': 'false'}}))
    monkeypatch.setattr(state_profile.subprocess, 'check_output',
                        lambda *args, **kwargs: model.read_text())
    result = prepare_state_profile(config, tmp_path, tmp_path / 'materialized')
    assert result.camera_pose_config == pose
    active = ET.parse(result.model).getroot()
    assert active.find("link[@name='head_camera_rgb_optical_frame']") is None
    assert active.find("joint[@name='head_pan_joint']") is not None
