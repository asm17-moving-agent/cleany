"""Coordinate ownership and contact permissions for the reused CAD linkage."""
import json
from pathlib import Path

import numpy as np
import pytest

from cleany_description.cad_coordinates import ATTRIBUTE, CadCoordinates, gripper_touch_links, match_gripper_endpoints


@pytest.fixture
def coordinates():
    source = Path(__file__).parents[1]/'config/geometry/freecad_kinematics.json'
    geometry = json.loads(source.read_text())
    return CadCoordinates({'schema': 'cleany.calibrated_cad/v1', 'grippers': [
        {'side': group['hand'], 'source_joint': group['hand']+'_gripper_joint',
         'scale': -1., 'offset_rad': -.3745329776277857,
         'linkage': group} for group in geometry['gripper_linkages']]})


def test_derived_fingers_replace_stale_values_without_changing_motor_coordinates(coordinates):
    motors = {'left_gripper_joint': 2.4173120563240813,
              'right_gripper_joint': -.3745329776277857,
              'left_elbow_pitch_joint': .8}
    expanded = coordinates.expanded({**motors, **dict.fromkeys(coordinates.dependent_names, 999.)})
    assert all(expanded[name] == value for name, value in motors.items())
    assert len(expanded) == len(motors)+16
    assert np.isfinite(list(expanded.values())).all()
    right = [v for name,v in expanded.items() if name.startswith('cad_delta_right_')]
    np.testing.assert_allclose(right, 0., atol=1e-12)
    assert motors['left_elbow_pitch_joint'] == .8


def test_nonfinite_cam_does_not_emit_partial_linkage(coordinates):
    motors = {'left_gripper_joint': float('nan'), 'right_gripper_joint': 0.}
    with pytest.raises(ValueError, match='Nonfinite'):
        coordinates.expanded(motors)
    assert set(motors) == {'left_gripper_joint','right_gripper_joint'}


def test_motor_stroke_matches_cad_endpoints_without_reusing_old_one_to_one(coordinates):
    root = Path(__file__).resolve().parents[4]
    endpoints = json.loads((root/'configs/robot/cad_gripper_endpoints.json').read_text())
    mapped = match_gripper_endpoints(coordinates.config, endpoints)
    original = {g['side']:g for g in coordinates.config['grippers']}
    for group in mapped['grippers']:
        pair = endpoints['grippers'][group['side']]
        assert group['scale'] != original[group['side']]['scale']
        for end in ('open','closed'):
            motor = pair['motor_'+end+'_rad']
            assert group['scale']*motor+group['offset_rad'] == pytest.approx(pair['cad_'+end+'_rad'])
        assert (pair['motor_open_rad']-pair['motor_closed_rad'])*180/np.pi == pytest.approx(159.9609375)
    corrected = CadCoordinates(mapped)
    values = np.linspace(0., 1., 321)
    for fraction in values:
        motors = {g['source_joint']:
                  g['endpoint_mapping']['motor_open_rad']*(1-fraction)+
                  g['endpoint_mapping']['motor_closed_rad']*fraction for g in mapped['grippers']}
        assert np.isfinite(list(corrected.expanded(motors).values())).all()
    assert all(g['scale'] == -1. for g in coordinates.config['grippers'])


def test_stale_endpoint_mapping_is_rejected(coordinates):
    root = Path(__file__).resolve().parents[4]
    endpoints = json.loads((root/'configs/robot/cad_gripper_endpoints.json').read_text())
    mapped = match_gripper_endpoints(coordinates.config, endpoints)
    mapped['grippers'][0]['offset_rad'] += .01
    with pytest.raises(ValueError, match='endpoint mapping is stale'):
        CadCoordinates(mapped)


def test_target_permissions_include_actual_hand_bodies_but_exclude_arm():
    xml = f'''<robot {ATTRIBUTE}="unused"><link name="base_link"><collision/></link>
      <link name="cad_left_wrist_roll"><collision/></link><link name="finger"><collision/></link>
      <link name="forearm"><collision/></link>
      <joint name="arm"><parent link="base_link"/><child link="forearm"/></joint>
      <joint name="wrist"><parent link="forearm"/><child link="cad_left_wrist_roll"/></joint>
      <joint name="finger"><parent link="cad_left_wrist_roll"/><child link="finger"/></joint></robot>'''
    links = gripper_touch_links(xml,'left')
    assert {'cad_left_wrist_roll','finger','left_gripper_frame','left_moving_jaw_link'} == set(links)
    assert not {'base_link','forearm'} & set(links)
