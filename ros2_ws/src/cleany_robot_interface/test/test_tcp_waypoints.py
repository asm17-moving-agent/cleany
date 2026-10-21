import json

import numpy as np
import pytest

from cleany_robot_interface.tcp_waypoints import Dataset, board_tcp, compare_point, digest, pose_error, sequence, transform


def translation(x=0., y=0., z=0.):
    matrix = np.eye(4)
    matrix[:3, 3] = (x, y, z)
    return matrix


def dataset(tmp_path):
    (tmp_path / 'model').write_text('frozen')
    manifest = {'schema': 1, 'side': 'left', 'model_sha256': 'model', 'command_joints': ['joint'],
                'state_joints': ['joint'], 'gripper_T_camera': np.eye(4).tolist(),
                'gripper_T_tcp': np.eye(4).tolist(), 'frozen_sha256': {'model': digest(tmp_path / 'model')}}
    (tmp_path / 'session.json').write_text(json.dumps(manifest))
    return Dataset.load(tmp_path)


def record():
    return {'positions_rad': {'joint': .2}, 'stationary_checks_passed': True, 'torque_hold_verified': True,
            'base_T_tcp': np.eye(4).tolist(), 'board_T_tcp': np.eye(4).tolist()}


def test_board_camera_composition_and_external_error_independent_of_encoders():
    # A camera displaced 10mm in the board frame returns a changed C_T_B.
    gripper_camera = translation(.01, -.06, .06)
    gripper_tcp = translation(0, -.105, 0)
    board_gripper = translation(.2, .3, .5)
    board_camera = board_gripper @ gripper_camera
    initial = board_tcp(np.linalg.inv(board_camera), gripper_camera, gripper_tcp)
    np.testing.assert_allclose(initial, board_gripper @ gripper_tcp, atol=1e-12)
    moved_camera = translation(.01) @ board_camera
    arrival = board_tcp(np.linalg.inv(moved_camera), gripper_camera, gripper_tcp)
    result = compare_point({'base_T_tcp': np.eye(4), 'board_T_tcp': initial},
        {'base_T_tcp': np.eye(4), 'board_T_tcp': arrival, 'stationary_checks_passed': True})
    assert result['encoder_tcp']['position_mm'] == 0
    assert result['board_video_tcp']['position_mm'] == pytest.approx(10)
    np.testing.assert_allclose(result['board_video_tcp']['delta_xyz_mm'], [10, 0, 0], atol=1e-10)
    assert not result['absolute_tcp_accuracy_verified']


def test_rotated_handeye_transform_and_angle():
    rotation = np.eye(4)
    rotation[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    g_c, g_tcp, b_g = rotation @ translation(.1, .2, .3), translation(.005, -.105, 0), translation(.3, .4, .5) @ rotation
    np.testing.assert_allclose(board_tcp(np.linalg.inv(b_g @ g_c), g_c, g_tcp), b_g @ g_tcp)
    assert pose_error(np.eye(4), rotation)['orientation_deg'] == pytest.approx(90)


@pytest.mark.parametrize('value', [np.zeros((4, 4)), np.full((4, 4), float('nan')), np.diag([-1, 1, 1, 1]), np.eye(3)])
def test_invalid_transforms_are_rejected(value):
    with pytest.raises(ValueError):
        transform(value)


def test_missing_board_or_unsettled_arrival_never_returns_zero():
    with pytest.raises(KeyError):
        compare_point(record(), {'base_T_tcp': np.eye(4), 'stationary_checks_passed': True})
    with pytest.raises(ValueError):
        compare_point(record(), {'base_T_tcp': np.eye(4), 'board_T_tcp': np.eye(4), 'stationary_checks_passed': False})


def test_point_provenance_duplicate_and_frozen_input_mutation(tmp_path):
    store = dataset(tmp_path)
    store.commit(1, record(), b'original image')
    assert store.point(1)['positions_rad'] == {'joint': .2}
    with pytest.raises(FileExistsError):
        store.commit(1, record(), b'other')
    (tmp_path / 'points/1/rgb.png').write_bytes(b'changed')
    with pytest.raises(ValueError, match='mismatch'):
        store.point(1)
    (tmp_path / 'model').write_text('modified')
    with pytest.raises(ValueError, match='Frozen input'):
        Dataset.load(tmp_path)


def test_wrong_side_and_unpowered_records_are_rejected(tmp_path):
    store = dataset(tmp_path)
    store.commit(1, record(), b'image')
    path = tmp_path / 'points/1/point.json'
    data = json.loads(path.read_text())
    data['side'] = 'right'
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        store.point(1)
    data['side'] = 'left'
    data['torque_hold_verified'] = False
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='powered-hold'):
        store.point(1)


def test_positive_numbered_sequence_allows_return_loops():
    assert sequence('1, 2,3,4,5,1') == (1, 2, 3, 4, 5, 1)
    for invalid in ('', '0', '-1,2', '1,,2', '1.2,3'):
        with pytest.raises(ValueError):
            sequence(invalid)


def test_invalid_capture_is_not_committed_and_number_can_be_retried(tmp_path):
    store = dataset(tmp_path)
    invalid = record()
    invalid['positions_rad'] = {}
    with pytest.raises(ValueError, match='Incomplete'):
        store.commit(1, invalid, b'image')
    assert not (tmp_path / 'points/1').exists()
    assert not list((tmp_path / 'points').iterdir())
    store.commit(1, record(), b'image')
    assert store.point(1)['number'] == 1
