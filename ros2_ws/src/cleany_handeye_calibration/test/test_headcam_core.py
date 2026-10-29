"""Real-board detection and offline calibration guards."""
import json

import cv2
import numpy as np
import pytest

from cleany_handeye_calibration.headcam_core import (
    calibrate_directory, detect, make_board,
)
from cleany_handeye_calibration.headcam_handeye import board_pose


def config():
    return dict(squares_x=7, squares_y=5, square_length_m=0.050,
                marker_length_m=0.035, dictionary='DICT_6X6_250',
                minimum_corners=16, width=640, height=480,
                minimum_calibration_views=8)


def test_actual_board_dictionary_and_blank():
    cfg = config()
    dictionary, board = make_board(cfg)
    image = cv2.cvtColor(board.draw((640, 480), marginSize=30), cv2.COLOR_GRAY2BGR)
    found = detect(image, dictionary, board, 16)
    assert found['valid'] and len(found['ids']) == 24
    assert set(found['marker_ids'].flatten()) == set(range(17))
    assert not detect(np.full_like(image, 255), dictionary, board, 16)['valid']
    cfg['marker_length_m'] = 0.060
    with pytest.raises(ValueError):
        make_board(cfg)


def test_insufficient_views_rejected(tmp_path):
    (tmp_path / 'manifest.json').write_text(json.dumps({'config': config()}))
    with pytest.raises(ValueError, match='Only 0 valid views'):
        calibrate_directory(tmp_path)


def test_partial_board_accepted_but_single_row_rejected():
    dictionary, board = make_board(config())
    image = cv2.cvtColor(board.draw((640, 480), marginSize=30), cv2.COLOR_GRAY2BGR)
    image[:200] = 255
    partial = detect(image, dictionary, board, 8)
    assert partial['valid'] and 8 <= len(partial['ids']) < 24
    image[:285] = 255
    # Even a lowered count threshold must reject a collinear board row.
    row = detect(image, dictionary, board, 4)
    assert len(row['ids']) >= 4 and not row['valid']


def test_board_pose_recovers_camera_to_board_and_rejects_bad_fit():
    _, board = make_board(config())
    k = np.array([[600., 0, 320], [0, 605., 240], [0, 0, 1]])
    rvec = np.array([.35, -.2, .1])
    tvec = np.array([-.15, -.1, .7])
    points = cv2.projectPoints(board.chessboardCorners, rvec, tvec, k, np.zeros(5))[0]
    found = {'ids': np.arange(24).reshape(-1, 1), 'corners': points}
    policy = {'maximum_pnp_rmse_px': 1., 'minimum_pnp_ratio': 1.05}
    camera = {'K': k.tolist(), 'D': [0.]*5}
    pose = board_pose(found, board, camera, policy)
    matrix = np.array(pose['best']['camera_T_board'])
    np.testing.assert_allclose(matrix[:3, 3], tvec, atol=1e-5)
    np.testing.assert_allclose(matrix[:3, :3], cv2.Rodrigues(rvec)[0], atol=1e-5)
    found['corners'] = points.copy()
    found['corners'][0] += 30
    with pytest.raises(ValueError, match='reprojection'):
        board_pose(found, board, camera, policy)


def test_calibration_from_projected_board_images(tmp_path):
    cfg = config()
    dictionary, board = make_board(cfg)
    (tmp_path / 'manifest.json').write_text(json.dumps({'config': cfg}))
    texture = board.draw((1400, 1000))
    k = np.array([[700., 0, 320], [0, 710., 240], [0, 0, 1]])
    outer = np.float32([[0, 0, 0], [.35, 0, 0], [.35, .25, 0], [0, .25, 0]])
    src = np.float32([[0, 0], [1399, 0], [1399, 999], [0, 999]])
    rng = np.random.default_rng(21)
    for i in range(20):
        rotation = rng.uniform(-.4, .4, 3)
        translation = np.array([-.175, -.125, rng.uniform(.7, .95)])
        dst, _ = cv2.projectPoints(outer, rotation, translation, k, np.zeros(5))
        image = cv2.warpPerspective(texture, cv2.getPerspectiveTransform(
            src, dst.reshape(-1, 2)), (640, 480), borderValue=255)
        sample = tmp_path / f'sample_{i:03d}'
        sample.mkdir()
        cv2.imwrite(str(sample / 'rgb.png'), image)
        (sample / 'record.json').write_text('{}')
    result = calibrate_directory(tmp_path)
    assert result['rms_px'] < 1.0
    assert abs(result['K'][0][0] / 700 - 1) < .06
    assert abs(result['K'][1][1] / 710 - 1) < .06
    assert result['review_required']
