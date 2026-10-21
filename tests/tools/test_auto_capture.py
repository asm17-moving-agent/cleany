import numpy as np

from auto_capture import AutoCaptureGate, view_signature
from cleany_handeye_calibration.headcam_core import make_board


def gate():
    return AutoCaptureGate(1.5, 2., 20., 1.)


def test_movement_dropout_and_frame_gap_restart_hold():
    g = gate()
    p = np.zeros((24, 2))
    assert not g.update(p, 0)[0]
    assert not g.update(p, .8)[0]
    assert not g.update(p + 5, 1.5)[0]
    assert not g.update(None, 2)[0]
    assert not g.update(p, 2.1)[0]
    assert not g.update(p, 2.9)[0]
    assert g.update(p, 3.7)[0]
    assert not g.update(p, 5)[0]


def test_return_to_previous_view_is_not_a_new_sample():
    g = gate()
    p = np.zeros((24, 2))
    g.commit(p)
    for t in [0, .8, 1.6]:
        assert not g.update(p + 1, t)[0]
    for t in [2, 2.8]:
        assert not g.update(p + 40, t)[0]
    assert g.update(p + 40, 3.6)[0]
    g.commit(p + 40)
    for t in [4, 4.8, 5.6]:
        assert not g.update(p, t)[0]


def test_partial_visibility_does_not_create_false_view_change():
    _, board = make_board(dict(squares_x=7, squares_y=5,
                              square_length_m=.05, marker_length_m=.035,
                              dictionary='DICT_6X6_250'))
    points = board.chessboardCorners[:, :2] * 800 + [30, 40]
    full = {'ids': np.arange(24).reshape(-1, 1), 'corners': points.reshape(-1, 1, 2)}
    partial = {'ids': np.arange(12).reshape(-1, 1), 'corners': points[:12].reshape(-1, 1, 2)}
    np.testing.assert_allclose(view_signature(full, board), view_signature(partial, board), atol=.001)
