"""Offline slip diagnostics must preserve frame, timing and unknown boundaries."""
import importlib.util
import math
from pathlib import Path
import sys
from dataclasses import replace

import numpy as np
import pytest

SOURCE = Path(__file__).resolve().parents[2] / 'tools/validation/slip_detection.py'
spec = importlib.util.spec_from_file_location('slip_detection', SOURCE)
slip = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = slip
spec.loader.exec_module(slip)


def test_body_frame_and_wrapped_yaw():
    a = np.array([1., 2., math.pi/2])
    b = slip.compose(a, np.array([.1, -.2, .03]))
    np.testing.assert_allclose(slip.relative_pose(a, b), [.1, -.2, .03], atol=1e-12)
    assert slip.wrap(-2*math.pi+.1) == pytest.approx(.1)


def trajectory():
    t = np.arange(0., 1.01, .02)
    return np.column_stack((t, t*.1, t*0, t*.1+3.12))


def test_interpolation_across_yaw_wrap():
    odom = trajectory()
    odom[:, 3] = [slip.wrap(x) for x in odom[:, 3]]
    delta = slip.odom_delta(odom, .015, .995, .12)
    assert np.linalg.norm(delta[:2]) == pytest.approx(.098)
    assert delta[2] == pytest.approx(.098)


@pytest.mark.parametrize('mode', ['gap', 'reset_xy', 'reset_yaw', 'nan', 'empty', 'duplicate'])
def test_bad_odometry_is_rejected(mode):
    odom = trajectory()
    if mode == 'gap':
        odom = np.delete(odom, np.arange(10, 30), axis=0)
    elif mode == 'reset_xy':
        odom[25:, 1] += 1
    elif mode == 'reset_yaw':
        odom[25:, 3] += 1
    elif mode == 'nan':
        odom[25, 1] = np.nan
    elif mode == 'empty':
        odom = odom[:0]
    else:
        odom[25, 0] = odom[24, 0]
    with pytest.raises(ValueError):
        slip.odom_delta(odom, .01, .99, .12)


@pytest.mark.parametrize('wheel,lidar,status,kind', [
    ([.1, 0, 0], [.1, 0, 0], 'no_evidence', 'none'),
    ([.1, 0, 0], [0, 0, 0], 'suspect', 'wheel_spin_or_stuck'),
    ([0, .1, 0], [-.1, 0, 0], 'suspect', 'direction_mismatch'),
    ([0, 0, .2], [0, 0, 0], 'suspect', 'rotation'),
    ([.001, 0, .001], [0, 0, 0], 'no_evidence', 'none'),
])
def test_motion_disagreement(wheel, lidar, status, kind):
    row = slip.classify(np.array(wheel), np.array(lidar), True, slip.Settings())
    assert (row['status'], row['kind']) == (status, kind)


def test_invalid_lidar_is_unknown_even_when_wheels_spin():
    assert slip.classify(np.array([1., 0, 0]), np.zeros(3), False, slip.Settings())['status'] == 'unknown'


def test_unknown_breaks_persistence():
    def row(t, status):
        return dict(t0=t, t1=t+1, status=status, kind='translation', translation_error_m=.05)
    rows = [row(0, 'suspect'), row(.5, 'unknown'), row(1, 'suspect')]
    assert slip.events(rows, slip.Settings()) == []
    rows.append(row(1.5, 'suspect'))
    assert slip.events(rows, slip.Settings())[0]['start'] == 1


def test_independent_icp_recovers_known_body_displacement():
    x = np.linspace(-2, 3, 151)
    y = np.linspace(-1, 2, 121)
    reference = np.vstack((np.column_stack((x, np.full_like(x, -1))),
                           np.column_stack((x, np.full_like(x, 2))),
                           np.column_stack((np.full_like(y, -2), y)),
                           np.column_stack((np.full_like(y, 3), y))))
    truth = np.array([.08, -.06, .025])
    c, s = math.cos(truth[2]), math.sin(truth[2])
    target = (reference-truth[:2]) @ np.array([[c, -s], [s, c]])
    result = slip.LidarScanMatcher(voxel_size_m=.05).match(reference, target)
    assert result.is_valid, result.rejection_reason
    np.testing.assert_allclose(slip.match_vector(result), truth, atol=.003)
    assert slip.classify(np.array([.08, .10, .025]), slip.match_vector(result), True, slip.Settings())['status'] == 'suspect'


def test_single_wall_cannot_certify_motion():
    wall = np.column_stack((np.linspace(-2, 2, 200), np.ones(200)))
    result = slip.LidarScanMatcher().match(wall, wall)
    assert not result.is_valid


@pytest.mark.parametrize('fault', ['far', 'cycle', 'unconverged', 'duration'])
def test_quality_failure_never_becomes_slip_or_no_evidence(monkeypatch, fault):
    from lidar_scan_matcher import ScanMatchResult
    good = ScanMatchResult(0, 0, 0, 0, 0, 0, 1, 100, 100, True, 1, 1)
    forward, reverse, far = good, good, good
    if fault == 'far':
        far = replace(good, delta_x_m=.1)
    elif fault == 'cycle':
        reverse = replace(good, delta_x_m=.1)
    elif fault == 'unconverged':
        forward = replace(good, is_valid=False, rejection_reason='not converged')
    outcomes = iter([forward, reverse, far])
    class Matcher:
        def match(self, *_):
            return next(outcomes)
    monkeypatch.setattr(slip, '_matcher', Matcher(), raising=False)
    monkeypatch.setattr(slip, '_config', slip.Settings(), raising=False)
    monkeypatch.setattr(slip, 'cloud_at', lambda *args: np.zeros((100, 2)))
    monkeypatch.setattr(slip, '_data', {
        'scans': np.array([[0, .4 if fault == 'duration' else .13, .13], [1, .13, .13]]),
        'odom': trajectory(), 'wheels': np.empty((0, 9)),
    }, raising=False)
    row = slip.analyze_pair((0, 1))
    assert row['status'] == 'unknown'
    assert row['reason']
