"""Synthetic geometry checks; these do not validate physical calibration."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest


path = Path(__file__).resolve().parents[2] / 'tools/validation/compare_depth_lidar_wall.py'
spec = importlib.util.spec_from_file_location('wall_comparison', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_tilted_wall_with_noise_and_outliers():
    rng = np.random.default_rng(4)
    yz = rng.uniform(-0.6, 0.6, (1200, 2))
    normal = np.array([1.0, 0.05, 0.08])
    normal /= np.linalg.norm(normal)
    x = (1.4-yz @ normal[1:])/normal[0] + rng.normal(0, 0.003, 1200)
    points = np.column_stack((x, yz))
    points[:100, 0] += 0.3
    result = module.fit_wall(points)
    np.testing.assert_allclose(result['normal'], normal, atol=0.002)
    assert abs(result['offset_m']-1.4) < 0.002
    assert 0.9 < result['inlier_fraction'] < 0.93


def test_lidar_line_distance_and_normal_sign():
    y = np.linspace(-0.5, 0.5, 200)
    result = module.fit_wall(np.column_stack((0.9-0.1*y, y)))
    assert result['offset_m']/result['normal'][0] == pytest.approx(0.9)
    assert result['yaw_deg'] == pytest.approx(np.degrees(np.arctan(0.1)))


def test_side_wall_is_not_front_wall():
    with pytest.raises(ValueError, match='No dominant'):
        module.fit_wall(np.column_stack((np.linspace(0, 2, 100), np.ones(100))))


def test_optical_axes_map_to_robot_axes():
    r = module.rotation([-0.5, 0.5, -0.5, 0.5])
    np.testing.assert_allclose(r @ [0, 0, 1], [1, 0, 0])
    np.testing.assert_allclose(r @ [1, 0, 0], [0, -1, 0])
    np.testing.assert_allclose(r @ [0, 1, 0], [0, 0, -1])


def test_dynamic_head_overrides_nominal_pose_and_rejects_stale_feedback():
    data = {'tf_static': {
        'd435_depth_optical_frame': {'parent': 'd435_depth_frame', 'xyz': [0, 0, 0], 'xyzw': [-.5, .5, -.5, .5]},
        'd435_depth_frame': {'parent': 'd435_link', 'xyz': [0, 0, 0], 'xyzw': [0, 0, 0, 1]}},
        'tf_dynamic': [
            {'child': 'd435_link', 'parent': 'd435_tilt_link', 'xyz': [.03, 0, .03], 'xyzw': [0, 0, 0, 1], 'stamp': 10},
            {'child': 'd435_tilt_link', 'parent': 'd435_pan_link', 'xyz': [0, 0, 0], 'xyzw': [0, 0, 0, 1], 'stamp': 10},
            {'child': 'd435_pan_link', 'parent': 'base_link', 'xyz': [.1, 0, 1], 'xyzw': [0, 0, np.sin(np.pi/4), np.cos(np.pi/4)], 'stamp': 10}]}
    fallback = {'parent': 'base_link', 'translation_m': [9, 9, 9]}
    r, p = module.camera_pose(data, 10.01, fallback)
    np.testing.assert_allclose(p, [.1, .03, 1.03], atol=1e-12)
    np.testing.assert_allclose(r @ [0, 0, 1], [0, 1, 0], atol=1e-12)
    with pytest.raises(ValueError, match='fresh'):
        module.camera_pose(data, 11, fallback)
