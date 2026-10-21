#!/usr/bin/env python3
"""Test 2D LiDAR Scan Matcher."""

import math
import numpy as np
import pytest

import lidar_scan_matcher
from lidar_scan_matcher import LidarScanMatcher, validate_scan_match


def test_synthetic_scan_matching_convergence():
    matcher = LidarScanMatcher(voxel_size_m=0.01)

    # Synthetic room: 4x4m square box with some random features
    wall_x1 = np.column_stack((np.full(100, 2.0), np.linspace(-2.0, 2.0, 100)))
    wall_x2 = np.column_stack((np.full(100, -2.0), np.linspace(-2.0, 2.0, 100)))
    wall_y1 = np.column_stack((np.linspace(-2.0, 2.0, 100), np.full(100, 2.0)))
    wall_y2 = np.column_stack((np.linspace(-2.0, 2.0, 100), np.full(100, -2.0)))
    pillar = np.column_stack((np.full(20, 0.5), np.linspace(0.5, 0.7, 20)))

    ref_points = np.vstack([wall_x1, wall_x2, wall_y1, wall_y2, pillar])

    # True offset: +12.0 mm in X, -18.0 mm in Y, +2.5 degrees in Yaw
    true_dx = 0.012
    true_dy = -0.018
    true_yaw_deg = 2.5
    true_yaw_rad = math.radians(true_yaw_deg)

    c = math.cos(true_yaw_rad)
    s = math.sin(true_yaw_rad)
    R = np.array([[c, -s], [s, c]])

    # Target points transformed by true motion
    # p_ref = R * p_target + t => p_target = R^T * (p_ref - t)
    target_points = (R.T @ (ref_points - np.array([true_dx, true_dy])).T).T

    # Add small Gaussian noise (1mm stddev)
    rng = np.random.default_rng(42)
    target_points += rng.normal(0, 0.001, target_points.shape)

    result = matcher.match(ref_points, target_points)

    assert result.converged
    assert result.inlier_ratio > 0.95
    assert abs(result.delta_x_m - true_dx) < 0.002  # error < 2mm
    assert abs(result.delta_y_m - true_dy) < 0.002  # error < 2mm
    assert abs(result.delta_yaw_deg - true_yaw_deg) < 0.2  # error < 0.2 deg
    assert result.rmse_mm < 3.0
    assert result.is_valid
    assert result.rejection_reason == "Valid scan match"


def test_single_wall_degeneracy_rejection():
    """Verify that a single straight planar wall is rejected due to unobservable translation axis.

    Reproduces the failure case where a 100mm sliding displacement along a featureless wall
    produces low RMSE and high inlier ratio but fails Hessian observability.
    """
    matcher = LidarScanMatcher(voxel_size_m=0.01)

    # Single straight wall along Y axis at X = 2.0m (-2.5m to +2.5m)
    wall_y = np.linspace(-2.5, 2.5, 200)
    wall_x = np.full(200, 2.0)
    ref_points = np.column_stack((wall_x, wall_y))

    # True motion: robot moves 100mm along the wall (Y direction)
    true_dx = 0.0
    true_dy = 0.100  # 100 mm displacement
    target_points = ref_points - np.array([true_dx, true_dy])

    result = matcher.match(ref_points, target_points)

    # Even if Point-to-Line residual claims convergence or low RMSE,
    # it must be REJECTED as invalid due to translation axis degeneracy!
    assert not result.is_valid
    assert "Degenerate translation" in result.rejection_reason
    assert result.min_translation_eigenvalue < 5.0 or result.translation_condition_number > 35.0


def test_circular_geometry_rejects_unobservable_yaw():
    matcher = LidarScanMatcher(voxel_size_m=0.005)
    angles = np.linspace(0.0, 2.0 * math.pi, 360, endpoint=False)
    circle = np.column_stack((2.0 * np.cos(angles), 2.0 * np.sin(angles)))

    result = matcher.match(circle, circle.copy())

    assert abs(result.delta_yaw_rad) < 1e-9
    assert math.isinf(result.condition_number)
    assert not result.is_valid
    assert "full SE(2) Hessian" in result.rejection_reason


def test_sparse_scan_match_is_invalid_with_specific_reason():
    matcher = LidarScanMatcher()
    result = matcher.match(np.zeros((10, 2)), np.zeros((10, 2)))

    assert not result.is_valid
    assert "Insufficient scan points" in result.rejection_reason


@pytest.mark.parametrize("minimum,accepted", [(.30, True), (.40, False)])
def test_matcher_applies_configured_inlier_minimum(monkeypatch, minimum, accepted):
    matcher = LidarScanMatcher(min_inlier_ratio=minimum)
    coordinates = np.linspace(-2., 2., 100)
    room = np.vstack([
        np.column_stack((coordinates, np.full(100, -2.))),
        np.column_stack((coordinates, np.full(100, 2.))),
        np.column_stack((np.full(100, -2.), coordinates)),
        np.column_stack((np.full(100, 2.), coordinates)),
    ])
    def validate(result, **kwargs):
        assert kwargs["min_inlier_ratio"] == minimum
        result.inlier_ratio = .35
        return validate_scan_match(result, **kwargs)
    monkeypatch.setattr(lidar_scan_matcher, "validate_scan_match", validate)
    result = matcher.match(room, room.copy())
    assert result.converged
    assert result.is_valid is accepted
    assert matcher.max_iterations == 50
    assert matcher.tolerance == 1e-5
    result.converged = False
    assert not validate_scan_match(result, min_inlier_ratio=.30)[0]


@pytest.mark.parametrize("value", [0, -.1, 1.01, float("nan"), float("inf"), True, "0.3"])
def test_invalid_inlier_minimum_rejected(value):
    with pytest.raises(ValueError, match="min_inlier_ratio"):
        LidarScanMatcher(min_inlier_ratio=value)


def square_room():
    coordinates = np.linspace(-2., 2., 100)
    return np.vstack([
        np.column_stack((coordinates, np.full(100, -2.))),
        np.column_stack((coordinates, np.full(100, 2.))),
        np.column_stack((np.full(100, -2.), coordinates)),
        np.column_stack((np.full(100, 2.), coordinates)),
    ])


@pytest.mark.parametrize("pose", [(.5, -.4, .2), (-2., 3., -1.)])
def test_mixed_residual_jacobian_matches_additive_pose_finite_difference(pose):
    points = np.array([[1., 2.], [-.7, 1.3]])
    normals = np.array([[.6, .8], [.8, -.6]])
    linearity = np.array([1., 0.])  # Both point-to-line and point-to-point.

    def rotated(value):
        c, s = math.cos(value[2]), math.sin(value[2])
        return points @ np.array([[c, s], [-s, c]])

    pose = np.array(pose)
    reference = rotated(pose) + pose[:2]

    def system(value):
        return lidar_scan_matcher._linearized_system(
            rotated(value), value[:2], reference, normals, linearity,
        )

    actual, _ = system(pose)
    numerical = np.empty_like(actual)
    for index in range(3):
        step = np.zeros(3)
        step[index] = 1e-6
        numerical[:, index] = -(system(pose + step)[1] - system(pose - step)[1]) / 2e-6
    np.testing.assert_allclose(actual, numerical, atol=1e-8, rtol=1e-6)


@pytest.mark.parametrize("iterations,converged", [(13, False), (14, True)])
def test_small_step_cannot_converge_before_final_correspondence_stage(iterations, converged):
    room = square_room()
    result = LidarScanMatcher(max_iterations=iterations).match(room, room)
    assert result.converged is converged
    assert result.is_valid is converged
    assert result.iterations == iterations
    assert result.convergence_reason == ("step_tolerance" if converged else "maximum_iterations")


def test_noisy_scan_cycle_converges_without_relaxing_strict_step_tolerances():
    room = square_room()
    rng = np.random.default_rng(2)
    translation = np.array([-.085, .489])
    yaw = .0058
    c, s = math.cos(yaw), math.sin(yaw)
    reference = room + rng.normal(0, .012, room.shape)
    target = (room - translation) @ np.array([[c, -s], [s, c]])
    target += rng.normal(0, .012, room.shape)
    matcher = LidarScanMatcher(voxel_size_m=.015, min_inlier_ratio=.30)
    result = matcher.match(reference, target, (*translation, yaw))

    assert result.is_valid
    assert result.convergence_reason == "bounded_cycle"
    assert 14 < result.iterations < 50
    assert result.final_step_translation_m > matcher.tolerance
    assert result.convergence_settings["translation_tolerance_m"] == 1e-5
    assert result.convergence_settings["rotation_tolerance_rad"] == 1e-5
    assert np.linalg.norm([result.delta_x_m - translation[0],
                           result.delta_y_m - translation[1]]) < .002
    assert abs(result.delta_yaw_rad - yaw) < math.radians(.1)
    # Convergence is not quality acceptance.
    result.inlier_ratio = .29
    assert not validate_scan_match(result, min_inlier_ratio=.30)[0]
    result.inlier_ratio = .9
    result.min_translation_eigenvalue = 0.
    assert not validate_scan_match(result, min_inlier_ratio=.30)[0]


def test_observability_uses_final_inliers_and_same_additive_jacobian(monkeypatch):
    room = square_room()
    outliers = np.column_stack((np.full(20, 2.035), np.linspace(-1.5, 1.5, 20)))
    calls = []
    original = lidar_scan_matcher._linearized_system

    def track(*args):
        matrix, rhs = original(*args)
        calls.append((matrix.copy(), args[0].copy()))
        return matrix, rhs

    monkeypatch.setattr(lidar_scan_matcher, "_linearized_system", track)
    result = LidarScanMatcher(voxel_size_m=.005).match(room, np.vstack([room, outliers]))
    matrix, rotated_points = calls[-1]
    assert len(calls[-2][0]) > len(matrix)  # 40 mm fit versus 30 mm final inliers.
    trans_h = matrix[:, :2].T @ matrix[:, :2]
    eigenvalues = np.linalg.eigvalsh(trans_h)
    assert result.min_translation_eigenvalue == pytest.approx(eigenvalues[0])
    assert result.translation_condition_number == pytest.approx(eigenvalues[-1] / eigenvalues[0])
    matrix[:, 2] /= np.sqrt(np.mean(np.sum(rotated_points ** 2, axis=1)))
    full_eigenvalues = np.linalg.eigvalsh(matrix.T @ matrix)
    assert result.full_min_eigenvalue == pytest.approx(full_eigenvalues[0])
    assert result.condition_number == pytest.approx(full_eigenvalues[-1] / full_eigenvalues[0])


def test_convergence_configuration_is_validated_and_snapshotted():
    settings = {"translation_tolerance_m": 2e-5, "rotation_tolerance_rad": 3e-5,
                "max_translation_span_m": .0005, "max_rotation_span_rad": .0001,
                "max_period": 4}
    matcher = LidarScanMatcher(convergence_config=settings)
    result = matcher.match(square_room(), square_room())
    assert result.convergence_settings == settings
    assert matcher.tolerance == 2e-5
    assert matcher.rotation_tolerance_rad == 3e-5
    settings["max_period"] = 100
    assert result.convergence_settings["max_period"] == 4


@pytest.mark.parametrize("iterations", [0, -1, 1.5, True])
def test_invalid_iteration_limit_rejected(iterations):
    with pytest.raises(ValueError, match="max_iterations"):
        LidarScanMatcher(max_iterations=iterations)
