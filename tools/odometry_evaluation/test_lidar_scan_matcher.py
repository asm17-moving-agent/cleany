#!/usr/bin/env python3
"""Test 2D LiDAR Scan Matcher."""

import math
import numpy as np
import pytest

from lidar_scan_matcher import LidarScanMatcher


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
