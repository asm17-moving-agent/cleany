import numpy as np
import pytest

from cleany_perception.core.geometry import (
    deproject_masked_depth,
    fit_plane_ransac,
    quaternion_xyzw_from_rotation,
    reconstruct_supported_obb,
    transform_box,
)


def test_synthetic_supported_box_meets_center_and_size_tolerances(
    synthetic_scene,
):
    snapshot = synthetic_scene['snapshot']
    mask = synthetic_scene['mask']
    support = ~mask
    support_points = deproject_masked_depth(
        snapshot.depth_m,
        snapshot.intrinsics,
        support,
        0.1,
        3.0,
        stride=4,
    )
    plane = fit_plane_ransac(
        support_points,
        iterations=100,
        distance_threshold_m=0.001,
        minimum_inliers=100,
        minimum_inlier_ratio=0.9,
    )
    object_points = deproject_masked_depth(
        snapshot.depth_m,
        snapshot.intrinsics,
        mask,
        0.1,
        3.0,
    )
    camera_box = reconstruct_supported_obb(
        object_points,
        plane,
        minimum_height_m=0.005,
        minimum_extent_m=0.005,
        minimum_points=30,
    )
    box = transform_box(camera_box, synthetic_scene['transform'])

    center_error = np.linalg.norm(
        box.center - synthetic_scene['expected_center']
    )
    size_error = np.abs(
        np.sort(box.size[:2])
        - np.sort(synthetic_scene['expected_size'][:2])
    )

    assert center_error <= 0.005
    assert np.all(size_error <= 0.010)
    assert box.size[2] == pytest.approx(0.20, abs=0.005)
    assert plane.normal == pytest.approx((0.0, 0.0, -1.0), abs=1e-6)


def test_plane_ransac_rejects_insufficient_inliers():
    points = np.random.default_rng(3).normal(size=(20, 3))

    with pytest.raises(ValueError, match='Not enough support points'):
        fit_plane_ransac(
            points,
            iterations=10,
            distance_threshold_m=0.001,
            minimum_inliers=30,
            minimum_inlier_ratio=0.9,
        )


def test_quaternion_conversion_round_trip_for_half_turn():
    rotation = np.diag((1.0, -1.0, -1.0))

    quaternion = quaternion_xyzw_from_rotation(rotation)

    assert quaternion == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-7)
