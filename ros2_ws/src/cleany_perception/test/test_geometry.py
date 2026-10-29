import numpy as np
import pytest

from cleany_perception.core.geometry import (
    deproject_masked_depth,
    fit_plane_ransac,
    quaternion_xyzw_from_rotation,
    reconstruct_supported_obb,
    transform_box,
)
from cleany_perception.core.models import CameraIntrinsics, Plane, RigidTransform
from cleany_perception.core.point_cloud import (
    ColoredPointCloud,
    colored_cloud_from_selection,
    transform_colored_cloud,
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


def test_cloud_applies_mask_preserves_rgb_and_limits_points() -> None:
    depth = np.ones((4, 4), dtype=np.float32)
    rgb = np.arange(48, dtype=np.uint8).reshape((4, 4, 3))
    mask = np.zeros((4, 4), dtype=bool)
    mask[1:3, 1:3] = True
    intrinsics = CameraIntrinsics(4, 4, 2.0, 2.0, 1.5, 1.5)

    cloud = colored_cloud_from_selection(
        depth, rgb, intrinsics, mask, 0.1, 2.0, 0.01, 3
    )

    assert cloud.points.shape == (3, 3)
    assert all(tuple(color) in map(tuple, rgb[mask]) for color in cloud.colors)
    assert np.all(cloud.points[:, 2] == 1.0)


def test_cloud_voxel_downsampling_is_deterministic() -> None:
    depth = np.ones((2, 2), dtype=np.float32)
    rgb = np.full((2, 2, 3), 42, dtype=np.uint8)
    intrinsics = CameraIntrinsics(2, 2, 1000.0, 1000.0, 0.5, 0.5)

    cloud = colored_cloud_from_selection(
        depth, rgb, intrinsics, np.ones((2, 2), dtype=bool), 0.1, 2.0, 0.01, 10
    )

    assert cloud.points.shape == (4, 3)  # Points straddle four signed voxels.


def test_grasp_target_reuses_reconstructed_support_plane():
    depth = np.array([[0.75, 0.8]], dtype=np.float32)
    rgb = np.array([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)
    intrinsics = CameraIntrinsics(2, 1, 100., 100., 0.5, 0.0)
    cloud = colored_cloud_from_selection(
        depth, rgb, intrinsics, np.ones((1, 2), dtype=bool),
        0.1, 2.0, 0.001, 100,
        support_plane=Plane(np.array((0, 0, -1)), 0.8, 100),
    )
    assert cloud.points.shape == (1, 3)
    assert cloud.colors.tolist() == [[1, 2, 3]]


def test_cloud_transform_preserves_rgb_and_changes_coordinates() -> None:
    cloud = ColoredPointCloud(
        points=np.array(((1.0, 2.0, 3.0),), dtype=np.float32),
        colors=np.array(((10, 20, 30),), dtype=np.uint8),
    )
    transform = RigidTransform(
        translation=np.array((4.0, 5.0, 6.0)),
        rotation=np.diag((1.0, -1.0, -1.0)),
    )

    transformed = transform_colored_cloud(cloud, transform)

    np.testing.assert_allclose(transformed.points, [[5.0, 3.0, 3.0]])
    assert transformed.colors.tolist() == [[10, 20, 30]]


@pytest.mark.parametrize('stride', (1, 2, 4, 20))
@pytest.mark.parametrize('noncontiguous', (False, True))
def test_depth_sampling_preserves_pixel_grid_validity_and_input(stride, noncontiguous):
    depth = np.linspace(0.25, 2.0, 99, dtype=np.float32).reshape(9, 11)
    depth[::2, ::3] = np.nan
    depth[1, :5] = (np.inf, -np.inf, -1., 0., 2.1)
    mask = np.ones(depth.shape, dtype=bool)
    mask[2:6, 3:8] = False
    if noncontiguous:
        depth, mask = depth[::-1, ::-1], mask[::-1, ::-1]
    original_depth, original_mask = depth.copy(), mask.copy()
    depth.setflags(write=False)
    mask.setflags(write=False)
    intrinsics = CameraIntrinsics(11, 9, 12., 10., 4.5, 3.5)

    points = deproject_masked_depth(depth, intrinsics, mask, 0.25, 2., stride)

    expected = []
    for row in range(0, 9, stride):
        for column in range(0, 11, stride):
            z = float(depth[row, column])
            if mask[row, column] and np.isfinite(z) and 0.25 <= z <= 2.:
                expected.append(((column-4.5)*z/12., (row-3.5)*z/10., z))
    np.testing.assert_array_equal(points, np.asarray(expected).reshape(-1, 3))
    np.testing.assert_array_equal(depth, original_depth)
    np.testing.assert_array_equal(mask, original_mask)


def test_plane_complete_consensus_still_refines_and_enforces_limits():
    rng = np.random.default_rng(18)
    xy = rng.uniform(-0.3, 0.3, (500, 2))
    points = np.column_stack((xy, np.full(len(xy), 0.8)))
    plane = fit_plane_ransac(points, 200, 0.001, 100, 0.9)
    np.testing.assert_allclose(plane.normal, (0., 0., -1.), atol=1e-12)
    assert plane.offset == pytest.approx(0.8, abs=1e-12)
    assert plane.inlier_count == len(points)
    with pytest.raises(ValueError, match='required'):
        fit_plane_ransac(points, 200, 0.001, 100, 1.1)
