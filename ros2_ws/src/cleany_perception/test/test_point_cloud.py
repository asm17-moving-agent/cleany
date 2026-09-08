import numpy as np

from cleany_perception.core.models import CameraIntrinsics, Plane, RigidTransform
from cleany_perception.core.point_cloud import (
    ColoredPointCloud,
    colored_cloud_from_selection,
    transform_colored_cloud,
)


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
