import numpy as np

from cleany_perception.core.models import CameraIntrinsics
from cleany_perception.core.point_cloud import colored_cloud_from_selection


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
