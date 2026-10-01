from __future__ import annotations

import numpy as np
import pytest

from cleany_perception.core.models import (
    BoundingBox2D,
    CameraIntrinsics,
    Detection2D,
    RgbdSnapshot,
    RigidTransform,
)


@pytest.fixture
def synthetic_scene():
    width = 320
    height = 240
    intrinsics = CameraIntrinsics(
        width=width,
        height=height,
        fx=400.0,
        fy=400.0,
        cx=159.5,
        cy=119.5,
    )
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    rgb[:, :] = (90, 70, 50)
    depth = np.ones((height, width), dtype=np.float32)
    mask = np.zeros((height, width), dtype=np.bool_)
    mask[100:140, 135:185] = True
    depth[mask] = 0.8
    rgb[mask] = (40, 100, 230)
    detection = Detection2D(
        label='box',
        confidence=0.95,
        bbox=BoundingBox2D(
            x_min=135.0,
            y_min=100.0,
            x_max=185.0,
            y_max=140.0,
        ),
    )
    snapshot = RgbdSnapshot(
        rgb=rgb,
        depth_m=depth,
        intrinsics=intrinsics,
        stamp_ns=1_500_000_000,
        source_frame='camera_optical_frame',
    )
    transform = RigidTransform(
        translation=np.array((0.0, 0.0, 1.0)),
        rotation=np.diag((1.0, -1.0, -1.0)),
    )
    return {
        'snapshot': snapshot,
        'mask': mask,
        'detection': detection,
        'transform': transform,
        'expected_center': np.array((0.0, 0.0, 0.1)),
        'expected_size': np.array((0.10, 0.08, 0.20)),
    }
