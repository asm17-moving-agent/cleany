from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from cleany_perception.core.models import (
    BoundingBox2D,
    CameraIntrinsics,
    Detection2D,
    RgbdSnapshot,
    RigidTransform,
)
from cleany_perception.core.object_ranking import (
    ObjectRankingConfig,
    rank_detections_by_distance,
)


def _scene():
    intrinsics = CameraIntrinsics(
        width=100,
        height=80,
        fx=100.0,
        fy=100.0,
        cx=49.5,
        cy=39.5,
    )
    depth = np.full((80, 100), np.nan, dtype=np.float32)
    depth[20:50, 10:40] = 0.7
    depth[20:50, 60:90] = 1.4
    snapshot = RgbdSnapshot(
        rgb=np.zeros((80, 100, 3), dtype=np.uint8),
        depth_m=depth,
        intrinsics=intrinsics,
        stamp_ns=1,
        source_frame='camera_optical_frame',
    )
    near = Detection2D(
        label='near',
        confidence=0.7,
        bbox=BoundingBox2D(10.0, 20.0, 40.0, 50.0),
    )
    far = Detection2D(
        label='far',
        confidence=0.99,
        bbox=BoundingBox2D(60.0, 20.0, 90.0, 50.0),
    )
    transform = RigidTransform(
        translation=np.zeros(3),
        rotation=np.eye(3),
    )
    config = ObjectRankingConfig(minimum_valid_depth_pixels=4)
    return snapshot, near, far, transform, config


def test_nearest_target_frame_object_precedes_higher_confidence_object():
    snapshot, near, far, transform, config = _scene()

    ranked = rank_detections_by_distance(
        snapshot,
        (far, near),
        transform,
        config,
    )

    assert [item.detection.label for item in ranked] == ['near', 'far']
    assert ranked[0].distance_m is not None
    assert ranked[1].distance_m is not None
    assert ranked[0].distance_m < ranked[1].distance_m


def test_invalid_depth_detection_is_retained_but_ranked_last():
    snapshot, near, _far, transform, config = _scene()
    invalid = replace(
        near,
        label='invalid',
        confidence=1.0,
        bbox=BoundingBox2D(42.0, 20.0, 58.0, 50.0),
    )

    ranked = rank_detections_by_distance(
        snapshot,
        (invalid, near),
        transform,
        config,
    )

    assert [item.detection.label for item in ranked] == ['near', 'invalid']
    assert ranked[0].distance_m is not None
    assert ranked[1].distance_m is None


def test_equal_distance_prefers_confidence_then_detector_order():
    snapshot, near, _far, transform, config = _scene()
    high = replace(near, label='high', confidence=0.9)
    low = replace(near, label='low', confidence=0.4)
    equal = replace(near, label='equal', confidence=0.9)

    ranked = rank_detections_by_distance(
        snapshot,
        (equal, low, high),
        transform,
        config,
    )

    assert [item.detection.label for item in ranked] == [
        'equal',
        'high',
        'low',
    ]


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('central_bbox_fraction', 0.0),
        ('minimum_valid_depth_pixels', 0),
        ('minimum_depth_m', -0.1),
        ('maximum_depth_m', 0.05),
    ],
)
def test_object_ranking_config_rejects_invalid_values(field, value):
    values = {
        'central_bbox_fraction': 0.5,
        'minimum_valid_depth_pixels': 20,
        'minimum_depth_m': 0.1,
        'maximum_depth_m': 3.0,
    }
    values[field] = value

    with pytest.raises(ValueError):
        ObjectRankingConfig(**values)
