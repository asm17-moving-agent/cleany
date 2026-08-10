import numpy as np
import pytest

from cleany_perception.core.models import ObjectMask
from cleany_perception.debug_image import (
    debug_image_message,
    render_debug_image,
)


def test_debug_image_overlays_mask_bbox_and_preserves_contract(
    synthetic_scene,
):
    snapshot = synthetic_scene['snapshot']
    detection = synthetic_scene['detection']
    object_mask = ObjectMask(
        detection=detection,
        mask=synthetic_scene['mask'],
        score=0.9,
    )

    debug = render_debug_image(snapshot.rgb, [detection], [object_mask])
    message = debug_image_message(
        debug,
        snapshot.stamp_ns,
        'rgb_optical_frame',
    )

    assert debug.shape == snapshot.rgb.shape
    assert debug.dtype == np.uint8
    assert not np.array_equal(debug, snapshot.rgb)
    assert (message.width, message.height, message.encoding) == (
        320,
        240,
        'rgb8',
    )
    assert message.header.frame_id == 'rgb_optical_frame'
    assert message.header.stamp.sec == 1
    assert message.header.stamp.nanosec == 500_000_000


def test_debug_image_preserves_selected_snapshot_object_id(synthetic_scene):
    snapshot = synthetic_scene['snapshot']
    detection = synthetic_scene['detection']

    default_number = render_debug_image(snapshot.rgb, [detection], [])
    selected_number = render_debug_image(
        snapshot.rgb,
        [detection],
        [],
        object_ids=[2],
    )

    assert not np.array_equal(default_number, selected_number)


def test_debug_image_rejects_mismatched_object_ids(synthetic_scene):
    with pytest.raises(ValueError, match='must match detections'):
        render_debug_image(
            synthetic_scene['snapshot'].rgb,
            [synthetic_scene['detection']],
            [],
            object_ids=[],
        )
