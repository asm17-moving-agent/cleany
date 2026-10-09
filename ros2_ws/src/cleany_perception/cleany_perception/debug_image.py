from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from PIL import Image as PilImage
from PIL import ImageDraw
from rclpy.time import Time
from sensor_msgs.msg import Image

from cleany_perception.core.models import Detection2D, ObjectMask, RgbArray


_COLORS = (
    (255, 64, 64),
    (64, 220, 96),
    (64, 128, 255),
    (255, 196, 64),
    (208, 96, 255),
)


def render_debug_image(
    rgb: RgbArray,
    detections: Sequence[Detection2D],
    masks: Sequence[ObjectMask],
    object_ids: Sequence[int] | None = None,
) -> np.ndarray:
    ids = (
        tuple(range(1, len(detections) + 1))
        if object_ids is None
        else tuple(object_ids)
    )
    if len(ids) != len(detections) or any(item < 1 for item in ids):
        raise ValueError(
            'Debug object IDs must match detections and be positive'
        )
    result = np.asarray(rgb, dtype=np.uint8).copy()
    for index, object_mask in enumerate(masks):
        color = np.asarray(_COLORS[index % len(_COLORS)], dtype=np.float32)
        mask = object_mask.mask
        blended = 0.65 * result[mask].astype(np.float32) + 0.35 * color
        result[mask] = np.clip(blended, 0.0, 255.0).astype(np.uint8)

    image = PilImage.fromarray(result, mode='RGB')
    draw = ImageDraw.Draw(image)
    for index, (object_id, detection) in enumerate(zip(ids, detections)):
        color = _COLORS[index % len(_COLORS)]
        box = detection.bbox
        draw.rectangle(
            (box.x_min, box.y_min, box.x_max, box.y_max),
            outline=color,
            width=2,
        )
        draw.text(
            (box.x_min + 2, max(0.0, box.y_min - 12)),
            f'{object_id}: {detection.label} {detection.confidence:.2f}',
            fill=color,
        )
    return np.asarray(image, dtype=np.uint8)


def debug_image_message(
    rgb: RgbArray,
    stamp_ns: int,
    frame_id: str,
) -> Image:
    contiguous = np.ascontiguousarray(rgb, dtype=np.uint8)
    message = Image()
    message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    message.header.frame_id = frame_id
    message.height = contiguous.shape[0]
    message.width = contiguous.shape[1]
    message.encoding = 'rgb8'
    message.is_bigendian = False
    message.step = contiguous.shape[1] * 3
    message.data = contiguous.tobytes()
    return message
