from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from PIL import Image as PilImage
from PIL import ImageDraw
from rclpy.time import Time
from sensor_msgs.msg import Image

from cleany_perception.core.models import (
    CameraIntrinsics,
    Detection2D,
    ObjectMask,
    OrientedBox3D,
    RgbArray,
)


_COLORS = (
    (255, 64, 64),
    (64, 220, 96),
    (64, 128, 255),
    (255, 196, 64),
    (208, 96, 255),
)
_OBB_EDGES = (
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 0),
    (4, 5),
    (5, 6),
    (6, 7),
    (7, 4),
    (0, 4),
    (1, 5),
    (2, 6),
    (3, 7),
)
_OBB_SIGNS = np.asarray(
    (
        (-1.0, -1.0, -1.0),
        (1.0, -1.0, -1.0),
        (1.0, 1.0, -1.0),
        (-1.0, 1.0, -1.0),
        (-1.0, -1.0, 1.0),
        (1.0, -1.0, 1.0),
        (1.0, 1.0, 1.0),
        (-1.0, 1.0, 1.0),
    ),
    dtype=np.float64,
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

    image = PilImage.fromarray(result)
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


def render_obb_debug_image(
    rgb: RgbArray,
    detection: Detection2D,
    object_mask: ObjectMask,
    box_in_camera: OrientedBox3D,
    intrinsics: CameraIntrinsics,
    object_id: int,
) -> np.ndarray:
    """Overlay a camera-frame 3D OBB, local axes and metric values."""
    result = render_debug_image(
        rgb,
        (detection,),
        (object_mask,),
        object_ids=(object_id,),
    )
    local_corners = _OBB_SIGNS * (0.5 * box_in_camera.size)
    corners = (
        box_in_camera.center
        + local_corners @ box_in_camera.rotation.T
    )
    projected = tuple(
        _project_camera_point(point, intrinsics) for point in corners
    )

    image = PilImage.fromarray(result)
    draw = ImageDraw.Draw(image)
    for start, end in _OBB_EDGES:
        if projected[start] is not None and projected[end] is not None:
            draw.line(
                (projected[start], projected[end]),
                fill=(255, 255, 0),
                width=3,
            )

    center_pixel = _project_camera_point(box_in_camera.center, intrinsics)
    if center_pixel is not None:
        axis_colors = ((255, 64, 64), (64, 255, 64), (64, 128, 255))
        for axis, color in enumerate(axis_colors):
            endpoint = (
                box_in_camera.center
                + box_in_camera.rotation[:, axis]
                * 0.5
                * box_in_camera.size[axis]
            )
            endpoint_pixel = _project_camera_point(endpoint, intrinsics)
            if endpoint_pixel is not None:
                draw.line(
                    (center_pixel, endpoint_pixel),
                    fill=color,
                    width=4,
                )
        draw.ellipse(
            (
                center_pixel[0] - 4,
                center_pixel[1] - 4,
                center_pixel[0] + 4,
                center_pixel[1] + 4,
            ),
            fill=(255, 255, 255),
            outline=(0, 0, 0),
        )
        center = box_in_camera.center
        size = box_in_camera.size
        text_y = min(intrinsics.height - 28, center_pixel[1] + 7)
        draw.text(
            (max(0.0, center_pixel[0] - 100), max(0.0, text_y)),
            f'xyz=({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f})m',
            fill=(255, 255, 255),
            stroke_width=2,
            stroke_fill=(0, 0, 0),
        )
        draw.text(
            (max(0.0, center_pixel[0] - 100), max(0.0, text_y + 12)),
            f'size=({size[0]:.3f}, {size[1]:.3f}, {size[2]:.3f})m',
            fill=(255, 255, 255),
            stroke_width=2,
            stroke_fill=(0, 0, 0),
        )
    return np.asarray(image, dtype=np.uint8)


def _project_camera_point(
    point: np.ndarray,
    intrinsics: CameraIntrinsics,
) -> tuple[float, float] | None:
    x, y, z = np.asarray(point, dtype=np.float64)
    if not np.isfinite((x, y, z)).all() or z <= 1e-6:
        return None
    u = intrinsics.fx * x / z + intrinsics.cx
    v = intrinsics.fy * y / z + intrinsics.cy
    limit_x = float(intrinsics.width * 4)
    limit_y = float(intrinsics.height * 4)
    return (
        float(np.clip(u, -limit_x, limit_x)),
        float(np.clip(v, -limit_y, limit_y)),
    )


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
