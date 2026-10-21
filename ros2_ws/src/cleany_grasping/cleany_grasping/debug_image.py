from __future__ import annotations

from array import array as byte_array
from collections.abc import Sequence

import numpy as np
from PIL import Image as PilImage
from PIL import ImageDraw
from rclpy.time import Time
from sensor_msgs.msg import Image

from cleany_grasping.core.models import PointCloud, RawGrasp


DEFAULT_MAXIMUM_DISPLAYED_CANDIDATES = 8


def _display_candidate_indices(
    candidates: Sequence[RawGrasp],
    selected_index: int | None,
    maximum_displayed_candidates: int,
) -> tuple[int, ...]:
    if maximum_displayed_candidates < 1:
        raise ValueError('At least one grasp candidate must be displayable')
    ranked = sorted(
        range(len(candidates)),
        key=lambda index: (-candidates[index].score, index),
    )
    visible = ranked[:maximum_displayed_candidates]
    if selected_index is not None and selected_index not in visible:
        visible[-1] = selected_index
    return tuple(visible)


def _debug_basis(candidates: Sequence[RawGrasp], points: np.ndarray) -> np.ndarray:
    if candidates:
        horizontal = candidates[0].rotation[:, 1:3]
        return horizontal
    centered = points - np.median(points, axis=0)
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    return vh[:2].T


def render_grasp_debug_image(
    target_cloud: PointCloud,
    context_cloud: PointCloud,
    candidates: Sequence[RawGrasp],
    selected_index: int | None = 0,
    image_size: int = 800,
    maximum_displayed_candidates: int = DEFAULT_MAXIMUM_DISPLAYED_CANDIDATES,
) -> np.ndarray:
    """Render a readable support-plane view of the highest-scoring candidates."""

    if image_size < 200:
        raise ValueError('Grasp debug image must be at least 200 pixels wide')
    if selected_index is not None and not 0 <= selected_index < len(candidates):
        raise ValueError('Selected grasp index is outside the candidate list')
    visible_indices = _display_candidate_indices(
        candidates,
        selected_index,
        maximum_displayed_candidates,
    )
    score_rank = {
        candidate_index: rank
        for rank, candidate_index in enumerate(
            sorted(
                range(len(candidates)),
                key=lambda index: (-candidates[index].score, index),
            ),
            start=1,
        )
    }
    all_points = np.vstack((context_cloud.points, target_cloud.points))
    basis = _debug_basis(candidates, target_cloud.points)
    origin = np.median(target_cloud.points, axis=0)
    projected = (all_points - origin) @ basis
    low = np.percentile(projected, 1.0, axis=0)
    high = np.percentile(projected, 99.0, axis=0)
    extent = np.maximum(high - low, 0.05)
    padding = 54
    scale = float((image_size - 2 * padding) / np.max(extent))
    center = (low + high) / 2.0

    def pixel(points: np.ndarray) -> np.ndarray:
        coordinates = (points - origin) @ basis
        result = (coordinates - center) * scale + image_size / 2.0
        result[:, 1] = image_size - result[:, 1]
        return result

    canvas = PilImage.new('RGB', (image_size, image_size), (20, 23, 28))
    draw = ImageDraw.Draw(canvas)
    context_pixels = pixel(context_cloud.points)
    stride = max(1, context_pixels.shape[0] // 12000)
    sampled = context_pixels[::stride]
    inside = ((sampled[:, 0] >= 0) & (sampled[:, 0] < image_size)
              & (sampled[:, 1] >= 0) & (sampled[:, 1] < image_size))
    draw.point(list(map(tuple, sampled[inside].tolist())), fill=(105, 115, 126))
    target_pixels = pixel(target_cloud.points)
    stride = max(1, target_pixels.shape[0] // 5000)
    for x, y in target_pixels[::stride]:
        if 0 <= x < image_size and 0 <= y < image_size:
            draw.ellipse((x - 2, y - 2, x + 2, y + 2), fill=(255, 145, 48))

    non_selected = [index for index in visible_indices if index != selected_index]
    draw_order = list(reversed(non_selected))
    if selected_index is not None and selected_index in visible_indices:
        draw_order.append(selected_index)
    for index in draw_order:
        candidate = candidates[index]
        contact = candidate.translation + candidate.depth_m * candidate.rotation[:, 0]
        closing = candidate.rotation[:, 1]
        half_span = candidate.width_m / 2.0
        endpoints = np.vstack(
            (contact - half_span * closing, contact + half_span * closing)
        )
        jaw_pixels = pixel(endpoints)
        contact_pixel = pixel(contact.reshape((1, 3)))[0]
        selected = index == selected_index
        rank = score_rank[index]
        if selected:
            color = (72, 236, 118)
        elif rank == 1:
            color = (255, 190, 70)
        elif rank <= 3:
            color = (126, 174, 204)
        else:
            color = (86, 105, 120)
        width = 5 if selected else (3 if rank <= 3 else 2)
        draw.line(tuple(map(tuple, jaw_pixels)), fill=color, width=width)
        for x, y in jaw_pixels:
            radius = 7 if selected else 4
            draw.rectangle(
                (x - radius, y - radius, x + radius, y + radius),
                outline=color,
                width=width,
            )
        x, y = contact_pixel
        draw.ellipse((x - 4, y - 4, x + 4, y + 4), fill=color)
        if selected or rank <= 3:
            label_x, label_y = jaw_pixels[1]
            label = (
                f'SELECTED {candidate.score:.2f}'
                if selected
                else f'#{rank} {candidate.score:.2f}'
            )
            draw.text((label_x + 7, label_y - 7), label, fill=color)

    shown = len(visible_indices)
    draw.rectangle((12, 12, 630, 82), fill=(8, 10, 13))
    draw.text(
        (22, 20),
        f'Grasp candidates - showing {shown} of {len(candidates)} '
        '(top-ranked + selected)',
        fill=(235, 238, 242),
    )
    draw.text(
        (22, 42),
        'orange: target  gray: context  green: selected',
        fill=(200, 207, 214),
    )
    draw.text(
        (22, 61),
        'yellow: best alternative  blue-gray: other top candidates',
        fill=(170, 181, 191),
    )
    if not candidates:
        draw.text((22, 61), 'No collision-free candidate', fill=(255, 92, 92))
    return np.asarray(canvas, dtype=np.uint8)


def debug_image_message(rgb: np.ndarray, stamp_ns: int, frame_id: str) -> Image:
    contiguous = np.ascontiguousarray(rgb, dtype=np.uint8)
    if contiguous.ndim != 3 or contiguous.shape[2] != 3:
        raise ValueError('Debug RGB image must have shape HxWx3')
    message = Image()
    message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    message.header.frame_id = frame_id
    message.height = contiguous.shape[0]
    message.width = contiguous.shape[1]
    message.encoding = 'rgb8'
    message.is_bigendian = False
    message.step = contiguous.shape[1] * 3
    payload = byte_array('B')
    if contiguous.size:
        payload.frombytes(memoryview(contiguous).cast('B'))
    message.data = payload
    return message
