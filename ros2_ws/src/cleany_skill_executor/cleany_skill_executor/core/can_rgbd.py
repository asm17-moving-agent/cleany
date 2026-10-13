"""Pure RGB-D red-can segmentation and projection for the MuJoCo demo."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from PIL import Image as PilImage
from PIL import ImageDraw


@dataclass(frozen=True, slots=True)
class CameraProjection:
    fx: float
    fy: float
    cx: float
    cy: float
    translation_base: tuple[float, float, float]
    rotation_base_from_optical: tuple[float, ...]

    def __post_init__(self) -> None:
        intrinsics = (self.fx, self.fy, self.cx, self.cy)
        if not all(math.isfinite(value) for value in intrinsics):
            raise ValueError('camera intrinsics must be finite')
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError('camera focal lengths must be positive')
        translation = np.asarray(self.translation_base, dtype=float)
        if translation.shape != (3,) or not np.isfinite(translation).all():
            raise ValueError('camera translation must contain three finite values')
        rotation = np.asarray(self.rotation_base_from_optical, dtype=float)
        if rotation.shape != (9,) or not np.isfinite(rotation).all():
            raise ValueError('camera rotation must contain nine finite values')
        matrix = rotation.reshape((3, 3))
        if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1.0e-6):
            raise ValueError('camera rotation must be orthonormal')


@dataclass(frozen=True, slots=True)
class SegmentedCanCloud:
    target_points: np.ndarray
    target_colors: np.ndarray
    context_points: np.ndarray
    context_colors: np.ndarray


def _project(
    rows: np.ndarray,
    columns: np.ndarray,
    depth: np.ndarray,
    camera: CameraProjection,
) -> np.ndarray:
    z = depth[rows, columns]
    optical = np.column_stack(
        (
            (columns - camera.cx) * z / camera.fx,
            (rows - camera.cy) * z / camera.fy,
            z,
        )
    )
    rotation = np.asarray(camera.rotation_base_from_optical).reshape((3, 3))
    translation = np.asarray(camera.translation_base)
    return optical @ rotation.T + translation


def _limited_indices(count: int, maximum: int) -> np.ndarray:
    if count <= maximum:
        return np.arange(count)
    return np.linspace(0, count - 1, maximum, dtype=int)


def segment_red_can(
    rgb: np.ndarray,
    depth_m: np.ndarray,
    camera: CameraProjection,
    *,
    context_margin_pixels: int = 80,
    minimum_target_pixels: int = 100,
    target_maximum_points: int = 12000,
    context_maximum_points: int = 30000,
) -> SegmentedCanCloud:
    """Segment rendered red can and reconstruct aligned base-frame clouds."""

    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError('RGB image must have shape HxWx3')
    if depth_m.shape != rgb.shape[:2]:
        raise ValueError('depth image shape must match RGB')
    if context_margin_pixels < 0:
        raise ValueError('context margin must be non-negative')
    if minimum_target_pixels <= 0:
        raise ValueError('minimum target pixels must be positive')
    if target_maximum_points <= 0 or context_maximum_points <= 0:
        raise ValueError('point-cloud limits must be positive')
    finite = np.isfinite(depth_m) & (depth_m > 0.1) & (depth_m < 3.0)
    red = rgb[:, :, 0].astype(float)
    green = rgb[:, :, 1].astype(float)
    blue = rgb[:, :, 2].astype(float)
    target_mask = (
        finite
        & (red > 140.0)
        & (red > 1.55 * green)
        & (red > 1.70 * blue)
    )
    target_rows, target_columns = np.nonzero(target_mask)
    if target_rows.size < minimum_target_pixels:
        raise ValueError(
            f'red can segmentation found only {target_rows.size} pixels'
        )

    row_min = max(0, int(target_rows.min()) - context_margin_pixels)
    row_max = min(
        rgb.shape[0], int(target_rows.max()) + context_margin_pixels + 1
    )
    column_min = max(0, int(target_columns.min()) - context_margin_pixels)
    column_max = min(
        rgb.shape[1], int(target_columns.max()) + context_margin_pixels + 1
    )
    context_mask = np.zeros_like(finite)
    context_mask[row_min:row_max, column_min:column_max] = True
    context_rows, context_columns = np.nonzero(context_mask & finite)

    target_selection = _limited_indices(
        target_rows.size, target_maximum_points
    )
    context_selection = _limited_indices(
        context_rows.size, context_maximum_points
    )
    target_rows = target_rows[target_selection]
    target_columns = target_columns[target_selection]
    context_rows = context_rows[context_selection]
    context_columns = context_columns[context_selection]
    return SegmentedCanCloud(
        target_points=_project(
            target_rows, target_columns, depth_m, camera
        ),
        target_colors=rgb[target_rows, target_columns],
        context_points=_project(
            context_rows, context_columns, depth_m, camera
        ),
        context_colors=rgb[context_rows, context_columns],
    )


def render_grasp_overlay(
    rgb: np.ndarray,
    camera: CameraProjection,
    tcp_positions: np.ndarray,
    approach_directions: np.ndarray,
    scores: np.ndarray,
    openings_m: np.ndarray,
    *,
    selected_index: int | None = None,
    selected_arm: str = '',
    pregrasp_offset_m: float = 0.08,
) -> np.ndarray:
    """Draw grasp directions and numeric angles over the rendered RGB image."""

    positions = np.asarray(tcp_positions, dtype=float)
    approaches = np.asarray(approach_directions, dtype=float)
    count = len(positions)
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError('RGB image must have shape HxWx3')
    if positions.shape != (count, 3) or approaches.shape != (count, 3):
        raise ValueError('grasp positions and approaches must have shape Nx3')
    if (
        np.asarray(scores).shape != (count,)
        or np.asarray(openings_m).shape != (count,)
    ):
        raise ValueError(
            'scores and openings must contain one value per grasp'
        )
    if selected_index is not None and not 0 <= selected_index < count:
        raise ValueError('selected grasp index is outside the candidate list')
    if not math.isfinite(pregrasp_offset_m) or pregrasp_offset_m <= 0.0:
        raise ValueError('pregrasp offset must be positive and finite')

    norms = np.linalg.norm(approaches, axis=1)
    if np.any(~np.isfinite(norms)) or np.any(norms <= 1.0e-9):
        raise ValueError('grasp approaches must be finite non-zero vectors')
    unit = approaches / norms[:, None]
    pregrasp = positions - pregrasp_offset_m * unit
    base_points = np.vstack((pregrasp, positions))
    rotation = np.asarray(camera.rotation_base_from_optical).reshape((3, 3))
    optical = (base_points - np.asarray(camera.translation_base)) @ rotation
    pixels = np.column_stack(
        (
            camera.fx * optical[:, 0] / optical[:, 2] + camera.cx,
            camera.fy * optical[:, 1] / optical[:, 2] + camera.cy,
        )
    )
    pre_pixels, tcp_pixels = pixels[:count], pixels[count:]

    canvas = PilImage.fromarray(np.ascontiguousarray(rgb, dtype=np.uint8))
    draw = ImageDraw.Draw(canvas)
    for index in reversed(range(count)):
        selected = index == selected_index
        color = (40, 255, 80) if selected else (20, 210, 255)
        width = 6 if selected else 3
        start = tuple(float(value) for value in pre_pixels[index])
        end = tuple(float(value) for value in tcp_pixels[index])
        draw.line((start, end), fill=color, width=width)
        direction = pre_pixels[index] - tcp_pixels[index]
        length = float(np.linalg.norm(direction))
        if length > 1.0:
            direction /= length
            normal = np.array((-direction[1], direction[0]))
            tip = tcp_pixels[index]
            wing_a = tip + 13.0 * direction + 7.0 * normal
            wing_b = tip + 13.0 * direction - 7.0 * normal
            draw.polygon(
                (tuple(tip), tuple(wing_a), tuple(wing_b)), fill=color
            )
        x, y = tcp_pixels[index]
        draw.ellipse((x - 7, y - 7, x + 7, y + 7), outline=color, width=width)
        azimuth = math.degrees(math.atan2(unit[index, 1], unit[index, 0]))
        elevation = math.degrees(
            math.atan2(
                unit[index, 2],
                math.hypot(unit[index, 0], unit[index, 1]),
            )
        )
        draw.text(
            (x + 10, y + 7),
            (
                f'C{index} score={float(scores[index]):.2f} '
                f'az={azimuth:+.1f} el={elevation:+.1f} '
                f'open={float(openings_m[index]) * 1000.0:.0f}mm'
            ),
            fill=color,
            stroke_width=2,
            stroke_fill=(0, 0, 0),
        )

    draw.rectangle((8, 8, min(rgb.shape[1] - 8, 525), 72), fill=(8, 10, 13))
    title = 'CAN GRASP: cyan=candidate  green=MoveIt-selected'
    draw.text((18, 16), title, fill=(240, 244, 248))
    state = 'evaluating MoveIt reachability'
    if selected_index is not None:
        state = (
            f'SELECTED C{selected_index} / {selected_arm.upper()} / '
            f'collision-checked pregrasp={pregrasp_offset_m * 1000:.0f}mm'
        )
    draw.text((18, 42), state, fill=(90, 255, 120))
    return np.asarray(canvas, dtype=np.uint8)
