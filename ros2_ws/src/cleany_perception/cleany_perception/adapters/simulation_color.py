"""Deterministic color perception adapter for rendered MuJoCo scenes."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from cleany_perception.core.models import (
    BoundingBox2D,
    Detection2D,
    ObjectMask,
    RgbArray,
)


@dataclass(frozen=True, slots=True)
class _ColorClass:
    label: str
    channel: int
    minimum_value: float
    other_channel_ratios: tuple[float, float]


_COLOR_CLASSES = (
    _ColorClass('red_can', 0, 140.0, (1.55, 1.70)),
    _ColorClass('blue_box', 2, 120.0, (1.35, 1.35)),
)


def _color_mask(rgb: RgbArray, color: _ColorClass) -> np.ndarray:
    values = np.asarray(rgb, dtype=np.float32)
    selected = values[:, :, color.channel]
    other_channels = tuple(
        index for index in range(3) if index != color.channel
    )
    first_other = values[:, :, other_channels[0]]
    second_other = values[:, :, other_channels[1]]
    return (
        (selected > color.minimum_value)
        & (selected > color.other_channel_ratios[0] * first_other)
        & (selected > color.other_channel_ratios[1] * second_other)
    )


class SimulationColorDetector:
    """Return bboxes measured from rendered red and blue pixels."""

    def __init__(self, minimum_pixels: int = 100) -> None:
        if minimum_pixels <= 0:
            raise ValueError('minimum_pixels must be positive')
        self._minimum_pixels = minimum_pixels

    def detect(
        self,
        rgb: RgbArray,
        _query: str,
    ) -> Sequence[Detection2D]:
        detections = []
        for color in _COLOR_CLASSES:
            rows, columns = np.nonzero(_color_mask(rgb, color))
            if rows.size < self._minimum_pixels:
                continue
            detections.append(
                Detection2D(
                    label=color.label,
                    confidence=1.0,
                    bbox=BoundingBox2D(
                        x_min=float(columns.min()),
                        y_min=float(rows.min()),
                        x_max=float(columns.max() + 1),
                        y_max=float(rows.max() + 1),
                    ),
                )
            )
        return tuple(detections)


class SimulationColorSegmenter:
    """Return full-resolution rendered color masks for selected bboxes."""

    def segment(
        self,
        rgb: RgbArray,
        detections: Sequence[Detection2D],
    ) -> Sequence[ObjectMask]:
        colors = {color.label: color for color in _COLOR_CLASSES}
        masks = []
        height, width = rgb.shape[:2]
        for detection in detections:
            color = colors.get(detection.label)
            if color is None:
                raise ValueError(
                    f'unsupported simulation color label: {detection.label}'
                )
            mask = _color_mask(rgb, color)
            bbox_mask = np.zeros((height, width), dtype=np.bool_)
            x_min = max(0, int(np.floor(detection.bbox.x_min)))
            y_min = max(0, int(np.floor(detection.bbox.y_min)))
            x_max = min(width, int(np.ceil(detection.bbox.x_max)))
            y_max = min(height, int(np.ceil(detection.bbox.y_max)))
            bbox_mask[y_min:y_max, x_min:x_max] = True
            masks.append(
                ObjectMask(
                    detection=detection,
                    mask=mask & bbox_mask,
                    score=1.0,
                )
            )
        return tuple(masks)
