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
    minimum_rgb: tuple[float, float, float]
    maximum_rgb: tuple[float, float, float]
    ratios: tuple[tuple[int, int, float], ...] = ()
    maximum_channel_spread: float | None = None


_COLOR_PROFILES = {
    'legacy': (
        _ColorClass(
            'red_can',
            (140.0, 0.0, 0.0),
            (255.0, 255.0, 255.0),
            ((0, 1, 1.55), (0, 2, 1.70)),
        ),
        _ColorClass(
            'blue_box',
            (0.0, 0.0, 120.0),
            (255.0, 255.0, 255.0),
            ((2, 0, 1.35), (2, 1, 1.35)),
        ),
    ),
    'study_cafe': (
        _ColorClass(
            'cup',
            (0.0, 0.0, 60.0),
            (255.0, 255.0, 255.0),
            ((2, 0, 1.50), (2, 1, 1.50)),
        ),
        _ColorClass(
            'wallet',
            (45.0, 0.0, 0.0),
            (180.0, 255.0, 255.0),
            ((0, 1, 1.80), (1, 2, 1.70)),
        ),
        _ColorClass(
            'phone',
            (0.0, 0.0, 0.0),
            (40.0, 40.0, 40.0),
            maximum_channel_spread=5.0,
        ),
        _ColorClass(
            'box',
            (7.0, 0.0, 0.0),
            (255.0, 255.0, 255.0),
            ((0, 1, 1.50), (2, 1, 1.30)),
        ),
    ),
}


def _color_mask(rgb: RgbArray, color: _ColorClass) -> np.ndarray:
    values = np.asarray(rgb, dtype=np.float32)
    mask = np.all(values >= np.asarray(color.minimum_rgb), axis=2)
    mask &= np.all(values <= np.asarray(color.maximum_rgb), axis=2)
    for numerator, denominator, ratio in color.ratios:
        mask &= values[:, :, numerator] > ratio * values[:, :, denominator]
    if color.maximum_channel_spread is not None:
        mask &= (
            values.max(axis=2) - values.min(axis=2)
            <= color.maximum_channel_spread
        )
    return mask


def _classes(profile: str) -> tuple[_ColorClass, ...]:
    try:
        return _COLOR_PROFILES[profile]
    except KeyError as error:
        raise ValueError(
            f'unsupported simulation color profile: {profile}'
        ) from error


class SimulationColorDetector:
    """Return bboxes measured from rendered red and blue pixels."""

    def __init__(
        self,
        minimum_pixels: int = 100,
        profile: str = 'legacy',
    ) -> None:
        if minimum_pixels <= 0:
            raise ValueError('minimum_pixels must be positive')
        self._minimum_pixels = minimum_pixels
        self._colors = _classes(profile)

    def detect(
        self,
        rgb: RgbArray,
        _query: str,
    ) -> Sequence[Detection2D]:
        detections = []
        for color in self._colors:
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

    def __init__(self, profile: str = 'legacy') -> None:
        self._colors = _classes(profile)

    def segment(
        self,
        rgb: RgbArray,
        detections: Sequence[Detection2D],
    ) -> Sequence[ObjectMask]:
        colors = {color.label: color for color in self._colors}
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
