"""Depth-aware autonomous object ranking without ROS dependencies."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from cleany_perception.core.models import (
    Detection2D,
    RgbdSnapshot,
    RigidTransform,
)


@dataclass(frozen=True, slots=True)
class ObjectRankingConfig:
    central_bbox_fraction: float = 0.5
    minimum_valid_depth_pixels: int = 20
    minimum_depth_m: float = 0.1
    maximum_depth_m: float = 3.0

    def __post_init__(self) -> None:
        if (
            not math.isfinite(self.central_bbox_fraction)
            or not 0.0 < self.central_bbox_fraction <= 1.0
        ):
            raise ValueError('central_bbox_fraction must be in (0, 1]')
        if self.minimum_valid_depth_pixels <= 0:
            raise ValueError('minimum_valid_depth_pixels must be positive')
        if (
            not math.isfinite(self.minimum_depth_m)
            or not math.isfinite(self.maximum_depth_m)
            or self.minimum_depth_m <= 0.0
            or self.maximum_depth_m <= self.minimum_depth_m
        ):
            raise ValueError('depth limits must be finite and increasing')


@dataclass(frozen=True, slots=True)
class RankedDetection:
    detection: Detection2D
    distance_m: float | None
    source_index: int

    def __post_init__(self) -> None:
        if self.source_index < 0:
            raise ValueError('source_index must not be negative')
        if self.distance_m is not None and (
            not math.isfinite(self.distance_m) or self.distance_m < 0.0
        ):
            raise ValueError('distance_m must be finite and non-negative')


def _representative_point_in_source_frame(
    snapshot: RgbdSnapshot,
    detection: Detection2D,
    config: ObjectRankingConfig,
) -> np.ndarray | None:
    bbox = detection.bbox
    center_x = 0.5 * (bbox.x_min + bbox.x_max)
    center_y = 0.5 * (bbox.y_min + bbox.y_max)
    half_width = (
        0.5 * (bbox.x_max - bbox.x_min) * config.central_bbox_fraction
    )
    half_height = (
        0.5 * (bbox.y_max - bbox.y_min) * config.central_bbox_fraction
    )
    x_min = max(0, int(math.floor(center_x - half_width)))
    x_max = min(
        snapshot.intrinsics.width,
        int(math.ceil(center_x + half_width)),
    )
    y_min = max(0, int(math.floor(center_y - half_height)))
    y_max = min(
        snapshot.intrinsics.height,
        int(math.ceil(center_y + half_height)),
    )
    if x_min >= x_max or y_min >= y_max:
        return None

    crop = snapshot.depth_m[y_min:y_max, x_min:x_max]
    valid = (
        np.isfinite(crop)
        & (crop >= config.minimum_depth_m)
        & (crop <= config.maximum_depth_m)
    )
    rows, columns = np.nonzero(valid)
    if rows.size < config.minimum_valid_depth_pixels:
        return None

    depth = float(np.median(crop[valid]))
    column = float(x_min) + float(np.median(columns))
    row = float(y_min) + float(np.median(rows))
    intrinsics = snapshot.intrinsics
    return np.array(
        (
            (column - intrinsics.cx) * depth / intrinsics.fx,
            (row - intrinsics.cy) * depth / intrinsics.fy,
            depth,
        ),
        dtype=np.float64,
    )


def rank_detections_by_distance(
    snapshot: RgbdSnapshot,
    detections: Sequence[Detection2D],
    capture_transform: RigidTransform,
    config: ObjectRankingConfig = ObjectRankingConfig(),
) -> tuple[RankedDetection, ...]:
    """Rank selectable detections by target-frame origin distance.

    Invalid-depth detections remain visible for diagnostics but are placed
    after every selectable detection. Equal distances prefer higher detector
    confidence and then retain the detector's original order.
    """

    ranked: list[RankedDetection] = []
    for source_index, detection in enumerate(detections):
        source_point = _representative_point_in_source_frame(
            snapshot,
            detection,
            config,
        )
        distance_m = None
        if source_point is not None:
            target_point = (
                capture_transform.rotation @ source_point
                + capture_transform.translation
            )
            distance_m = float(np.linalg.norm(target_point))
        ranked.append(
            RankedDetection(
                detection=detection,
                distance_m=distance_m,
                source_index=source_index,
            )
        )
    ranked.sort(
        key=lambda item: (
            item.distance_m is None,
            item.distance_m if item.distance_m is not None else math.inf,
            -item.detection.confidence,
            item.source_index,
        )
    )
    return tuple(ranked)
