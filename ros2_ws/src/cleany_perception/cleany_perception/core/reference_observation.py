"""Visible RGB-D surface bounds; no table-plane or hidden-volume completion."""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from cleany_perception.core.geometry import deproject_masked_depth
from cleany_perception.core.models import (
    FailureKind, FloatArray, InspectionFailure, MaskArray, RgbdSnapshot, RigidTransform,
)


@dataclass(frozen=True)
class ReferenceObservationConfig:
    minimum_points: int = 30
    maximum_mask_fraction: float = 0.5
    minimum_valid_depth_fraction: float = 0.8
    border_margin_px: int = 2
    minimum_depth_m: float = 0.1
    maximum_depth_m: float = 3.0
    trim_fraction: float = 0.01

    def __post_init__(self):
        if self.minimum_points < 1 or self.border_margin_px < 0:
            raise ValueError('Invalid reference point count/border margin')
        for value in (self.maximum_mask_fraction, self.minimum_valid_depth_fraction):
            if not math.isfinite(value) or not 0. < value <= 1.:
                raise ValueError('Reference mask/depth fractions must be in (0, 1]')
        if (not math.isfinite(self.minimum_depth_m) or not math.isfinite(self.maximum_depth_m)
                or not 0. < self.minimum_depth_m < self.maximum_depth_m):
            raise ValueError('Invalid reference depth range')
        if not math.isfinite(self.trim_fraction) or not 0. <= self.trim_fraction < .5:
            raise ValueError('Invalid reference bounds trim fraction')


@dataclass(frozen=True)
class ObservedSurface:
    center: FloatArray
    extent: FloatArray
    points: FloatArray
    mask_pixels: int
    valid_depth_fraction: float


def observe_surface(snapshot: RgbdSnapshot, mask: MaskArray,
                    transform: RigidTransform,
                    config: ReferenceObservationConfig) -> ObservedSurface:
    mask = np.asarray(mask)
    if mask.dtype != np.bool_ or mask.shape != snapshot.depth_m.shape:
        raise InspectionFailure(FailureKind.MASK, 'Reference mask shape/type differs from current depth')
    rows, columns = np.nonzero(mask)
    count = len(rows)
    if count < config.minimum_points or count > mask.size * config.maximum_mask_fraction:
        raise InspectionFailure(FailureKind.MASK, 'Reference mask is absent, too small, or too large')
    margin = config.border_margin_px
    height, width = mask.shape
    if (rows.min() < margin or columns.min() < margin
            or rows.max() >= height - margin or columns.max() >= width - margin):
        raise InspectionFailure(FailureKind.MASK, 'Reference object is clipped by the image border')
    camera_points = deproject_masked_depth(
        snapshot.depth_m, snapshot.intrinsics, mask,
        config.minimum_depth_m, config.maximum_depth_m)
    fraction = len(camera_points) / count
    if len(camera_points) < config.minimum_points or fraction < config.minimum_valid_depth_fraction:
        raise InspectionFailure(FailureKind.DEPTH, 'Insufficient current depth in the reference mask')
    points = camera_points @ transform.rotation.T + transform.translation
    low, high = np.quantile(points, (config.trim_fraction, 1. - config.trim_fraction), axis=0)
    # These are observed-surface AABB bounds, not a complete object OBB. In
    # particular, no support plane fills the space between a held object and desk.
    return ObservedSurface((low + high) / 2., high - low, points, count, fraction)
