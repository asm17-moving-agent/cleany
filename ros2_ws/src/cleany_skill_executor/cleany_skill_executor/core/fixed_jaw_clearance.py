"""Conservative target/fixed-finger separation, without an aperture-fit gate."""
from __future__ import annotations

from dataclasses import dataclass
from itertools import product
import math
from typing import Sequence

import numpy as np

from .alignment import rotation_from_quaternion
from .physical_grasp_selection import PoseValue


@dataclass(frozen=True)
class Bounds:
    minimum: tuple[float, float, float]
    maximum: tuple[float, float, float]

    def __post_init__(self) -> None:
        if (len(self.minimum) != 3 or len(self.maximum) != 3
                or not all(math.isfinite(v) for v in (*self.minimum, *self.maximum))
                or any(a >= b for a, b in zip(self.minimum, self.maximum))):
            raise ValueError('Invalid fixed-jaw bounds')


@dataclass(frozen=True)
class TargetBox:
    pose: PoseValue
    size: tuple[float, float, float]

    def __post_init__(self) -> None:
        if (len(self.size) != 3 or any(not math.isfinite(v) or v <= 0 for v in self.size)
                or len(self.pose.position) != 3
                or not all(math.isfinite(v) for v in self.pose.position)):
            raise ValueError('Invalid target box for fixed-jaw clearance')
        rotation_from_quaternion(self.pose.orientation_xyzw)

    def corners(self) -> np.ndarray:
        local = np.asarray(tuple(product((-1., 1.), repeat=3))) * np.asarray(self.size) / 2
        return local @ rotation_from_quaternion(self.pose.orientation_xyzw).T + self.pose.position


@dataclass(frozen=True)
class ClearanceConfig:
    minimum_m: float
    maximum_correction_m: float

    def __post_init__(self) -> None:
        if (not math.isfinite(self.minimum_m) or self.minimum_m < 0
                or not math.isfinite(self.maximum_correction_m)
                or self.maximum_correction_m < 0):
            raise ValueError('Fixed-jaw clearance and correction must be finite and nonnegative')


def closing_clearance(jaw: Bounds, corners: np.ndarray, poses: Sequence[PoseValue],
                      sign: float = 1.) -> float | None:
    """Closing-axis gap if the swept target overlaps the finger in Y and Z.

    Union of endpoint projections bounds a straight, fixed-orientation approach.
    This is a separating-plane margin, not an exact mesh distance. None means
    the target misses the fixed finger. No moving-jaw test.
    """
    if sign not in (-1., 1.) or not poses:
        raise ValueError('Invalid fixed-jaw side or empty approach')
    corners = np.asarray(corners, dtype=float)
    if corners.shape != (8, 3) or not np.isfinite(corners).all():
        raise ValueError('Invalid target corners')
    if any(len(pose.position) != 3 or not all(math.isfinite(v) for v in pose.position)
           for pose in poses):
        raise ValueError('Invalid approach pose')
    projections = np.concatenate([
        (corners - pose.position) @ rotation_from_quaternion(pose.orientation_xyzw)
        for pose in poses
    ])
    lower, upper = projections.min(axis=0), projections.max(axis=0)
    if any(upper[i] < jaw.minimum[i] or lower[i] > jaw.maximum[i] for i in (1, 2)):
        return None
    # An object outside the fixed finger also needs no centering correction.
    # Do not require it to lie in the gripper aperture.
    if (sign > 0 and lower[0] > jaw.maximum[0]) or (sign < 0 and upper[0] < jaw.minimum[0]):
        return None
    return float(jaw.minimum[0] - upper[0] if sign > 0 else lower[0] - jaw.maximum[0])


def correction_for_clearance(clearance: float | None, config: ClearanceConfig) -> float:
    if clearance is not None and not math.isfinite(clearance):
        raise ValueError('Nonfinite fixed-jaw clearance')
    correction = 0. if clearance is None else max(0., config.minimum_m - clearance)
    if not math.isfinite(correction) or correction > config.maximum_correction_m + 1e-9:
        raise ValueError(
            f'Fixed-jaw clearance requires {correction * 1000:.3f}mm correction; '
            f'limit={config.maximum_correction_m * 1000:.3f}mm')
    return correction
