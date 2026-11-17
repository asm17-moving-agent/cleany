"""Shoulder yaw bounds for the motion from the start state to the grasp."""

from collections.abc import Iterator
from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class GraspApproachYawBounds:
    """Front workspace limits in calibrated ROS radians, by commanded arm."""

    left: tuple[float, float]
    right: tuple[float, float]

    def __post_init__(self) -> None:
        for lower, upper in (self.left, self.right):
            if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
                raise ValueError('Grasp approach yaw bounds must be finite and increasing')

    def intersect(self, joint: str, lower: float, upper: float) -> tuple[float, float]:
        if joint == 'left_shoulder_yaw_joint':
            approach = self.left
        elif joint == 'right_shoulder_yaw_joint':
            approach = self.right
        else:
            return lower, upper
        lower, upper = max(lower, approach[0]), min(upper, approach[1])
        if lower >= upper:
            raise ValueError(f'Grasp approach bounds do not overlap the physical range for {joint}')
        return lower, upper


def declare_grasp_approach_yaw(node) -> GraspApproachYawBounds:
    return GraspApproachYawBounds(
        tuple(node.declare_parameter('grasp_approach_left_yaw_rad', [-2.16, 0.0]).value),
        tuple(node.declare_parameter('grasp_approach_right_yaw_rad', [0.0, 2.0033789089786476]).value),
    )


active_grasp_approach_yaw: ContextVar[GraspApproachYawBounds | None] = ContextVar(
    'active_grasp_approach_yaw', default=None)


@contextmanager
def grasp_approach_yaw_scope(bounds: GraspApproachYawBounds | None) -> Iterator[None]:
    """Apply the approach bounds to replanning and dispatch in this call stack."""
    token = active_grasp_approach_yaw.set(bounds)
    try:
        yield
    finally:
        active_grasp_approach_yaw.reset(token)
