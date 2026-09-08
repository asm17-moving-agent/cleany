"""ROS-independent safety gates for the refreshed grasp pipeline."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from cleany_skill_executor.core.grasp_selection import (
    directed_axis_error_deg,
    unsigned_axis_error_deg,
)


Vector3 = tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class ObservedGrasp:
    """Geometry needed to associate and gate a refreshed observation."""

    key: int
    center: Vector3
    approach: Vector3
    closing: Vector3


@dataclass(frozen=True, slots=True)
class ReinspectionLimits:
    maximum_center_shift_m: float = 0.030
    maximum_axis_change_deg: float = 15.0
    ambiguity_distance_m: float = 0.010

    def __post_init__(self) -> None:
        values = (
            self.maximum_center_shift_m,
            self.maximum_axis_change_deg,
            self.ambiguity_distance_m,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError('reinspection limits must be finite and non-negative')


def associate_refreshed_grasp(
    previous: ObservedGrasp,
    candidates: Sequence[ObservedGrasp],
    limits: ReinspectionLimits = ReinspectionLimits(),
) -> ObservedGrasp:
    """Return the unique nearest observation after enforcing motion gates."""
    if not candidates:
        raise ValueError('same-label object was not found in refreshed RGB-D')
    ranked = sorted(
        ((_distance(previous.center, item.center), item) for item in candidates),
        key=lambda pair: (pair[0], pair[1].key),
    )
    distance, selected = ranked[0]
    if distance > limits.maximum_center_shift_m:
        raise ValueError(
            f'refreshed OBB center shifted {distance:.4f}m '
            f'(maximum {limits.maximum_center_shift_m:.4f}m)'
        )
    if len(ranked) > 1 and ranked[1][0] - distance <= limits.ambiguity_distance_m:
        raise ValueError('refreshed same-label association is ambiguous')
    approach_error = directed_axis_error_deg(
        selected.approach, previous.approach
    )
    closing_error = unsigned_axis_error_deg(
        selected.closing, previous.closing
    )
    if max(approach_error, closing_error) > limits.maximum_axis_change_deg:
        raise ValueError(
            'refreshed grasp axes changed too far: '
            f'approach={approach_error:.2f}deg '
            f'closing={closing_error:.2f}deg'
        )
    return selected


def linear_approach_error_deg(
    start: Vector3,
    goal: Vector3,
    approach: Vector3,
) -> float:
    direction = tuple(end - begin for begin, end in zip(start, goal, strict=True))
    if _norm(direction) <= 1.0e-9:
        return 0.0
    return directed_axis_error_deg(direction, approach)


@dataclass(frozen=True, slots=True)
class ContactSample:
    tcp_distance_m: float
    position_errors_rad: tuple[float, ...]
    velocities_rad_s: tuple[float, ...]
    efforts: tuple[float, ...] | None = None


@dataclass(frozen=True, slots=True)
class ContactLimits:
    maximum_tcp_distance_m: float = 0.020
    minimum_joint_error_rad: float = 0.10
    maximum_joint_velocity_rad_s: float = 0.05
    consecutive_samples: int = 5
    effort_threshold: float | None = None

    def __post_init__(self) -> None:
        if self.consecutive_samples <= 0:
            raise ValueError('consecutive_samples must be positive')
        numeric = (
            self.maximum_tcp_distance_m,
            self.minimum_joint_error_rad,
            self.maximum_joint_velocity_rad_s,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in numeric):
            raise ValueError('contact limits must be finite and non-negative')
        if self.effort_threshold is not None and (
            not math.isfinite(self.effort_threshold)
            or self.effort_threshold <= 0.0
        ):
            raise ValueError('effort threshold must be positive when enabled')


class ContactDebounce:
    """Debounce position/velocity contact, with optional effort augmentation."""

    def __init__(self, limits: ContactLimits = ContactLimits()) -> None:
        self._limits = limits
        self._count = 0

    def update(self, sample: ContactSample) -> bool:
        position_contact = (
            sample.position_errors_rad
            and max(abs(value) for value in sample.position_errors_rad)
            >= self._limits.minimum_joint_error_rad
        )
        effort_contact = (
            self._limits.effort_threshold is not None
            and sample.efforts is not None
            and any(
                abs(value) >= self._limits.effort_threshold
                for value in sample.efforts
            )
        )
        stopped = (
            sample.velocities_rad_s
            and max(abs(value) for value in sample.velocities_rad_s)
            <= self._limits.maximum_joint_velocity_rad_s
        )
        accepted = (
            math.isfinite(sample.tcp_distance_m)
            and sample.tcp_distance_m <= self._limits.maximum_tcp_distance_m
            and stopped
            and (position_contact or effort_contact)
        )
        self._count = self._count + 1 if accepted else 0
        return self._count >= self._limits.consecutive_samples


def _norm(value: Vector3) -> float:
    return math.sqrt(sum(component * component for component in value))


def _distance(left: Vector3, right: Vector3) -> float:
    return _norm(tuple(a - b for a, b in zip(left, right, strict=True)))
