"""Pose-error evidence without changing IK selection or acceptance."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .physical_grasp_selection import PoseAlignment


@dataclass(frozen=True, slots=True)
class AlignmentLimits:
    position_m: float
    approach_degrees: float
    closing_degrees: float

    def as_dict(self) -> dict:
        return {
            'position_error_mm': self.position_m * 1000.,
            'approach_error_degrees': self.approach_degrees,
            'closing_error_degrees': self.closing_degrees,
        }


def alignment_errors(item: PoseAlignment, limits: AlignmentLimits) -> dict:
    values = (item.position_error_m * 1000., item.approach_error_degrees,
              item.closing_error_degrees)
    errors = dict(zip(limits.as_dict(),
                      (float(v) if math.isfinite(v) else None for v in values)))
    violations = [name for name, value in errors.items()
                  if value is None or value > limits.as_dict()[name]]
    def finite_values(values: Sequence[float]) -> list[float | None]:
        return [float(value) if math.isfinite(value) else None for value in values]
    return {
        **errors, 'violations': violations,
        'joint_names': list(item.solution.names),
        'joint_positions_rad': list(item.solution.positions),
        'fk_position_m': finite_values(item.pose.position),
        'fk_orientation_xyzw': finite_values(item.pose.orientation_xyzw),
    }


def alignment_rank(errors: dict, limits: AlignmentLimits) -> tuple[float, float]:
    ratios = [errors[name] / limit if errors[name] is not None else math.inf
              for name, limit in limits.as_dict().items()]
    return max(ratios), sum(ratios)


def summarize_alignments(items: Sequence[PoseAlignment], limits: AlignmentLimits) -> dict:
    errors = [alignment_errors(item, limits) for item in items]
    closest = min(errors, key=lambda value: alignment_rank(value, limits)) if errors else None
    # Each minimum may belong to a different joint solution. Keep a complete
    # witness as well; independent minima do not prove that a pose is feasible.
    minima = {}
    for name in limits.as_dict():
        values = [value[name] for value in errors if value[name] is not None]
        minima[name] = min(values) if values else None
    return {
        'evaluated_variants': len(items),
        'pose_feasible': sum(not value['violations'] for value in errors),
        'pose_rejection_counts': {
            name: sum(name in value['violations'] for value in errors)
            for name in limits.as_dict()
        },
        'minimum_errors': minima,
        'closest_alignment': closest,
    }
