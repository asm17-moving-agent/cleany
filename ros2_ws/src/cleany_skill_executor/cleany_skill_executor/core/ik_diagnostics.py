"""Pose-error evidence without changing IK selection or acceptance."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Sequence

import numpy as np

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


@dataclass(frozen=True, slots=True)
class ScoredAlignments:
    """Selection witnesses and complete error statistics for one solver call.

    Candidates retain every feasible pose and the best rejected pose in source
    order. Statistics include discarded poses and use the recorded limits.
    """
    candidates: list[PoseAlignment]
    summary: dict
    limits: AlignmentLimits


def alignment_errors(item: PoseAlignment, limits: AlignmentLimits) -> dict:
    thresholds = limits.as_dict()
    values = (item.position_error_m * 1000., item.approach_error_degrees,
              item.closing_error_degrees)
    errors = dict(zip(thresholds,
                      (float(v) if math.isfinite(v) else None for v in values)))
    violations = [name for name, value in errors.items()
                  if value is None or value > thresholds[name]]
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


def summarize_error_rows(errors: Sequence[Sequence[float]] | np.ndarray, limits: AlignmentLimits,
                         witness: Callable[[int], PoseAlignment]) -> dict:
    """Summarize position/approach/closing errors in metres and degrees."""
    thresholds = limits.as_dict()
    values = np.asarray(errors, dtype=float).reshape(-1, 3) * (1000., 1., 1.)
    finite = np.isfinite(values)
    accepted = finite & (values <= tuple(thresholds.values()))
    closest = None
    if len(values):
        ratios = np.where(finite, values, math.inf) / tuple(thresholds.values())
        worst = ratios.max(axis=1)
        tied = np.flatnonzero(worst == worst.min())
        closest = int(tied[np.argmin(ratios[tied].sum(axis=1))])
    return {
        'evaluated_variants': len(values),
        'pose_feasible': int(accepted.all(axis=1).sum()),
        'pose_rejection_counts': {
            name: int((~accepted[:, index]).sum()) for index, name in enumerate(thresholds)
        },
        # Independent minima may belong to different poses; retain a witness.
        'minimum_errors': {
            name: float(values[finite[:, index], index].min()) if finite[:, index].any() else None
            for index, name in enumerate(thresholds)
        },
        'closest_alignment': alignment_errors(witness(closest), limits) if closest is not None else None,
    }


def summarize_alignments(items: Sequence[PoseAlignment] | ScoredAlignments,
                         limits: AlignmentLimits) -> dict:
    if isinstance(items, ScoredAlignments):
        if items.limits != limits:
            raise ValueError('Compacted alignment limits changed within a solver call')
        return items.summary
    return summarize_error_rows(
        [(item.position_error_m, item.approach_error_degrees, item.closing_error_degrees)
         for item in items], limits, lambda index: items[index])


def merge_alignments(first: Sequence[PoseAlignment] | ScoredAlignments,
                     second: Sequence[PoseAlignment] | ScoredAlignments
                     ) -> list[PoseAlignment] | ScoredAlignments:
    """Append refinement results while preserving full statistics and tie order."""
    if not isinstance(first, ScoredAlignments) and not isinstance(second, ScoredAlignments):
        return [*first, *second]
    limits = first.limits if isinstance(first, ScoredAlignments) else second.limits
    summaries = [summarize_alignments(items, limits) for items in (first, second)]
    closest = [value['closest_alignment'] for value in summaries
               if value['closest_alignment'] is not None]
    minima = {}
    for name in limits.as_dict():
        values = [value['minimum_errors'][name] for value in summaries
                  if value['minimum_errors'][name] is not None]
        minima[name] = min(values) if values else None
    summary = {
        'evaluated_variants': sum(value['evaluated_variants'] for value in summaries),
        'pose_feasible': sum(value['pose_feasible'] for value in summaries),
        'pose_rejection_counts': {
            name: sum(value['pose_rejection_counts'][name] for value in summaries)
            for name in limits.as_dict()},
        'minimum_errors': minima,
        'closest_alignment': min(closest, key=lambda value: alignment_rank(value, limits))
                             if closest else None,
    }
    candidates = [item for group in (first, second)
                  for item in (group.candidates if isinstance(group, ScoredAlignments) else group)]
    return ScoredAlignments(candidates, summary, limits)
