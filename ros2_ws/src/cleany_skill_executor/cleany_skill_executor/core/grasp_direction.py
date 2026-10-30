"""Cheap ordering of equivalent grasp directions, before IK or planning."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TYPE_CHECKING, Sequence

from .alignment import quaternion_axis

if TYPE_CHECKING:
    from .physical_grasp_selection import Candidate


@dataclass(frozen=True, slots=True)
class GraspDirectionConfig:
    enabled: bool = False
    movement_tie_degrees: float = 5.0
    pair_position_tolerance_m: float = 1e-5
    pair_axis_tolerance_degrees: float = 1.0
    pair_score_tolerance: float = 1e-6

    def __post_init__(self) -> None:
        if type(self.enabled) is not bool:
            raise ValueError('Grasp direction preference must be boolean')
        if not math.isfinite(self.movement_tie_degrees) or not 0 <= self.movement_tie_degrees < 90:
            raise ValueError('Wrist direction tie tolerance must be in [0, 90) degrees')
        if not math.isfinite(self.pair_position_tolerance_m) or self.pair_position_tolerance_m <= 0:
            raise ValueError('Grasp direction pair position tolerance must be positive')
        if not math.isfinite(self.pair_axis_tolerance_degrees) or not 0 < self.pair_axis_tolerance_degrees < 90:
            raise ValueError('Grasp direction pair axis tolerance must be in (0, 90) degrees')
        if not math.isfinite(self.pair_score_tolerance) or self.pair_score_tolerance < 0:
            raise ValueError('Grasp direction pair score tolerance must be nonnegative')


def _dot(left: Sequence[float], right: Sequence[float]) -> float:
    return sum(a * b for a, b in zip(left, right, strict=True))


def grasp_direction_pairs(
    ranked: Sequence[Candidate], config: GraspDirectionConfig,
) -> list[tuple[Candidate, ...]]:
    """Keep grasp-quality order; group only equivalent 180-degree variants."""
    axes = [(
        quaternion_axis(item.orientation_xyzw, (0., -1., 0.)),
        quaternion_axis(item.orientation_xyzw, (1., 0., 0.)),
    ) for item in ranked]
    cosine = math.cos(math.radians(config.pair_axis_tolerance_degrees))
    used: set[int] = set()
    groups: list[tuple[Candidate, ...]] = []
    for index, item in enumerate(ranked):
        if index in used:
            continue
        used.add(index)
        pair = (item,)
        for other_index in range(index + 1, len(ranked)):
            if other_index in used:
                continue
            other = ranked[other_index]
            if abs(item.score - other.score) > config.pair_score_tolerance:
                # Scores are descending, so no later candidate can match.
                break
            if math.dist(item.position, other.position) > config.pair_position_tolerance_m:
                continue
            if any(abs(getattr(item, name) - getattr(other, name)) > config.pair_position_tolerance_m
                   for name in ('required_opening_m', 'depth_m', 'approach_offset_m')):
                continue
            if item.lateral_offset_m is None or other.lateral_offset_m is None:
                if item.lateral_offset_m != other.lateral_offset_m:
                    continue
            elif abs(item.lateral_offset_m - other.lateral_offset_m) > config.pair_position_tolerance_m:
                continue
            approach, closing = axes[index]
            other_approach, other_closing = axes[other_index]
            if _dot(approach, other_approach) < cosine or _dot(closing, other_closing) > -cosine:
                continue
            used.add(other_index)
            pair = (item, other)
            break
        groups.append(pair)
    return groups


def order_grasp_directions(
    pair: tuple[Candidate, ...], current_orientation_xyzw: tuple[float, float, float, float],
    config: GraspDirectionConfig, fixed_jaw_sign: float,
) -> tuple[Candidate, ...]:
    """Estimate roll by projecting current closing onto the requested approach plane.

    This is an ordering heuristic, not an IK solution or a reachability check.
    If the projection is degenerate, retain the incoming order.
    """
    if len(pair) != 2:
        return pair
    current_closing = quaternion_axis(current_orientation_xyzw, (1., 0., 0.))
    movement = []
    leftness = []
    for item in pair:
        approach = quaternion_axis(item.orientation_xyzw, (0., -1., 0.))
        closing = quaternion_axis(item.orientation_xyzw, (1., 0., 0.))
        along_approach = _dot(current_closing, approach)
        projection = tuple(a - along_approach * b
                           for a, b in zip(current_closing, approach, strict=True))
        norm = math.sqrt(_dot(projection, projection))
        if norm <= 1e-9:
            return pair
        cosine = max(-1., min(1., _dot(projection, closing) / norm))
        movement.append(math.degrees(math.acos(cosine)))
        # Candidate and current pose are expressed in base_link: +Y is left.
        leftness.append(fixed_jaw_sign * closing[1])
    if abs(movement[0] - movement[1]) <= config.movement_tie_degrees:
        first = 1 if leftness[1] - leftness[0] > 1e-9 else 0
    else:
        first = 1 if movement[1] < movement[0] else 0
    return pair if first == 0 else (pair[1], pair[0])
