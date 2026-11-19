"""Geometry association used only by the independent simulation verifier."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class BodyGeometry:
    body: str
    position: tuple[float, float, float]
    size: tuple[float, float, float]


def associate_body(position: tuple[float, float, float], size: tuple[float, float, float],
                   candidates: Iterable[BodyGeometry], *, maximum_distance_m: float,
                   ambiguity_margin_m: float, maximum_size_error_m: float) -> str:
    if (len(position) != 3 or len(size) != 3
            or not all(math.isfinite(v) for v in (*position, *size)) or min(size) <= 0):
        raise ValueError('Target geometry must be finite with positive size')
    if not all(math.isfinite(v) and v > 0 for v in (
            maximum_distance_m, ambiguity_margin_m, maximum_size_error_m)):
        raise ValueError('Invalid registration thresholds')
    matches = []
    for candidate in candidates:
        if (not all(math.isfinite(v) for v in (*candidate.position, *candidate.size))
                or min(candidate.size) <= 0):
            continue
        distance = math.dist(position, candidate.position)
        # Sorting extents tolerates OBB axis permutations and modest occlusion.
        size_error = max(abs(a-b) for a, b in zip(sorted(size), sorted(candidate.size)))
        if size_error <= maximum_size_error_m:
            matches.append((distance, candidate.body))
    matches.sort()
    if not matches or matches[0][0] > maximum_distance_m:
        raise ValueError('No simulator body matches the observed object')
    if len(matches) > 1 and matches[1][0] - matches[0][0] < ambiguity_margin_m:
        raise ValueError('Observed object has ambiguous simulator association')
    return matches[0][1]
