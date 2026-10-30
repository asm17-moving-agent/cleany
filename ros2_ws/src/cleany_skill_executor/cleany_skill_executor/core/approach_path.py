"""Bounded approach detours with an unchanged goal and a straight final entry."""
from __future__ import annotations

from dataclasses import dataclass
import math
from collections.abc import Sequence

from .alignment import quaternion_axis
from .cartesian import CartesianPose, validate_position_endpoint


class CartesianPathUnavailable(RuntimeError):
    """A Cartesian planning request failed without submitting a motion goal."""

    def __init__(self, message: str, *, error_code: int, fraction: float):
        super().__init__(message)
        self.error_code = error_code
        self.fraction = fraction

    @property
    def entry_blocked(self) -> bool:
        return self.error_code == -10  # MoveIt START_STATE_IN_COLLISION


@dataclass(frozen=True)
class ApproachRoute:
    name: str
    waypoints: tuple[CartesianPose, ...]


def approach_detours(start: CartesianPose, target: CartesianPose, *, offset_m: float,
                     final_straight_m: float, maximum_extra_length_m: float,
                     attempts: int) -> tuple[ApproachRoute, ...]:
    """Search +/- closing and perpendicular tool axes before the final entry.

    These are planning candidates, never unchecked physical probe movements.
    """
    if (type(attempts) is not int or not 1 <= attempts <= 4
            or any(not math.isfinite(value) or value <= 0 for value in
                   (offset_m, final_straight_m, maximum_extra_length_m))):
        raise ValueError('Invalid approach detour limits')
    approach = quaternion_axis(target.orientation, (0., -1., 0.))
    remaining = sum((t-s)*a for s, t, a in zip(
        start.position, target.position, approach, strict=True))
    if remaining <= final_straight_m:
        return ()  # Never divert the TCP sideways inside the final grasp entry.
    entry = CartesianPose(tuple(t-a*final_straight_m for t, a in zip(
        target.position, approach, strict=True)), target.orientation)
    pivot = tuple((s+e)/2 for s, e in zip(start.position, entry.position, strict=True))
    direct_length = math.dist(start.position, target.position)
    routes = []
    axes = (('closing', (1., 0., 0.)), ('tool_z', (0., 0., 1.)))
    for name, axis in axes:
        direction = quaternion_axis(target.orientation, axis)
        for sign, suffix in ((1., 'plus'), (-1., 'minus')):
            via = CartesianPose(tuple(p+sign*offset_m*d for p, d in zip(
                pivot, direction, strict=True)), target.orientation)
            points = (start, via, entry, target)
            length = sum(math.dist(a.position, b.position) for a, b in zip(points, points[1:]))
            if length <= direct_length + maximum_extra_length_m + 1e-9:
                routes.append(ApproachRoute(name+'_'+suffix, (via, entry, target)))
    return tuple(routes[:attempts])


def validate_detour_samples(samples: Sequence[CartesianPose], start: CartesianPose,
                            route: ApproachRoute, *, radius_m: float,
                            maximum_extra_length_m: float) -> None:
    """Check sampled FK against the ordered, bounded waypoint corridor."""
    if (len(samples) < 2 or len(route.waypoints) != 3
            or any(not math.isfinite(value) or value <= 0
                   for value in (radius_m, maximum_extra_length_m))):
        raise ValueError('Invalid detour corridor inputs')
    target = route.waypoints[-1]
    validate_position_endpoint(samples[0], start, radius_m)
    validate_position_endpoint(samples[-1], target, radius_m)
    points = (start, *route.waypoints)
    segments = []
    cumulative = 0.
    for left, right in zip(points, points[1:]):
        length = math.dist(left.position, right.position)
        if length <= 1e-9:
            raise ValueError('Detour has a zero-length corridor segment')
        direction = tuple((r-l)/length for l, r in zip(left.position, right.position, strict=True))
        segments.append((left.position, direction, length, cumulative))
        cumulative += length
    maximum_length = math.dist(start.position, target.position) + maximum_extra_length_m
    if cumulative > maximum_length + 1e-9:
        raise ValueError('Detour exceeds its path length budget')
    previous = 0.
    for sample in samples:
        projections = []
        for origin, direction, length, progress in segments:
            offset = tuple(p-o for p, o in zip(sample.position, origin, strict=True))
            along = max(0., min(length, sum(o*d for o, d in zip(offset, direction, strict=True))))
            distance = math.sqrt(sum((o-along*d)**2 for o, d in zip(offset, direction, strict=True)))
            projections.append((distance, progress+along))
        distance, progress = min(projections)
        if distance > radius_m or progress < previous-radius_m:
            raise ValueError('FK samples leave or reverse the detour corridor')
        previous = max(previous, progress)
    actual_length = sum(math.dist(a.position, b.position) for a, b in zip(samples, samples[1:]))
    if actual_length > maximum_length + 2*radius_m:
        raise ValueError('FK samples exceed the detour path length budget')
