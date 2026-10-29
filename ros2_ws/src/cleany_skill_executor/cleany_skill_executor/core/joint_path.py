"""Monotone cubic joint interpolation with analytical derivative bounds.

Positions + velocities reconstruct these same cubics in ROS 2 JTC's spline
interpolator. Acceleration fields must remain empty (not quintic interpolation).
This provides joint-rate bounds, not continuous Cartesian collision guarantees.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class JointPathSample:
    progress: float
    positions: tuple[float, ...]
    derivatives: tuple[float, ...]


class CubicJointPath:
    def __init__(self, positions: list[tuple[float, ...]],
                 progressions: tuple[float, ...] | None = None) -> None:
        try:
            q = np.asarray(positions, dtype=float)
        except (TypeError, ValueError) as error:
            raise ValueError('joint path needs at least two finite equal-sized states') from error
        if q.ndim != 2 or len(q) < 2 or q.shape[1] < 1 or not np.isfinite(q).all():
            raise ValueError('joint path needs at least two finite equal-sized states')
        if progressions is None:
            knots = np.linspace(0.0, 1.0, len(q))
        else:
            knots = np.asarray(progressions, dtype=float)
            if (knots.shape != (len(q),) or not np.isfinite(knots).all()
                    or abs(knots[0]) > 1e-12 or abs(knots[-1]-1.0) > 1e-12
                    or np.any(np.diff(knots) <= 0)):
                raise ValueError('joint path progressions must increase from zero to one')
        steps = np.diff(knots)
        secants = np.diff(q, axis=0) / steps[:, None]
        slopes = np.zeros_like(q)  # rest at both endpoints
        for i in range(1, len(q)-1):
            previous, following = secants[i-1], secants[i]
            same_sign = previous * following > 0
            previous_step, following_step = steps[i-1], steps[i]
            previous_weight = 2 * following_step + previous_step
            following_weight = following_step + 2 * previous_step
            slopes[i, same_sign] = (
                (previous_weight + following_weight)
                / (previous_weight / previous[same_sign]
                   + following_weight / following[same_sign])
            )
        delta = np.diff(q, axis=0)
        c = steps[:, None] * slopes[:-1]
        end_slope = steps[:, None] * slopes[1:]
        self._coefficients = np.stack((c+end_slope-2*delta,
                                       3*delta-2*c-end_slope, c, q[:-1]), axis=1)
        self._knots = knots
        self._steps = steps
        self.positions = q
        self.progressions = tuple(float(value) for value in knots)

    def sample(self, segment: int, fraction: float) -> JointPathSample:
        if not 0 <= segment < len(self._coefficients) or not 0 <= fraction <= 1:
            raise ValueError('invalid cubic path sample')
        a, b, c, d = self._coefficients[segment]
        u = fraction
        q = ((a*u+b)*u+c)*u+d
        # Polynomial cancellation can move a boundary goal a few ULPs outside
        # its strict physical limit. Preserve the supplied knots exactly.
        if u == 0.0 or u == 1.0:
            q = self.positions[segment + int(u)]
        derivative = (3*a*u*u+2*b*u+c)/self._steps[segment]
        progress = self._knots[segment] + u*self._steps[segment]
        return JointPathSample(float(progress), tuple(q), tuple(derivative))

    def _segment_derivative_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """Exact per-segment bounds with respect to local fraction u."""
        a, b, c = (self._coefficients[:, index] for index in range(3))
        maximum = np.maximum(np.abs(c), np.abs(3*a+2*b+c))
        u = np.divide(-b, 3*a, out=np.zeros_like(a), where=np.abs(a) > 1e-14)
        maximum = np.maximum(maximum,
            np.where((u > 0) & (u < 1), np.abs(3*a*u*u+2*b*u+c), 0.))
        return maximum, np.maximum(np.abs(2*b), np.abs(6*a+2*b))

    def derivative_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """Exact max |dq/ds|, |d²q/ds²| for normalized progress s."""
        velocity, acceleration = self._segment_derivative_bounds()
        return (np.max(velocity/self._steps[:, None], axis=0),
                np.max(acceleration/self._steps[:, None]**2, axis=0))

    def retimed(self, velocity_limits, acceleration_limits) -> CubicJointPath:
        """Allocate time locally, retaining knots and continuous velocities.

        A tiny entry/turn segment must not slow every other segment to satisfy
        its acceleration bound. Rebuild the monotone spline after local time
        changes; callers must validate this resulting geometry before dispatch.
        The final minimum_duration still analytically enforces every limit,
        even if the bounded local iterations have not fully converged.
        """
        initial_duration = self.minimum_duration(velocity_limits, acceleration_limits)
        if len(self.positions) == 2:
            # One segment already spans [0, 1]; no time allocation can change it.
            return self
        v = np.asarray(velocity_limits, dtype=float)
        a = np.asarray(acceleration_limits, dtype=float)

        def required_times(path: CubicJointPath) -> np.ndarray:
            velocity, acceleration = path._segment_derivative_bounds()
            return np.maximum(np.max(velocity/v, axis=1),
                              np.sqrt(np.max(acceleration/a, axis=1)))

        points = [tuple(row) for row in self.positions]
        # Invert rest-to-rest scalar smoothstep for a second initial schedule.
        # Dense straight paths then accelerate over the path rather than only
        # over their tiny first/last segment. No knot is removed.
        eased = .5-np.sin(np.arcsin(np.clip(1.-2*self._knots, -1., 1.))/3.)
        eased[0], eased[-1] = 0., 1.
        best, best_duration = self, initial_duration
        if np.all(np.diff(eased) > 0):
            candidate = CubicJointPath(points, tuple(eased))
            duration = candidate.minimum_duration(v, a)
            if duration < best_duration:
                best, best_duration = candidate, duration
        seed = best
        # Seed each segment with the time its own cubic needs, rather than
        # multiplying every segment by the worst segment's global duration.
        times = np.maximum(required_times(seed), 1e-6)
        for _ in range(32):
            progress = np.concatenate(([0.0], np.cumsum(times)/np.sum(times)))
            progress[-1] = 1.0
            candidate = CubicJointPath(points, tuple(progress))
            duration = candidate.minimum_duration(v, a)
            if duration < best_duration:
                best, best_duration = candidate, duration
            required = required_times(candidate)
            if np.all(required <= times*(1.0+1e-6)):
                break
            # Adjacent harmonic slopes change with time allocation. Stretch
            # only the affected segments and recompute their shared slopes.
            times = np.maximum(times, required)
        return best

    def minimum_duration(self, velocity_limits, acceleration_limits) -> float:
        velocity, acceleration = self.derivative_bounds()
        v = np.asarray(velocity_limits, dtype=float)
        a = np.asarray(acceleration_limits, dtype=float)
        if (v.shape != velocity.shape or a.shape != acceleration.shape or
                not np.isfinite(v).all() or not np.isfinite(a).all() or
                np.any(v <= 0) or np.any(a <= 0)):
            raise ValueError('joint derivative limits must be finite positive per-joint values')
        return max(float(np.max(velocity/v)), float(np.sqrt(np.max(acceleration/a))), 1e-3)

    def samples(self, maximum_joint_step: float, minimum_subdivisions: int,
                maximum_points: int = 2000) -> list[JointPathSample]:
        if (not math.isfinite(maximum_joint_step) or maximum_joint_step <= 0 or
                minimum_subdivisions < 1 or maximum_points < 2):
            raise ValueError('invalid joint path sampling limits')
        # Bound each segment separately: one difficult segment must not force
        # every near-stationary segment to use the same dense sampling rate.
        counts = []
        for a, b, c, _ in self._coefficients:
            maximum = np.maximum(np.abs(c), np.abs(3*a+2*b+c))
            for j in range(len(a)):
                if abs(a[j]) > 1e-14:
                    u = -b[j]/(3*a[j])
                    if 0 < u < 1:
                        maximum[j] = max(maximum[j], abs(3*a[j]*u*u+2*b[j]*u+c[j]))
            counts.append(max(minimum_subdivisions,
                              math.ceil(float(np.max(maximum))/maximum_joint_step)))
        if sum(counts)+1 > maximum_points:
            raise ValueError('joint path exceeds sample budget')
        result = [self.sample(0, 0.)]
        for segment, subdivisions in enumerate(counts):
            result.extend(self.sample(segment, step/subdivisions)
                          for step in range(1, subdivisions+1))
        return result


def executable_joint_path(
        positions: list[tuple[float, ...]], *,
        simplification_tolerance: float = .002,
        duplicate_tolerance: float = 1e-7) -> CubicJointPath:
    """Build the collision-checkable cubic geometry sent to the controller.

    MoveIt commonly emits many nearly collinear states with nonuniform joint
    distances. Removing only sub-tolerance bends and parameterizing the
    retained states by joint-space distance prevents every planner waypoint
    from becoming an artificial speed-change point.
    """
    if (not math.isfinite(simplification_tolerance)
            or not math.isfinite(duplicate_tolerance)
            or simplification_tolerance < 0 or duplicate_tolerance < 0):
        raise ValueError('invalid joint path simplification tolerance')
    try:
        q = np.asarray(positions, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError('joint path needs at least two finite equal-sized states') from error
    if q.ndim != 2 or len(q) < 2 or q.shape[1] < 1 or not np.isfinite(q).all():
        raise ValueError('joint path needs at least two finite equal-sized states')

    # Preserve both command endpoints while removing duplicate interior states.
    rows = [q[0]]
    for row in q[1:-1]:
        if np.max(np.abs(row-rows[-1])) > duplicate_tolerance:
            rows.append(row)
    if len(rows) == 1 or np.max(np.abs(q[-1]-rows[-1])) > duplicate_tolerance:
        rows.append(q[-1])
    else:
        rows[-1] = q[-1]
    reduced = np.asarray(rows)

    # Ramer-Douglas-Peucker in joint space. The infinity norm makes the
    # configured tolerance an explicit per-joint maximum geometric change.
    if len(reduced) > 2 and simplification_tolerance > 0:
        keep = {0, len(reduced)-1}
        pending = [(0, len(reduced)-1)]
        while pending:
            first, last = pending.pop()
            if last <= first+1:
                continue
            start, finish = reduced[first], reduced[last]
            chord = finish-start
            chord_squared = float(np.dot(chord, chord))
            maximum_error, maximum_index = -1.0, -1
            for index in range(first+1, last):
                if chord_squared <= duplicate_tolerance**2:
                    projection = start
                else:
                    fraction = float(np.dot(reduced[index]-start, chord)/chord_squared)
                    projection = start + min(1.0, max(0.0, fraction))*chord
                error = float(np.max(np.abs(reduced[index]-projection)))
                if error > maximum_error:
                    maximum_error, maximum_index = error, index
            if maximum_error > simplification_tolerance:
                keep.add(maximum_index)
                pending.extend(((first, maximum_index), (maximum_index, last)))
        reduced = reduced[sorted(keep)]

    lengths = np.linalg.norm(np.diff(reduced, axis=0), axis=1)
    total = float(np.sum(lengths))
    if total <= duplicate_tolerance:
        progressions = tuple(np.linspace(0.0, 1.0, len(reduced)))
    else:
        progressions = tuple(np.concatenate(([0.0], np.cumsum(lengths)/total)))
        progressions = (*progressions[:-1], 1.0)
    return CubicJointPath([tuple(row) for row in reduced], progressions)
