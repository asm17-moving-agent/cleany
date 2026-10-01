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
    def __init__(self, positions: list[tuple[float, ...]]) -> None:
        q = np.asarray(positions, dtype=float)
        if q.ndim != 2 or len(q) < 2 or q.shape[1] < 1 or not np.isfinite(q).all():
            raise ValueError('joint path needs at least two finite equal-sized states')
        self._h = 1.0 / (len(q)-1)
        secants = np.diff(q, axis=0) / self._h
        slopes = np.zeros_like(q)  # rest at both endpoints
        for i in range(1, len(q)-1):
            previous, following = secants[i-1], secants[i]
            same_sign = previous * following > 0
            slopes[i, same_sign] = (2 * previous[same_sign] * following[same_sign]
                                    / (previous[same_sign]+following[same_sign]))
        delta = np.diff(q, axis=0)
        c = self._h * slopes[:-1]
        end_slope = self._h * slopes[1:]
        self._coefficients = np.stack((c+end_slope-2*delta,
                                       3*delta-2*c-end_slope, c, q[:-1]), axis=1)
        self.positions = q

    def sample(self, segment: int, fraction: float) -> JointPathSample:
        if not 0 <= segment < len(self._coefficients) or not 0 <= fraction <= 1:
            raise ValueError('invalid cubic path sample')
        a, b, c, d = self._coefficients[segment]
        u = fraction
        q = ((a*u+b)*u+c)*u+d
        derivative = (3*a*u*u+2*b*u+c)/self._h
        return JointPathSample((segment+u)*self._h, tuple(q), tuple(derivative))

    def derivative_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """Exact max |dq/ds|, |d²q/ds²| for normalized progress s."""
        velocity = np.zeros(self.positions.shape[1])
        acceleration = velocity.copy()
        for a, b, c, _ in self._coefficients:
            maximum = np.maximum(np.abs(c), np.abs(3*a+2*b+c))
            for j in range(len(a)):
                if abs(a[j]) > 1e-14:
                    u = -b[j]/(3*a[j])
                    if 0 < u < 1:
                        maximum[j] = max(maximum[j], abs(3*a[j]*u*u+2*b[j]*u+c[j]))
            velocity = np.maximum(velocity, maximum/self._h)
            acceleration = np.maximum(acceleration, np.maximum(np.abs(2*b), np.abs(6*a+2*b))/self._h**2)
        return velocity, acceleration

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
