"""Retiming for a straight joint segment with continuous, bounded acceleration."""

from bisect import bisect_right
from dataclasses import replace
from math import sqrt
from typing import TYPE_CHECKING
from collections.abc import Sequence

import numpy as np


if TYPE_CHECKING:
    from .hardware_execution import BoundedJointPath
from .joint_path import JointPathSample


def retime_straight(plan: "BoundedJointPath", velocities: Sequence[float],
                    accelerations: Sequence[float], floor: float, maximum_points: int) -> "BoundedJointPath":
    """Keep every checked position and synchronize all axes on one scalar path."""
    if len(plan.command_knots) != 2:
        return plan
    start, end = (np.array(k.positions) for k in plan.command_knots)
    delta = end - start
    moving = np.abs(delta) > 1e-12
    if not moving.any():
        return plan
    v_cap = np.min(np.array(velocities)[moving] / np.abs(delta[moving]))
    a_cap = np.min(np.array(accelerations)[moving] / np.abs(delta[moving]))
    # One fifth of the acceleration interval at each end rounds the ramp.
    blend = 0.2
    speed = min(v_cap, sqrt(a_cap * (1 - blend)))
    ramp = speed / (a_cap * (1 - blend))
    cruise = max(0.0, 1.0 / speed - ramp)
    total = 2 * ramp + cruise
    duration = max(floor, 1.05 * total)
    if duration >= plan.duration:
        return plan
    shoulder = blend * ramp
    jerk = a_cap / shoulder
    middle = ramp - 2 * shoulder
    segments = [
        (shoulder, jerk),
        (middle, 0.0),
        (shoulder, -jerk),
        (cruise, 0.0),
        (shoulder, -jerk),
        (middle, 0.0),
        (shoulder, jerk),
    ]
    times, positions, speeds, accels, jerks = [0.0], [0.0], [0.0], [0.0], []
    for length, j in segments:
        if length < 1e-12:
            continue
        s, v, a = positions[-1], speeds[-1], accels[-1]
        positions.append(s + length * (v + length * (a / 2 + length * j / 6)))
        speeds.append(v + length * (a + length * j / 2))
        accels.append(a + length * j)
        times.append(times[-1] + length)
        jerks.append(j)

    def sample(t):
        if t <= 0.0:
            return 0.0, 0.0
        if t >= total:
            return 1.0, 0.0
        i = min(bisect_right(times, t) - 1, len(jerks) - 1)
        h = t - times[i]
        return (
            positions[i] + h * (speeds[i] + h * (accels[i] / 2 + h * jerks[i] / 6)),
            speeds[i] + h * (accels[i] + h * jerks[i] / 2),
        )

    def knot(t):
        if t <= 0 or t >= total:
            endpoint = plan.command_knots[0 if t <= 0 else -1]
            return JointPathSample(0.0 if t <= 0 else 1.0, endpoint.positions, (0.0,) * len(delta))
        fraction, velocity = sample(t)
        return JointPathSample(
            t / total, tuple(start + fraction * delta), tuple(delta * velocity * total)
        )

    knots = tuple(knot(t) for t in [*times[:-1], total])
    axis = int(np.argmax(abs(delta)))
    interior = plan.samples[1:-1]
    fractions = (np.array([p.positions[axis] for p in interior]) - start[axis]) / delta[axis]
    segment = np.clip(np.searchsorted(positions, fractions, side="right") - 1, 0, len(jerks) - 1)
    t0, p0, v0, a0, j = (np.asarray(x)[segment] for x in (times, positions, speeds, accels, jerks))
    lo, hi = np.zeros_like(fractions), np.diff(times)[segment]
    # Invert each monotone cubic on its own interval, for all collision samples together.
    for _ in range(44):
        h = (lo + hi) / 2
        below = p0 + h * (v0 + h * (a0 / 2 + h * j / 6)) < fractions
        lo, hi = np.where(below, h, lo), np.where(below, hi, h)
    h = (lo + hi) / 2
    progress = (t0 + h) / total
    rates = (v0 + h * (a0 + h * j / 2)) * total
    samples = [JointPathSample(t, p.positions, tuple(delta * rate))
               for t, p, rate in zip(progress, interior, rates, strict=True)]
    # Existing spatial samples plus all velocity/acceleration transitions go to FCL.
    # ROS controller timestamps have nanosecond precision. Prefer a transition
    # knot if its checked sample would serialize at the same timestamp.
    by_time = {round(p.progress * duration * 1e9): p for p in (*samples, *knots)}
    if len(by_time) > maximum_points:
        return plan
    return replace(plan, samples=[by_time[t] for t in sorted(by_time)],
                   duration=duration, command_knots=knots)
