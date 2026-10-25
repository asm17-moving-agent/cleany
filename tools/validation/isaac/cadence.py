"""Simulator publication cadence and pose-derived odometry, independent of ROS."""
from __future__ import annotations

import math
from collections import Counter


PUBLICATION_PROFILES = {
    'legacy': {'clock_hz': 500, 'odometry_hz': 500, 'scan_hz': 10},
    'split': {'clock_hz': 100, 'odometry_hz': 50, 'scan_hz': 10},
}


def publication_config(profile: str, physics_hz: int) -> dict:
    values = PUBLICATION_PROFILES[profile].copy()
    for name, hz in values.items():
        if hz <= 0 or physics_hz % hz:
            raise ValueError(f'{name} must divide physics_hz without rounding')
    if values['clock_hz'] % values['odometry_hz'] or values['odometry_hz'] % values['scan_hz']:
        raise ValueError('Clock, odometry and scan ticks must align')
    return {'publication_profile': profile, **values}


def yaw(pose) -> float:
    qx, qy, qz, qw = pose[3:7]
    return math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))


def measured_twist(before, after, seconds: float) -> tuple[float, float, float]:
    """Return body-frame displacement over the full odometry publication interval."""
    if seconds <= 0:
        raise ValueError('Odometry timestamps must increase')
    heading = yaw(before)
    dx, dy = (after[0]-before[0])/seconds, (after[1]-before[1])/seconds
    angle = yaw(after)-heading
    return (math.cos(heading)*dx+math.sin(heading)*dy,
            -math.sin(heading)*dx+math.cos(heading)*dy,
            math.atan2(math.sin(angle), math.cos(angle))/seconds)


def confirmation_window(samples: list[tuple], now: float) -> list[tuple]:
    """Select a fresh 0.8s simulation window spanning at least 0.5s.

    Samples contain (wall receipt time, simulation stamp, payload). Requiring
    recent wall receipt also prevents a paused or disconnected stream passing.
    """
    if not samples or now-samples[-1][0] > .8:
        return []
    last = samples[-1][1]
    window = [sample for sample in samples if last-sample[1] <= .8]
    if len(window) < 10 or window[-1][1]-window[0][1] < .5:
        return []
    return window


class StampStats:
    """Observe actual received message stamps without retaining message payloads."""

    def __init__(self) -> None:
        self.count = 0
        self.first = self.last = None
        self.intervals = Counter()

    def observe(self, seconds: int, nanoseconds: int) -> None:
        stamp = seconds*1_000_000_000+nanoseconds
        if self.first is None:
            self.first = stamp
        if self.last is not None:
            self.intervals[stamp-self.last] += 1
        self.last = stamp
        self.count += 1

    def result(self) -> dict:
        duration = (self.last-self.first)/1e9 if self.count else 0.
        return {'count': self.count, 'simulation_span_s': duration,
                'received_hz_sim': (self.count-1)/duration if duration > 0 else None,
                'nonincreasing_stamps': sum(count for dt, count in self.intervals.items() if dt <= 0),
                'interval_ns_histogram': dict(self.intervals)}
