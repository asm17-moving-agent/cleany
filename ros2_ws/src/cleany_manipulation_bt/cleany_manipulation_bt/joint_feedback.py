"""Per-joint freshness and continuous stationary evidence, independent of ROS."""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class JointSample:
    stamp_ns: int
    received_at: float
    velocity: float

    def fresh(self, now: float, clock_ns: int, max_age_sec: float,
              *, after_ns: int = 0, after_wall: float = -math.inf) -> bool:
        return (math.isfinite(self.velocity) and self.stamp_ns > after_ns
                and self.received_at > after_wall
                and 0.0 <= now - self.received_at < max_age_sec
                and abs(clock_ns - self.stamp_ns) / 1e9 < max_age_sec)


class JointFeedback:
    def __init__(self, required_names: Sequence[str]) -> None:
        self.required_names = tuple(required_names)
        self.samples: dict[str, JointSample] = {}

    def update(self, names: Sequence[str], velocities: Sequence[float],
               stamp_ns: int, received_at: float) -> None:
        counts = Counter(names)
        complete_velocities = len(names) == len(velocities)
        for index, name in enumerate(names):
            if name not in self.required_names:
                continue
            velocity = velocities[index] if complete_velocities and counts[name] == 1 else math.nan
            previous = self.samples.get(name)
            if previous is not None and stamp_ns <= previous.stamp_ns:
                # Replayed or out-of-order feedback cannot refresh its receipt
                # time, overwrite motion with an old zero, or count as a sample.
                self.samples[name] = JointSample(previous.stamp_ns, previous.received_at, math.nan)
            else:
                self.samples[name] = JointSample(stamp_ns, received_at, velocity)

    def invalid_joints(self, now: float, clock_ns: int, max_age_sec: float,
                       *, after_ns: int = 0, after_wall: float = -math.inf) -> tuple[str, ...]:
        return tuple(name for name in self.required_names
                     if name not in self.samples or not self.samples[name].fresh(
                         now, clock_ns, max_age_sec, after_ns=after_ns, after_wall=after_wall))


class StationaryWindow:
    """Count rounds in which every required joint has supplied a new sample."""
    def __init__(self, *, barrier_ns: int, started_at: float, max_age_sec: float,
                 maximum_velocity: float, required_samples: int, minimum_duration_sec: float) -> None:
        for name, value in (('max_age_sec', max_age_sec), ('minimum_duration_sec', minimum_duration_sec)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if not math.isfinite(maximum_velocity) or maximum_velocity < 0.0:
            raise ValueError('maximum_velocity must be finite and nonnegative')
        if required_samples < 1:
            raise ValueError('required_samples must be positive')
        self.barrier_ns, self.started_at = barrier_ns, started_at
        self.max_age_sec, self.maximum_velocity = max_age_sec, maximum_velocity
        self.required_samples, self.minimum_duration_sec = required_samples, minimum_duration_sec
        self.samples = 0
        self.stationary_since: float | None = None
        self._consumed: dict[str, int] = {}
        self._previous_receipts: dict[str, float] = {}

    def observe(self, feedback: JointFeedback, *, now: float, clock_ns: int, terminal: bool) -> bool:
        gap = self.stationary_since is not None and any(
            sample.received_at - self._previous_receipts.get(name, sample.received_at) >= self.max_age_sec
            for name, sample in feedback.samples.items())
        self._previous_receipts = {name: sample.received_at for name, sample in feedback.samples.items()}
        if (not terminal or gap or feedback.invalid_joints(
                now, clock_ns, self.max_age_sec, after_ns=self.barrier_ns, after_wall=self.started_at)
                or any(abs(feedback.samples[name].velocity) > self.maximum_velocity
                       for name in feedback.required_names)):
            self.samples, self.stationary_since = 0, None
            self._consumed = {name: sample.stamp_ns for name, sample in feedback.samples.items()}
            return False
        if any(feedback.samples[name].stamp_ns <= self._consumed.get(name, self.barrier_ns)
               for name in feedback.required_names):
            return False
        self._consumed = {name: feedback.samples[name].stamp_ns for name in feedback.required_names}
        self.samples += 1
        if self.stationary_since is None:
            self.stationary_since = now
        return (self.samples >= self.required_samples
                and now - self.stationary_since >= self.minimum_duration_sec)
