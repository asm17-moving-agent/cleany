"""Measured trajectory response checks, independent of ROS and collision maps."""
from __future__ import annotations

from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass
import math
from threading import Lock


@dataclass(frozen=True)
class MotionMonitorConfig:
    maximum_error_rad: float
    error_duration_sec: float
    response_window_sec: float
    response_error_rad: float
    minimum_motion_rad: float
    feedback_timeout_sec: float
    abort_on_deviation: bool = True

    def __post_init__(self) -> None:
        if type(self.abort_on_deviation) is not bool:
            raise ValueError('abort_on_deviation must be boolean')
        if any(not math.isfinite(value) or value <= 0 for value in (
                self.maximum_error_rad, self.error_duration_sec,
                self.response_window_sec, self.response_error_rad,
                self.minimum_motion_rad, self.feedback_timeout_sec)):
            raise ValueError('Invalid motion-monitor limits')
        if self.minimum_motion_rad >= self.response_error_rad:
            raise ValueError('Response error must exceed the encoder-motion threshold')


class MotionMonitor:
    """Latch a fault on sustained deviation, stalled joints, or lost feedback."""

    def __init__(self, names: tuple[str, ...], config: MotionMonitorConfig,
                 started_sec: float):
        if not names or len(set(names)) != len(names) or not math.isfinite(started_sec):
            raise ValueError('Invalid monitored trajectory')
        self.names, self.config, self.started_sec = names, config, started_sec
        self.last_feedback_sec: float | None = None
        self.fault = ''
        self.samples = 0
        self.maximum_error_rad = dict.fromkeys(names, 0.)
        self._deviation_since: dict[str, float] = {}
        self._response: dict[str, deque[tuple[float, float]]] = {
            name: deque() for name in names}
        self._lock = Lock()
        self._warned_deviations: set[str] = set()
        self._warning_counts: dict[str, int] = {}
        self._pending_warnings: deque[str] = deque()

    def observe(self, names: Sequence[str], desired: Sequence[float],
                actual: Sequence[float], now_sec: float) -> None:
        with self._lock:
            if self.fault:
                return
            if (tuple(names) != self.names or len(desired) != len(self.names)
                    or len(actual) != len(self.names)
                    or not all(math.isfinite(value) for value in (*desired, *actual, now_sec))
                    or now_sec < self.started_sec):
                self.fault = 'Invalid physical trajectory feedback'
                return
            if self.last_feedback_sec is not None and now_sec <= self.last_feedback_sec:
                return
            self.last_feedback_sec = now_sec
            self.samples += 1
            config = self.config
            for name, target, measured in zip(self.names, desired, actual, strict=True):
                error = abs(target-measured)
                self.maximum_error_rad[name] = max(self.maximum_error_rad[name], error)
                if error > config.maximum_error_rad:
                    since = self._deviation_since.setdefault(name, now_sec)
                    if now_sec-since >= config.error_duration_sec:
                        detail = f'Sustained trajectory deviation: {name} error={error:.6f}rad'
                        if config.abort_on_deviation:
                            self.fault = detail
                            return
                        if name not in self._warned_deviations:
                            self._warned_deviations.add(name)
                            self._warning_counts[name] = self._warning_counts.get(name, 0)+1
                            self._pending_warnings.append(detail)
                else:
                    self._deviation_since.pop(name, None)
                    self._warned_deviations.discard(name)
                history = self._response[name]
                # Only accumulate a response window while movement is demanded.
                # A completed/near-target joint must not inherit a stale stall window.
                if error <= config.response_error_rad:
                    history.clear()
                    continue
                history.append((now_sec, float(measured)))
                boundary = now_sec-config.response_window_sec
                while len(history) > 1 and history[1][0] <= boundary:
                    history.popleft()
                if now_sec-history[0][0] >= config.response_window_sec:
                    positions = [sample[1] for sample in history]
                    if max(positions)-min(positions) < config.minimum_motion_rad:
                        self.fault = (
                            f'Joint did not respond: {name} error={error:.6f}rad '
                            f'window={config.response_window_sec:.3f}s')
                        return

    def check(self, now_sec: float) -> None:
        with self._lock:
            last = self.started_sec if self.last_feedback_sec is None else self.last_feedback_sec
            if not self.fault and now_sec-last > self.config.feedback_timeout_sec:
                self.fault = 'Physical trajectory feedback missing or stale'
            if self.fault:
                raise RuntimeError(self.fault)

    def report(self) -> dict[str, object]:
        with self._lock:
            return {'feedback_samples': self.samples,
                    'maximum_error_rad_by_joint': dict(self.maximum_error_rad),
                    'deviation_warning_counts_by_joint': dict(self._warning_counts),
                    'abort_on_deviation': self.config.abort_on_deviation,
                    'fault': self.fault}

    def take_warnings(self) -> tuple[str, ...]:
        with self._lock:
            warnings = tuple(self._pending_warnings)
            self._pending_warnings.clear()
            return warnings
