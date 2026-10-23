"""Pure-Python detection of bounded repeating ICP poses."""

from __future__ import annotations

import math
from numbers import Real


class BoundedCycleDetector:
    """Detect stable pose cycles while bounding their total spatial excursion."""

    def __init__(
        self,
        *,
        translation_tolerance_m: float = 1e-5,
        rotation_tolerance_rad: float = 1e-5,
        max_translation_span_m: float = 1e-3,
        max_rotation_span_rad: float = math.radians(0.02),
        max_period: int = 6,
    ) -> None:
        values = (
            ("translation_tolerance_m", translation_tolerance_m),
            ("rotation_tolerance_rad", rotation_tolerance_rad),
            ("max_translation_span_m", max_translation_span_m),
            ("max_rotation_span_rad", max_rotation_span_rad),
        )
        for name, value in values:
            if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be a finite positive number")
        if isinstance(max_period, bool) or not isinstance(max_period, int) or max_period < 2:
            raise ValueError("max_period must be an integer >= 2")
        if translation_tolerance_m > max_translation_span_m:
            raise ValueError("translation tolerance must not exceed translation span")
        if rotation_tolerance_rad > max_rotation_span_rad:
            raise ValueError("rotation tolerance must not exceed rotation span")
        self.translation_tolerance_m = float(translation_tolerance_m)
        self.rotation_tolerance_rad = float(rotation_tolerance_rad)
        self.max_translation_span_m = float(max_translation_span_m)
        self.max_rotation_span_rad = float(max_rotation_span_rad)
        self.max_period = max_period
        self._history: list[tuple[float, float, float]] = []

    @staticmethod
    def _angle_distance(a: float, b: float) -> float:
        return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)

    def observe(self, pose: tuple[float, float, float], *, final_stage: bool) -> bool:
        """Record a pose and report whether three bounded cycles have repeated."""
        if not final_stage:
            self._history.clear()
            return False
        try:
            if len(pose) != 3:
                raise ValueError
            current = tuple(float(value) for value in pose)
        except (TypeError, ValueError, OverflowError):
            self._history.clear()
            return False
        if not all(math.isfinite(value) for value in current):
            self._history.clear()
            return False
        self._history.append(current)  # type: ignore[arg-type]
        self._history = self._history[-(3 * self.max_period):]

        for period in range(2, self.max_period + 1):
            if len(self._history) < 3 * period:
                continue
            samples = self._history[-(3 * period):]
            if any(
                math.hypot(samples[i][0] - samples[j][0], samples[i][1] - samples[j][1])
                >= self.translation_tolerance_m
                or self._angle_distance(samples[i][2], samples[j][2]) >= self.rotation_tolerance_rad
                for i in range(2 * period)
                for j in range(i + period, 3 * period, period)
            ):
                continue
            if any(
                math.hypot(a[0] - b[0], a[1] - b[1]) > self.max_translation_span_m
                or self._angle_distance(a[2], b[2]) > self.max_rotation_span_rad
                for i, a in enumerate(samples)
                for b in samples[i + 1:]
            ):
                continue
            return True
        return False
