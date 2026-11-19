from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, pi
from random import Random


WHEEL_COUNT = 4


@dataclass(frozen=True)
class EncoderReading:
    """One quantized four-wheel encoder sample."""

    counts: tuple[int, int, int, int]
    positions_rad: tuple[float, float, float, float]
    velocities_rad_s: tuple[float, float, float, float]


class SimulatedQuadratureEncoder:
    """Convert ideal cumulative wheel angles into synthetic encoder readings."""

    def __init__(
        self,
        *,
        ticks_per_revolution: int,
        wheel_scales: tuple[float, float, float, float],
        tick_noise_stddev: float,
        random_seed: int,
    ) -> None:
        if ticks_per_revolution <= 0:
            raise ValueError('ticks_per_revolution must be positive')
        if len(wheel_scales) != WHEEL_COUNT or not all(
            isfinite(scale) and scale > 0.0 for scale in wheel_scales
        ):
            raise ValueError('Four positive finite wheel scales are required')
        if not isfinite(tick_noise_stddev) or tick_noise_stddev < 0.0:
            raise ValueError('tick_noise_stddev must be finite and non-negative')

        self._ticks_per_revolution = ticks_per_revolution
        self._wheel_scales = wheel_scales
        self._tick_noise_stddev = tick_noise_stddev
        self._random = Random(random_seed)
        self._previous_true_positions: tuple[
            float, float, float, float
        ] | None = None
        self._continuous_counts: list[float] = []
        self._previous_counts: tuple[int, int, int, int] | None = None
        self._previous_stamp_s: float | None = None

    def update(
        self,
        positions_rad: tuple[float, float, float, float],
        stamp_s: float,
    ) -> EncoderReading:
        """Quantize one ideal joint sample, preserving fractional tick motion."""
        if len(positions_rad) != WHEEL_COUNT or not all(
            isfinite(position) for position in positions_rad
        ):
            raise ValueError('Four finite wheel positions are required')
        if not isfinite(stamp_s):
            raise ValueError('Sample timestamp must be finite')

        previous_positions = self._previous_true_positions
        previous_stamp_s = self._previous_stamp_s
        if (
            previous_positions is None
            or previous_stamp_s is None
            or stamp_s <= previous_stamp_s
        ):
            return self._initialize(positions_rad, stamp_s)

        radians_to_ticks = self._ticks_per_revolution / (2.0 * pi)
        for index, position in enumerate(positions_rad):
            delta_ticks = (
                (position - previous_positions[index])
                * radians_to_ticks
                * self._wheel_scales[index]
            )
            if delta_ticks != 0.0 and self._tick_noise_stddev > 0.0:
                delta_ticks += self._random.gauss(
                    0.0, self._tick_noise_stddev
                )
            self._continuous_counts[index] += delta_ticks

        counts = tuple(round(value) for value in self._continuous_counts)
        previous_counts = self._previous_counts
        assert previous_counts is not None
        dt_s = stamp_s - previous_stamp_s
        ticks_to_radians = 2.0 * pi / self._ticks_per_revolution
        positions = tuple(count * ticks_to_radians for count in counts)
        velocities = tuple(
            (counts[index] - previous_counts[index])
            * ticks_to_radians
            / dt_s
            for index in range(WHEEL_COUNT)
        )

        self._previous_true_positions = positions_rad
        self._previous_counts = counts
        self._previous_stamp_s = stamp_s
        return EncoderReading(counts, positions, velocities)

    def _initialize(
        self,
        positions_rad: tuple[float, float, float, float],
        stamp_s: float,
    ) -> EncoderReading:
        radians_to_ticks = self._ticks_per_revolution / (2.0 * pi)
        self._continuous_counts = [
            position * radians_to_ticks * self._wheel_scales[index]
            for index, position in enumerate(positions_rad)
        ]
        counts = tuple(round(value) for value in self._continuous_counts)
        ticks_to_radians = 2.0 * pi / self._ticks_per_revolution
        positions = tuple(count * ticks_to_radians for count in counts)
        velocities = (0.0, 0.0, 0.0, 0.0)
        self._previous_true_positions = positions_rad
        self._previous_counts = counts
        self._previous_stamp_s = stamp_s
        return EncoderReading(counts, positions, velocities)
