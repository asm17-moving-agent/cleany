"""Validated mecanum command shaping and encoder feedback adaptation."""
from __future__ import annotations

from dataclasses import dataclass
from math import isfinite

UINT32 = 0xFFFFFFFF


def newer(new: int, old: int) -> bool:
    """Compare wrapping uint32 sequences using the strict half-range rule."""
    delta = (new - old) & UINT32
    return 0 < delta < 0x80000000


def finite_twist_axes(*axes: float) -> bool:
    return len(axes) == 6 and all(isfinite(value) for value in axes)


@dataclass(frozen=True)
class Geometry:
    wheel_radius: float
    wheelbase: float
    separation: float

    def __post_init__(self) -> None:
        if not all(isfinite(x) and x > 0 for x in
                   (self.wheel_radius, self.wheelbase, self.separation)):
            raise ValueError('geometry dimensions must be positive and finite')


@dataclass(frozen=True)
class Limits:
    linear_x: float
    linear_y: float
    angular_z: float
    wheel_rad_s: float
    timeout_s: float

    def __post_init__(self) -> None:
        if not all(isfinite(x) and x > 0 for x in
                   (self.linear_x, self.linear_y, self.angular_z,
                    self.wheel_rad_s, self.timeout_s)):
            raise ValueError('limits and timeout must be positive and finite')
        if self.wheel_rad_s > 10.0:
            raise ValueError('wheel speed limit exceeds MCU contract (10 rad/s)')


def wheel_speeds(x: float, y: float, yaw: float, geometry: Geometry,
                 limits: Limits) -> tuple[float, float, float, float]:
    """Convert a bounded body-frame twist to wheel angular velocities.

    Clamp each body axis first, then scale all wheels together if a wheel limit
    is exceeded. Non-finite input axes produce four zero targets.

    Args:
        x: Forward velocity in metres per second.
        y: Leftward velocity in metres per second.
        yaw: Counterclockwise yaw rate in radians per second.
        geometry: Wheel radius, wheelbase and separation in metres.

    Returns:
        Wheel targets in radians per second, ordered FL, FR, RL, RR.
    """
    axes = (x, y, yaw)
    if not all(isfinite(v) for v in axes):
        return (0.,) * 4
    x = max(-limits.linear_x, min(limits.linear_x, x))
    y = max(-limits.linear_y, min(limits.linear_y, y))
    yaw = max(-limits.angular_z, min(limits.angular_z, yaw))
    rot = (geometry.wheelbase + geometry.separation) * yaw / 2
    raw = ((x-y-rot)/geometry.wheel_radius, (x+y+rot)/geometry.wheel_radius,
           (x+y-rot)/geometry.wheel_radius, (x-y+rot)/geometry.wheel_radius)
    peak = max(map(abs, raw))
    scale = min(1., limits.wheel_rad_s / peak) if peak else 1.
    return tuple(v*scale for v in raw)


class EncoderAdapter:
    """Convert signed int32 count samples to continuous wheel radians."""
    def __init__(self, counts_per_revolution: int = 3172) -> None:
        if counts_per_revolution <= 0:
            raise ValueError('counts per revolution must be positive')
        self.scale = counts_per_revolution
        self.boot: int | None = None
        self.previous: tuple[int, ...] | None = None
        self.totals = [0, 0, 0, 0]

    def rebase(self) -> None:
        """Discard the next delta while retaining accumulated wheel angles."""
        self.previous = None

    def update(self, boot: int, counts: tuple[int, ...]) -> tuple[float, ...]:
        """Unwrap counts while preserving accumulated angles across MCU reboots.

        The first sample after a boot change or rebase establishes a baseline.
        Between samples in one boot, each true delta must fit the int32 half-range.

        Args:
            boot: MCU boot identity used to detect encoder resets.
            counts: Four signed int32 counts in FL, FR, RL, RR order.

        Returns:
            Cumulative wheel angles in radians in the same order.

        Raises:
            ValueError: Counts do not contain four signed int32 values.
        """
        if len(counts) != 4 or any(not -0x80000000 <= c <= 0x7fffffff for c in counts):
            raise ValueError('expected four signed int32 encoder counts')
        if self.boot != boot:
            self.boot = boot
        elif self.previous is not None:
            for i, (cur, prev) in enumerate(zip(counts, self.previous)):
                delta = ((cur - prev + 0x80000000) & UINT32) - 0x80000000
                self.totals[i] += delta
        self.previous = counts
        return tuple(v * 6.283185307179586 / self.scale for v in self.totals)
