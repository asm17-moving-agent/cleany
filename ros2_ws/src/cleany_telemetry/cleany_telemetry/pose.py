"""Backend-independent pose validation using the source ROS clock."""
from __future__ import annotations

import math


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float | None:
    """Return counterclockwise yaw in radians from frame +X, or None if invalid."""
    if not all(math.isfinite(v) for v in (x, y, z, w)):
        return None
    norm = math.hypot(x, y, z, w)
    if not math.isfinite(norm) or norm < 1e-12:
        return None
    x, y, z, w = (v / norm for v in (x, y, z, w))
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class PoseFreshness:
    """Accept new samples in their source frame without restamping or extrapolating."""

    def __init__(self, timeout: float = 1.5, future_tolerance: float = 0.1):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("input_timeout must be positive and finite")
        if not math.isfinite(future_tolerance) or future_tolerance < 0:
            raise ValueError("future_tolerance must be nonnegative and finite")
        self.timeout = timeout
        self.future_tolerance = future_tolerance
        self._last_stamp: float | None = None
        self._last_now: float | None = None
        self._frame: str | None = None

    def accept(self, frame: str, stamp: float, now: float) -> bool:
        if not all(math.isfinite(v) for v in (stamp, now)):
            return False
        if self._last_now is not None and now < self._last_now:
            self._last_stamp = None
        self._last_now = now
        if (not frame.strip() or stamp <= 0 or now - stamp > self.timeout
                or stamp - now > self.future_tolerance):
            return False
        if frame != self._frame:
            self._last_stamp = None
        if self._last_stamp is not None and stamp <= self._last_stamp:
            return False
        self._last_stamp, self._frame = stamp, frame
        return True
