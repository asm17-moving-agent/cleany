"""Standstill brake model for the existing ideal root-velocity actuator."""
from __future__ import annotations

import math


class StationaryBaseHold:
    """Counter arm reaction drift with bounded velocity, never pose writes."""

    def __init__(self, translation_gain_s: float, yaw_gain_s: float):
        if not all(math.isfinite(v) and v > 0. for v in (translation_gain_s, yaw_gain_s)):
            raise ValueError('Standstill gains must be finite and positive')
        self.translation_gain = translation_gain_s
        self.yaw_gain = yaw_gain_s
        self.anchor: tuple[float, float, float] | None = None
        self.samples = 0
        self.maximum_translation_error_m = 0.
        self.maximum_yaw_error_rad = 0.

    def command(self, x: float, y: float, yaw: float,
                requested: tuple[float, float, float]) -> tuple[float, float, float]:
        if any(abs(v) > 1e-8 for v in requested):
            self.anchor = None
            return requested
        if self.anchor is None:
            self.anchor = (x, y, yaw)
        dx, dy = self.anchor[0]-x, self.anchor[1]-y
        angle = math.atan2(math.sin(self.anchor[2]-yaw), math.cos(self.anchor[2]-yaw))
        self.samples += 1
        self.maximum_translation_error_m = max(self.maximum_translation_error_m, math.hypot(dx, dy))
        self.maximum_yaw_error_rad = max(self.maximum_yaw_error_rad, abs(angle))
        c, s = math.cos(yaw), math.sin(yaw)
        return (max(-.25, min(.25, self.translation_gain*(c*dx+s*dy))),
                max(-.25, min(.25, self.translation_gain*(-s*dx+c*dy))),
                max(-.5, min(.5, self.yaw_gain*angle)))

    def report(self) -> dict:
        return {'scope': 'Ideal base brake via bounded root velocity; no pose reset',
                'samples': self.samples,
                'maximum_translation_error_m': self.maximum_translation_error_m,
                'maximum_yaw_error_rad': self.maximum_yaw_error_rad}
