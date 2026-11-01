"""ROI obstacle confirmation and final distance/velocity limiting."""
from dataclasses import dataclass
import math
import numpy as np
from .core import Settings
from .observation import Observation


def clearance(points: np.ndarray, settings: Settings) -> np.ndarray:
    """Return planar footprint clearance after the configured uncertainty margin."""
    outside = np.maximum(np.abs(points[:, :2])-[settings.half_length, settings.half_width], 0)
    return np.linalg.norm(outside, axis=1)-settings.margin-3*settings.max_sigma_m


@dataclass(frozen=True)
class Detection:
    available: bool
    points: np.ndarray
    emergency: bool = False
    held: bool = False
    pending_zones: int = 0


class ProximityDetector:
    """Confirm adjacent ROI returns across fresh frames; latch close singletons.

    Call once per received frame using its monotonic receipt time in seconds.
    Missing confirmed returns briefly stop motion instead of reusing old points
    in a moving base frame. NO_RETURN alone never starts an obstacle track.
    """
    def __init__(self, settings: Settings):
        self.s = settings
        self.roi = np.zeros(64, dtype=bool)
        self.roi[[r*8+c for r in settings.guard_rows for c in settings.guard_columns]] = True
        self.reset()

    def reset(self) -> None:
        self.since = np.full(64, np.nan)
        self.last_hit = np.full(64, -math.inf)
        self.active = np.zeros(64, dtype=bool)
        self.last_time = self.emergency_at = -math.inf
        self.result = Detection(False, np.empty((0, 3)))

    def _supported(self, near: np.ndarray, points: np.ndarray) -> np.ndarray:
        remaining = set(np.flatnonzero(near))
        supported = np.zeros(64, dtype=bool)
        while remaining:
            component = [remaining.pop()]
            for index in component:
                r, c = divmod(index, 8)
                for row, col in ((r-1, c), (r+1, c), (r, c-1), (r, c+1)):
                    neighbour = row*8+col
                    if (0 <= row < 8 and 0 <= col < 8 and neighbour in remaining
                            and np.linalg.norm(points[index]-points[neighbour]) <= self.s.cluster_distance_m):
                        remaining.remove(neighbour)
                        component.append(neighbour)
            if len(component) >= self.s.min_cluster_zones:
                supported[component] = True
        return supported

    def update(self, observation: Observation, now: float) -> Detection:
        if not math.isfinite(now):
            raise ValueError('invalid detection time')
        if now == self.last_time:
            return self.result
        if now < self.last_time or now-self.last_time > self.s.source_timeout_s or not observation.available:
            self.reset()
        self.last_time = now
        if not observation.available:
            return self.result
        valid = self.roi & observation.ranges
        distances = clearance(observation.points, self.s)
        urgent_limit = self.s.emergency_clearance_m
        if now-self.emergency_at < self.s.release_s:
            urgent_limit += self.s.hysteresis_m
        if np.any(valid & (distances <= urgent_limit)):
            self.emergency_at = now
        near = valid & (distances <= self.s.obstacle_clearance_m+self.active*self.s.hysteresis_m)
        supported = self._supported(near, observation.points)
        self.since[~supported] = np.nan
        self.since[supported & np.isnan(self.since)] = now
        self.last_hit[supported | (self.active & near)] = now
        self.active = ((self.active & (now-self.last_hit < self.s.release_s))
                       | (supported & (now-self.since >= self.s.confirm_s)))
        current = self.active & near
        self.result = Detection(True, observation.points[current].copy(),
            emergency=now-self.emergency_at < self.s.release_s,
            held=bool(self.active.any() and not current.any()),
            pending_zones=int(np.count_nonzero(supported & ~self.active)))
        return self.result


def limit_velocity(command, measured, detection: Detection, age_s: float,
                   settings: Settings):
    """Limit only detected obstacles, preserving direction and command curvature."""
    zero = (0., 0., 0.)
    if (not settings.calibrated or not detection.available or not math.isfinite(age_s)
            or not 0 <= age_s <= settings.source_timeout_s
            or not all(math.isfinite(v) for v in (*command, *measured))):
        return zero, 'TOF_UNAVAILABLE'
    if detection.emergency:
        return zero, 'PROXIMITY_STOP'
    if detection.held:
        return zero, 'OBSTACLE_HOLD'
    scale = min(1., settings.max_speed/max(math.hypot(*command[:2]), 1e-9),
                settings.max_yaw_rate/max(abs(command[2]), 1e-9))
    radius = math.hypot(settings.half_length, settings.half_width)
    for (x, y, _), gap in zip(detection.points, clearance(detection.points, settings)):
        if gap <= 0:
            return zero, 'PROXIMITY_STOP'
        vx, vy, wz = command
        mx, my, mw = measured
        closing = (math.hypot(vx, vy) if vx*x > 0 or vy*y > 0 else 0.)+abs(wz)*radius
        actual = (math.hypot(mx, my) if mx*x > 0 or my*y > 0 else 0.)+abs(mw)*radius
        latency = settings.reaction_s+settings.confirm_s+age_s
        safe = math.sqrt((settings.deceleration*latency)**2+2*settings.deceleration*gap)-settings.deceleration*latency
        if actual > safe:
            return zero, 'BRAKING_ENVELOPE'
        if closing > 0:
            scale = min(scale, safe/closing)
    return tuple(float(v*scale) for v in command), 'LIMITED' if scale < .999 else 'CLEAR'
