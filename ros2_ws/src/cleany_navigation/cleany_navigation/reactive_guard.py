"""Direct-observation planar braking envelope; no voxel map or pan alignment."""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import numpy as np

from cleany_navigation.voxel_guard import GuardConfig, twist_pose


ZERO = (0., 0., 0.)


@dataclass
class InputWatchdog:
    source_timeout_s: float = .5
    command_timeout_s: float = .5
    last_ros: float | None = None
    advanced_wall: float = -math.inf
    fault: str = ''
    sources: dict[str, tuple[float, float]] = field(default_factory=dict)

    def tick(self, now: float, wall: float) -> None:
        if not math.isfinite(now) or not math.isfinite(wall):
            raise ValueError('INVALID_CLOCK')
        if self.last_ros is not None and now < self.last_ros-.01:
            self.fault = 'CLOCK_RESET_RESTART_REQUIRED'
        if self.last_ros != now:
            self.advanced_wall = wall
        self.last_ros = now
        if self.fault:
            raise ValueError(self.fault)
        if wall-self.advanced_wall > self.command_timeout_s:
            raise ValueError('CLOCK_TIMEOUT')

    def check(self, now: float, wall: float, command_wall: float,
              raw_wall: float, required: tuple[str, ...]) -> float:
        self.tick(now, wall)
        if any(not 0 <= wall-t <= self.command_timeout_s for t in (command_wall, raw_wall)):
            raise ValueError('COMMAND_TIMEOUT')
        ages = []
        for name in required:
            stamp, received = self.sources.get(name, (-math.inf, -math.inf))
            if (not 0 <= now-stamp <= self.source_timeout_s
                    or not 0 <= wall-received <= self.source_timeout_s):
                raise ValueError('STALE_'+name.upper())
            ages.append(now-stamp)
        return max(ages, default=0.)


class PlanarBrakingGuard:
    """Enclose the complete parked body in XY; retain existing braking bounds.

    Holding full speed for reaction + braking time overestimates proportional
    braking distance. Sample padding also covers motion between sweep poses.
    This checks observed surfaces, not unobserved space or moving obstacles.
    """
    def __init__(self, low: np.ndarray, high: np.ndarray, config: GuardConfig):
        self.low, self.high = np.asarray(low, dtype=float), np.asarray(high, dtype=float)
        if (self.low.shape != (2,) or self.high.shape != (2,)
                or not np.isfinite([self.low, self.high]).all()
                or (self.high <= self.low).any()):
            raise ValueError('Invalid body footprint bounds')
        self.config = config
        self.radius = float(np.linalg.norm(np.maximum(np.abs(self.low), np.abs(self.high))))

    def hits(self, points: np.ndarray, velocity: np.ndarray, age: float, extra: float = 0.) -> bool:
        c = self.config
        speed = float(np.linalg.norm(velocity[:2]))
        horizon = c.reaction_s+age+max(speed/c.linear_deceleration,
                                      abs(velocity[2])/c.angular_deceleration)+extra
        if not math.isfinite(horizon) or not 0 <= horizon <= 5:
            raise ValueError('Unbounded braking horizon')
        padding = c.margin+(speed+abs(velocity[2])*self.radius)*c.step_s/2
        radius = self.radius+padding+speed*horizon
        nearby = points[(np.abs(points) <= radius).all(axis=1)]
        if not len(nearby):
            return False
        times = [0.] if not np.any(velocity) else np.linspace(0, horizon, int(np.ceil(horizon/c.step_s))+1)
        for seconds in times:
            pose = twist_pose(velocity, seconds)
            moved = (nearby-pose[:2, 3])@pose[:2, :2]
            if ((moved >= self.low-padding) & (moved <= self.high+padding)).all(axis=1).any():
                return True
        return False

    def evaluate(self, points: np.ndarray, command: np.ndarray, measured: np.ndarray,
                 source_age: float) -> tuple[np.ndarray, str]:
        points, command, measured = [np.asarray(a, dtype=float) for a in (points, command, measured)]
        if (points.ndim != 2 or points.shape[1] != 2 or command.shape != (3,)
                or measured.shape != (3,) or not math.isfinite(source_age) or source_age < 0
                or any(not np.isfinite(a).all() for a in (points, command, measured))):
            raise ValueError('Invalid collision input')
        c = self.config
        # Use the serial guard's exact command limiting before collision tests.
        # Actual measured velocity, including overshoot, remains in the sweep.
        command = command.copy()
        command[:2] *= min(1., c.max_speed/max(np.linalg.norm(command[:2]), 1e-12))
        command[2] = np.clip(command[2], -c.max_yaw_rate, c.max_yaw_rate)
        if self.hits(points, measured, source_age) or self.hits(points, command, source_age):
            return np.zeros(3), 'STOP_OBSTACLE'
        if self.hits(points, command, source_age, c.slowdown_lookahead_s):
            return command*c.slowdown_ratio, 'SLOW_OBSTACLE'
        return command.copy(), 'PASS'
