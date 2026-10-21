"""ROS-independent proportional motion controller with stopping envelopes."""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


def _finite(*values: float) -> bool:
    return all(math.isfinite(float(value)) for value in values)


@dataclass(frozen=True)
class MotionConfig:
    linear_kp: float = 1.0
    yaw_kp: float = 2.0
    linear_tolerance_m: float = 0.005
    yaw_tolerance_rad: float = math.radians(1)
    min_linear_speed_mps: float = 0.01
    min_angular_speed_rad_s: float = 0.08
    stop_linear_speed_mps: float = 0.005
    stop_angular_speed_rad_s: float = 0.02
    linear_deceleration_mps2: float = 0.35
    angular_deceleration_rad_s2: float = 0.35
    hold_time_s: float = 0.5
    candidate_settings: bool = True
    linear_x_kp: float | None = None
    linear_y_kp: float | None = None

    def __post_init__(self) -> None:
        values = asdict(self)
        values.pop("candidate_settings")
        overrides = (values.pop("linear_x_kp"), values.pop("linear_y_kp"))
        if not _finite(*values.values()) or any(value <= 0 for value in values.values()):
            raise ValueError("motion settings must be finite and positive")
        for value in overrides:
            if value is not None and (
                isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value <= 0
            ):
                raise ValueError("axis-specific motion gains must be finite and positive")

    @classmethod
    def from_json(cls, path: str | Path | None = None) -> "MotionConfig":
        path = Path(path) if path else Path(__file__).with_name("motion_settings.json")
        with path.open(encoding="utf-8") as stream:
            return cls(**json.load(stream))

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class MotionStep:
    vx_mps: float
    vy_mps: float
    wz_rad_s: float
    done: bool
    error: float
    actual_displacement: float
    overshoot: float


class MotionController:
    def __init__(self, start_pose: Any, vx: float, vy: float, wz: float,
                 target_displacement: float, is_rotation: bool,
                 config: MotionConfig | None = None):
        self.config = config or MotionConfig.from_json()
        vals = (start_pose.x_m, start_pose.y_m, start_pose.yaw_rad, vx, vy, wz, target_displacement)
        if not _finite(*vals) or target_displacement <= 0:
            raise ValueError("motion inputs must be finite and target positive")
        self.start = (start_pose.x_m, start_pose.y_m, start_pose.yaw_rad)
        self.rotation = bool(is_rotation)
        if self.rotation and (vx != 0 or vy != 0):
            raise ValueError("rotation goal requires vx=vy=0")
        if not self.rotation and wz != 0:
            raise ValueError("translation goal requires wz=0")
        mag = abs(wz) if self.rotation else math.hypot(vx, vy)
        if mag <= 0:
            raise ValueError("initial goal velocity must be nonzero")
        self.direction = math.copysign(1.0, wz) if self.rotation else 1.0
        if self.rotation:
            self.ux, self.uy = 0.0, 0.0
        else:
            c, s = math.cos(start_pose.yaw_rad), math.sin(start_pose.yaw_rad)
            self.ux, self.uy = ((c * vx - s * vy) / mag, (s * vx + c * vy) / mag)
        self._initial_speed = mag
        self._initial_vx, self._initial_vy = vx, vy
        self.target = target_displacement
        self.last_yaw = start_pose.yaw_rad
        self.yaw_accum = 0.0
        self.hold_since: float | None = None

    def step(self, pose: Any, linear_speed_mps: float, angular_speed_rad_s: float, now_s: float) -> MotionStep:
        values = (pose.x_m, pose.y_m, pose.yaw_rad, linear_speed_mps, angular_speed_rad_s, now_s)
        if not _finite(*values):
            raise ValueError("motion measurements must be finite")
        dyaw = (pose.yaw_rad - self.last_yaw + math.pi) % (2 * math.pi) - math.pi
        self.yaw_accum += dyaw
        self.last_yaw = pose.yaw_rad
        dx, dy = pose.x_m - self.start[0], pose.y_m - self.start[1]
        actual = self.yaw_accum * self.direction if self.rotation else dx * self.ux + dy * self.uy
        error = self.target - actual
        overshoot = max(0.0, -error)
        tol = self.config.yaw_tolerance_rad if self.rotation else self.config.linear_tolerance_m
        speed_tol = self.config.stop_angular_speed_rad_s if self.rotation else self.config.stop_linear_speed_mps
        speed = abs(angular_speed_rad_s if self.rotation else linear_speed_mps)
        stopped = speed <= speed_tol
        in_tol = abs(error) <= tol
        if in_tol and stopped:
            if self.hold_since is None:
                self.hold_since = now_s
        else:
            self.hold_since = None
        done = self.hold_since is not None and now_s - self.hold_since >= self.config.hold_time_s
        if in_tol:
            cmd = 0.0
        else:
            if self.rotation:
                kp = self.config.yaw_kp
            elif self._initial_vx != 0 and self._initial_vy == 0:
                kp = self.config.linear_x_kp or self.config.linear_kp
            elif self._initial_vy != 0 and self._initial_vx == 0:
                kp = self.config.linear_y_kp or self.config.linear_kp
            else:
                kp = self.config.linear_kp
            decel = self.config.angular_deceleration_rad_s2 if self.rotation else self.config.linear_deceleration_mps2
            ceiling = abs(self._initial_speed)
            magnitude = min(ceiling, kp * abs(error), math.sqrt(2 * decel * abs(error)))
            minimum = self.config.min_angular_speed_rad_s if self.rotation else self.config.min_linear_speed_mps
            if magnitude > 0:
                magnitude = min(ceiling, max(minimum, magnitude))
            cmd = math.copysign(magnitude, error) * self.direction
        if self.rotation:
            return MotionStep(0.0, 0.0, cmd, done, error, actual, overshoot)
        c, s = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
        return MotionStep(c * self.ux * cmd + s * self.uy * cmd,
                          -s * self.ux * cmd + c * self.uy * cmd,
                          0.0, done, error, actual, overshoot)
