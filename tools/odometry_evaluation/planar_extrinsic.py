"""Pure planar LiDAR-to-base extrinsic and relative-motion conversions."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class PlanarExtrinsic:
    """Planar LiDAR pose relative to the base, with lever arm in base axes."""

    yaw_deg: float = -7.85
    lever_arm_x_m: float = 0.029
    lever_arm_y_m: float = 0.0

    def __post_init__(self) -> None:
        for name in ("yaw_deg", "lever_arm_x_m", "lever_arm_y_m"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a finite number")
            if not math.isfinite(value):
                raise ValueError(f"{name} must be a finite number")

    @classmethod
    def from_json(cls, path: Path | None) -> PlanarExtrinsic:
        if path is None:
            return cls()
        with path.open(encoding="utf-8") as stream:
            data: Any = json.load(stream)
        if not isinstance(data, dict):
            raise ValueError("extrinsic JSON must be an object")
        keys = ("yaw_deg", "lever_arm_x_m", "lever_arm_y_m")
        missing = [key for key in keys if key not in data]
        if missing:
            raise ValueError(f"missing extrinsic keys: {', '.join(missing)}")
        return cls(**{key: data[key] for key in keys})

    def to_dict(self) -> dict[str, float]:
        return {
            "yaw_deg": float(self.yaw_deg),
            "lever_arm_x_m": float(self.lever_arm_x_m),
            "lever_arm_y_m": float(self.lever_arm_y_m),
        }

    def lidar_to_base(self, dx: float, dy: float, dyaw: float) -> tuple[float, float, float]:
        """Convert LiDAR relative motion to base motion (extrinsic conjugation)."""
        angle = math.radians(self.yaw_deg)
        c, s = math.cos(angle), math.sin(angle)
        cy, sy = math.cos(dyaw), math.sin(dyaw)
        x = c * dx - s * dy + self.lever_arm_x_m * (1.0 - cy) + self.lever_arm_y_m * sy
        y = s * dx + c * dy + self.lever_arm_y_m * (1.0 - cy) - self.lever_arm_x_m * sy
        return x, y, dyaw

    def base_to_lidar(self, dx: float, dy: float, dyaw: float) -> tuple[float, float, float]:
        """Convert base relative motion to LiDAR motion; inverse of lidar_to_base."""
        angle = math.radians(self.yaw_deg)
        c, s = math.cos(angle), math.sin(angle)
        cy, sy = math.cos(dyaw), math.sin(dyaw)
        x = dx - self.lever_arm_x_m * (1.0 - cy) - self.lever_arm_y_m * sy
        y = dy - self.lever_arm_y_m * (1.0 - cy) + self.lever_arm_x_m * sy
        return c * x + s * y, -s * x + c * y, dyaw
