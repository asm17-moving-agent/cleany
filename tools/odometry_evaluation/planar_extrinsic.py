"""순수 평면 LiDAR-to-base 외부 변환과 상대 움직임 변환입니다."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class PlanarExtrinsic:
    """베이스 상대 평면 LiDAR 자세이며 lever arm은 베이스 축으로 표현합니다."""

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
        """외부 변환의 켤레 연산으로 LiDAR 상대 움직임을 베이스 움직임으로 변환합니다."""
        angle = math.radians(self.yaw_deg)
        c, s = math.cos(angle), math.sin(angle)
        cy, sy = math.cos(dyaw), math.sin(dyaw)
        x = c * dx - s * dy + self.lever_arm_x_m * (1.0 - cy) + self.lever_arm_y_m * sy
        y = s * dx + c * dy + self.lever_arm_y_m * (1.0 - cy) - self.lever_arm_x_m * sy
        return x, y, dyaw

    def base_to_lidar(self, dx: float, dy: float, dyaw: float) -> tuple[float, float, float]:
        """베이스 상대 움직임을 LiDAR 움직임으로 변환하며 lidar_to_base의 역연산입니다."""
        angle = math.radians(self.yaw_deg)
        c, s = math.cos(angle), math.sin(angle)
        cy, sy = math.cos(dyaw), math.sin(dyaw)
        x = dx - self.lever_arm_x_m * (1.0 - cy) - self.lever_arm_y_m * sy
        y = dy - self.lever_arm_y_m * (1.0 - cy) + self.lever_arm_x_m * sy
        return c * x + s * y, -s * x + c * y, dyaw
