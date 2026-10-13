"""Validated deployment settings, separate from product and hardware decisions."""

import math
from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class TargetPose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class TargetMap:
    map_id: str
    frame_id: str
    home: TargetPose
    seats: dict[str, TargetPose]


def load_targets(path: str) -> TargetMap:
    data = yaml.safe_load(Path(path).expanduser().read_text())
    if data.get("schema_version") != 1 or not data.get("map_id") or data.get("frame_id") != "map":
        raise ValueError("target map requires schema_version=1, map_id and frame_id=map")

    def pose(values) -> TargetPose:
        if (not isinstance(values, list) or len(values) != 3
                or any(isinstance(value, bool) or not isinstance(value, (int, float))
                       or not math.isfinite(value) for value in values)):
            raise ValueError("pose must be finite [x_m, y_m, yaw_rad]")
        return TargetPose(*values)

    seats = {name: pose(value) for name, value in data["seats"].items()}
    if not seats or any(not name.startswith("seat-") for name in seats):
        raise ValueError("at least one named seat pose is required")
    return TargetMap(data["map_id"], "map", pose(data["home"]), seats)
