"""Pure result construction: rejected observations never become error estimates."""

from dataclasses import asdict
import math
from numbers import Integral, Real
import json
import os
from pathlib import Path
import tempfile
from typing import Any, Callable

from lidar_scan_matcher import ScanMatchResult


def json_finite(value: Any) -> Any:
    if isinstance(value, bool):
        return value
    if isinstance(value, Integral):
        return int(value)
    if isinstance(value, Real):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: json_finite(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_finite(item) for item in value]
    if hasattr(value, "tolist"):
        return json_finite(value.tolist())
    return value


def match_metadata(match: ScanMatchResult) -> dict[str, Any]:
    return json_finite(asdict(match))


def write_data_json(path: Path, value: Any) -> None:
    """Publish complete standard JSON atomically, without replacing an existing run."""
    encoded = json.dumps(json_finite(value), indent=2, allow_nan=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", prefix=f".{path.name}.", dir=path.parent,
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def build_leg_result(
    odom: tuple[float, float, float],
    match: ScanMatchResult,
    displacement: float,
    is_rotation: bool,
    transform: Callable[[float, float, float], tuple[float, float, float]],
) -> dict[str, Any]:
    if not math.isfinite(displacement) or displacement <= 0:
        raise ValueError("displacement must be finite and positive")
    reported = {
        "dx_m": odom[0], "dy_m": odom[1], "dyaw_deg": math.degrees(odom[2]),
    }
    data: dict[str, Any] = {
        "is_valid": match.is_valid,
        "odom_reported": reported,
        "raw_lidar_icp": match_metadata(match),
        "calibrated_base_ground_truth": None,
        "one_way_error": None,
    }
    if not match.is_valid:
        return data
    dx, dy, yaw = transform(match.delta_x_m, match.delta_y_m, match.delta_yaw_rad)
    error_x, error_y = (odom[0] - dx) * 1000, (odom[1] - dy) * 1000
    error_yaw = math.degrees((odom[2] - yaw + math.pi) % (2 * math.pi) - math.pi)
    norm = math.hypot(error_x, error_y)
    data["calibrated_base_ground_truth"] = {
        "dx_m": dx, "dy_m": dy, "dyaw_deg": math.degrees(yaw),
    }
    data["one_way_error"] = {
        "error_x_mm": error_x, "error_y_mm": error_y,
        "error_yaw_deg": error_yaw, "error_norm_mm": norm,
        "error_rate_pct": (
            abs(error_yaw) / math.degrees(displacement) * 100
            if is_rotation else norm / (displacement * 1000) * 100
        ),
    }
    return data
