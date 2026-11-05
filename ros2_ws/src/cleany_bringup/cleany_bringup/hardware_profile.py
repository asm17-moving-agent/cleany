"""Resolve hardware configuration through the installed package index."""

from __future__ import annotations
import math
from pathlib import Path
import re
import yaml


def load_profile(path: Path, *, mock=False, state_root=None, package_share=None):
    if package_share is None:
        from ament_index_python.packages import get_package_share_directory

        package_share = get_package_share_directory
    data = yaml.safe_load(Path(path).read_text())
    if data.get("schema_version") != 1:
        raise ValueError("Unsupported hardware mapping profile")
    data = dict(data)
    mount = data["mount"]
    if (
        len(mount["translation_m"]) != 3
        or not all(
            math.isfinite(float(v)) for v in mount["translation_m"] + [mount["yaw_deg"]]
        )
        or not mount.get("source")
    ):
        raise ValueError("Mount needs finite values and provenance")
    if data.get("base_frame", "base_link") != "base_link":
        raise ValueError("This Cartographer profile tracks base_link")
    for frame in (data.get("base_frame", "base_link"), data["lidar"]["frame_id"]):
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", frame):
            raise ValueError("Invalid frame name")
    for device in (data["lidar"], data["agent"]):
        if not isinstance(device["serial_port"], str) or not device[
            "serial_port"
        ].startswith("/dev/"):
            raise ValueError("Explicit serial device path required")
        if int(device["baudrate"]) <= 0:
            raise ValueError("Invalid serial baudrate")
    minimum = float(data["scan_filter"]["min_range_m"])
    half_angle = float(data["scan_filter"]["half_angle_deg"])
    if minimum != 0.30 or half_angle != 90:
        raise ValueError(
            "Mapping profile must retain front 180 degrees and mask <=0.30 m"
        )
    if not 0.5 <= float(data["teleop"]["max_wheel_speed"]) <= 10:
        raise ValueError("Invalid phone wheel limit")
    if (
        not math.isfinite(float(data.get("startup_timeout_s", 30)))
        or not 0 < float(data.get("startup_timeout_s", 30)) <= 120
    ):
        raise ValueError("Invalid startup timeout")
    base = (
        Path(package_share("cleany_base_driver"))
        / "config"
        / ("base_synthetic.yaml" if mock else data["base_profile"])
    )
    base_data = yaml.safe_load(base.read_text())
    if base_data.get("mode") != ("synthetic" if mock else "hardware"):
        raise ValueError("Base profile backend mismatch")
    data["base_config"] = str(base)
    data["base_settings"] = base_data
    data["mock"] = mock
    root = Path(state_root or "~/.local/state/cleany/hardware").expanduser().resolve()
    data["state_root"] = str(root)
    data["recordings_directory"] = str(root / "recordings")
    data["logger_config"] = str(root / "logger.yaml")
    data["scan_filter"]["front_center_rad"] = -math.radians(float(mount["yaw_deg"]))
    data["source_profile"] = str(Path(path).resolve())
    return data
