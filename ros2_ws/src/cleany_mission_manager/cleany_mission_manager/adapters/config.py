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


def load_runtime_parameters(path: str) -> dict:
    data = yaml.safe_load(Path(path).expanduser().read_text())
    try:
        parameters = data["mission_runtime"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise ValueError("runtime_config requires mission_runtime.ros__parameters") from exc
    if not isinstance(parameters, dict):
        raise ValueError("runtime parameters must be a mapping")
    return parameters


def validate_manipulation_fixture(parameters: dict, path: str, scenario: str | None = None) -> dict:
    """Check the shared logical desk fixture, without a dependency on the executor."""
    fixture = yaml.safe_load(Path(path).expanduser().read_text())
    if not isinstance(fixture, dict):
        raise ValueError("manipulation fixture must be a mapping")
    snapshot_ids = parameters.get("manipulation_mock_snapshot_ids", [
        "mock-snapshot-001", "mock-snapshot-002", "mock-snapshot-003", "mock-snapshot-004",
    ])
    if (not isinstance(snapshot_ids, (list, tuple)) or len(snapshot_ids) < 4
            or any(not isinstance(item, str) or not item.strip() for item in snapshot_ids)
            or len(set(snapshot_ids)) != len(snapshot_ids)):
        raise ValueError("action_mock requires at least four distinct registered snapshot IDs")
    snapshots = fixture.get("snapshots", {})
    if not isinstance(snapshots, dict):
        raise ValueError("fixture snapshots must be a mapping")
    for index, snapshot_id in enumerate(snapshot_ids):
        snapshot = snapshots.get(snapshot_id)
        expected = [1, 2] if index == 0 else [2] if index == 1 else []
        if (not isinstance(snapshot, dict) or snapshot.get("object_ids") != expected
                or any(type(item) is not int for item in snapshot.get("object_ids", []))):
            raise ValueError(f"fixture snapshot {snapshot_id} must register wire object IDs {expected}")
    if fixture.get("destination_id") != parameters.get("manipulation_destination", "mock_trash_bin"):
        raise ValueError("runtime and manipulation fixture destination IDs must match")
    if scenario is not None and scenario not in fixture.get("scenarios", {}):
        raise ValueError(f"unknown mock scenario: {scenario}")
    return fixture
