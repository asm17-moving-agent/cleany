"""ROS-independent facility geometry, expressed in the dashboard's map frame."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, isfinite, radians, sin
from pathlib import Path

import yaml


@dataclass(frozen=True)
class WallSegment:
    name: str
    start: tuple[float, float]
    end: tuple[float, float]
    guard: bool = False
    material: str = "solid"


@dataclass(frozen=True)
class FacilityLayout:
    raw: dict
    units_per_meter: float
    origin_map: tuple[float, float]
    segments: tuple[WallSegment, ...]

    def world(self, point) -> tuple[float, float]:
        return (
            (point[0] - self.origin_map[0]) / self.units_per_meter,
            (self.origin_map[1] - point[1]) / self.units_per_meter,
        )

    def map_point(self, point) -> tuple[float, float]:
        return (
            self.origin_map[0] + point[0] * self.units_per_meter,
            self.origin_map[1] - point[1] * self.units_per_meter,
        )


def _point(value, length=2) -> tuple[float, ...]:
    try:
        result = tuple(float(x) for x in value)
        if len(result) != length or not all(isfinite(x) for x in result):
            raise ValueError()
        return result
    except (TypeError, ValueError) as error:
        raise ValueError(f"expected {length} finite coordinates: {value}") from error


def load_facility_layout(path: Path) -> FacilityLayout:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("schema_version") != 1:
        raise ValueError("unsupported facility layout schema")
    try:
        scale = float(raw["transform"]["units_per_meter"])
        if not isfinite(scale) or scale <= 0:
            raise ValueError("units_per_meter must be positive")
        origin = _point(raw["transform"]["origin_map"])
        for key in ("thickness_m", "height_m"):
            value = float(raw["wall"][key])
            if not isfinite(value) or value <= 0:
                raise ValueError(f"wall {key} must be positive")
        _point(raw["spawn_pose"], 6)
        bounds = _point(raw["floor_bounds"], 4)
        tile = float(raw["floor"]["texture_tile_m"])
        if not isfinite(tile) or tile <= 0 or raw["floor"]["material"] != "grey_felt":
            raise ValueError("invalid felt floor configuration")
        if bounds[0] >= bounds[2] or bounds[1] >= bounds[3]:
            raise ValueError("invalid floor_bounds")
        color = _point(raw["wall"]["color"], 4)
        if not all(0 <= x <= 1 for x in color):
            raise ValueError("wall color must be RGBA")
        result = FacilityLayout(raw, scale, origin, ())
        segments, ids, geometry = [], set(), set()

        def add_path(entry, guard=False):
            name = entry["id"]
            if not isinstance(name, str) or not name or name in ids:
                raise ValueError(f"duplicate/invalid geometry id: {name}")
            ids.add(name)
            material = entry.get("material", "solid")
            if material not in ("solid", "glass"):
                raise ValueError(f"{name}: invalid wall material")
            points = [result.world(_point(p)) for p in entry["points"]]
            offset = _point(entry.get("offset_m", [0, 0]))
            if len(points) < 2:
                raise ValueError(f"{name}: needs at least two points")
            for i, (a, b) in enumerate(zip(points, points[1:])):
                a, b = (
                    tuple(x + y for x, y in zip(a, offset)),
                    tuple(x + y for x, y in zip(b, offset)),
                )
                if hypot(b[0] - a[0], b[1] - a[1]) < 1e-6:
                    raise ValueError(f"{name}: zero length wall")
                key = tuple(sorted((a, b)))
                if key in geometry:
                    raise ValueError(f"{name}: duplicate wall")
                geometry.add(key)
                segments.append(WallSegment(f"{name}_{i:02d}", a, b, guard, material))

        for entry in raw["walls"]:
            add_path(entry)
        for entry in raw.get("arcs", []):
            center = _point(entry["center"])
            radius = (
                float(entry["radius_units"])
                + float(entry.get("outward_offset_m", 0)) * scale
            )
            n = entry["segments"]
            start, end = float(entry["start_deg"]), float(entry["end_deg"])
            if (
                not all(isfinite(v) for v in (radius, start, end))
                or radius <= 0
                or not isinstance(n, int)
                or not 2 <= n <= 128
                or start == end
            ):
                raise ValueError("invalid arc")
            points = [
                [
                    center[0] + radius * cos(radians(start + (end - start) * i / n)),
                    center[1] + radius * sin(radians(start + (end - start) * i / n)),
                ]
                for i in range(n + 1)
            ]
            add_path({"id": entry["id"], "points": points})
        for entry in raw.get("stair_boundaries", []):
            add_path(entry, True)
        room_ids = set()
        for room in raw["rooms"]:
            if room["id"] in room_ids or room["columns"] not in (2, 3):
                raise ValueError("invalid/duplicate room")
            room_ids.add(room["id"])
            _point(room["center"])
            if not isfinite(float(room["rotation_deg"])):
                raise ValueError("invalid room rotation")
        for door in raw["doors"]:
            if len(door["points"]) != 2 or _point(door["points"][0]) == _point(
                door["points"][1]
            ):
                raise ValueError("invalid door opening")
            if door.get("material", "solid") not in ("solid", "glass"):
                raise ValueError("invalid door material")
            if not isfinite(float(door.get("open_angle_deg", 0))):
                raise ValueError("invalid door angle")
        for section, keys in (
            ("glass", ("thickness_m", "frame_width_m")),
            ("central_table", ("height_m",)),
            ("ceiling", ("height_m", "thickness_m")),
        ):
            for key in keys:
                value = float(raw[section][key])
                if not isfinite(value) or value <= 0:
                    raise ValueError(f"invalid {section} {key}")
            rgba = _point(raw[section]["color"], 4)
            if not all(0 <= x <= 1 for x in rgba):
                raise ValueError(f"invalid {section} color")
        transparency = float(raw["glass"]["transparency"])
        if not isfinite(transparency) or not 0 <= transparency < 1:
            raise ValueError("invalid glass transparency")
        table = _point(raw["central_table"]["bounds"], 4)
        if table[0] >= table[2] or table[1] >= table[3]:
            raise ValueError("invalid table bounds")
        from cleany_gazebo_sim.route_control import RouteLimits

        RouteLimits(**raw["route_control"])
        for point in raw["route_xy"]:
            _point(point)
        furniture = raw["room_furniture"]
        if min(_point(furniture["desk_size_units"])) <= 0:
            raise ValueError("invalid desk size")
        for key in (
            "column_spacing_units",
            "row_offset_units",
            "desk_top_m",
            "chair_offset_m",
        ):
            if not isfinite(float(furniture[key])) or float(furniture[key]) <= 0:
                raise ValueError(f"invalid furniture {key}")
        return FacilityLayout(raw, scale, origin, tuple(segments))
    except (KeyError, TypeError) as error:
        raise ValueError(f"incomplete facility layout: {error}") from error
