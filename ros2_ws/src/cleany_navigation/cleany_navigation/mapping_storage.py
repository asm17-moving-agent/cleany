"""Atomic map snapshots with the OccupancyGrid's planar origin preserved."""

from __future__ import annotations
from pathlib import Path
import json
import math
import os
import uuid
import yaml


def atomic_bytes(path: Path, data: bytes) -> None:
    temporary = path.with_name("." + path.name + "-" + uuid.uuid4().hex)
    try:
        with temporary.open("xb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_json(path: Path, value) -> None:
    atomic_bytes(
        path, (json.dumps(value, indent=2, ensure_ascii=False) + "\n").encode()
    )


def save_grid(
    output: Path,
    width: int,
    height: int,
    resolution: float,
    origin: tuple[float, float, float],
    data,
) -> dict:
    if (
        width <= 0
        or height <= 0
        or width * height > 16_000_000
        or len(data) != width * height
    ):
        raise ValueError("Invalid map dimensions")
    if (
        not math.isfinite(resolution)
        or resolution <= 0
        or not all(map(math.isfinite, origin))
    ):
        raise ValueError("Invalid map origin or resolution")
    if any(v < -1 or v > 100 for v in data):
        raise ValueError("Invalid occupancy value")
    pixels = bytes(
        205 if v < 0 else round(254 * (1 - v / 100))
        for y in range(height - 1, -1, -1)
        for v in data[y * width : (y + 1) * width]
    )
    pgm = f"P5\n{width} {height}\n255\n".encode() + pixels
    name = "map-" + uuid.uuid4().hex + ".pgm"
    document = dict(
        image=name,
        mode="trinary",
        resolution=resolution,
        origin=list(origin),
        negate=0,
        occupied_thresh=0.65,
        free_thresh=0.196,
    )
    output.mkdir(parents=True, exist_ok=True)
    atomic_bytes(output / name, pgm)
    atomic_bytes(output / "map.pgm", pgm)
    atomic_bytes(
        output / "map.yaml", yaml.safe_dump(document, sort_keys=False).encode()
    )
    # The YAML always references an immutable image, including across interrupted writes.
    for old in output.glob("map-*.pgm"):
        if old.name != name:
            old.unlink()
    return document


def planar_origin(pose) -> tuple[float, float, float]:
    q = pose.orientation
    components = (q.x, q.y, q.z, q.w)
    if (
        not all(map(math.isfinite, components))
        or abs(sum(v * v for v in components) - 1) > 1e-4
        or abs(q.x) > 1e-5
        or abs(q.y) > 1e-5
    ):
        raise ValueError("Map origin must be a normalized planar quaternion")
    return (
        pose.position.x,
        pose.position.y,
        math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z),
    )
