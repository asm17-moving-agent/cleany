"""Bounded Cartographer observations and texture decoding, independent of ROS."""

from __future__ import annotations

import base64
import io
import math
import zlib
from dataclasses import dataclass, field
from typing import Any
from uuid import uuid4

from .observation import grid_origin, transform_point, yaw


def pose2(pose: Any) -> list[float]:
    q = pose.orientation
    values = [pose.position.x, pose.position.y, q.x, q.y, q.z, q.w]
    if (
        not all(math.isfinite(v) for v in values)
        or abs(sum(v * v for v in values[2:]) - 1) > 0.01
    ):
        raise ValueError("Invalid submap pose")
    # This viewer deliberately represents planar SLAM, not arbitrary 3D slices.
    if abs(q.x) > 0.01 or abs(q.y) > 0.01:
        raise ValueError("Non-planar submap; 2D SLAM viewer required")
    return [pose.position.x, pose.position.y, yaw(q)]


def texture_png(texture: Any, pixel_limit: int) -> dict:
    """Decode interleaved intensity/alpha with a strict gzip expansion limit.

    Cartographer encodes additive intensity separately from subtractive alpha.
    Preserve both as PNG layers: black source-over, then white additive (lighter).
    Pixel column advances along local -Y, row along local -X (OgreSlice).
    """
    width, height = int(texture.width), int(texture.height)
    resolution = float(texture.resolution)
    count = width * height
    if min(width, height) <= 0 or count > pixel_limit:
        raise ValueError("Submap texture exceeds pixel limit")
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("Invalid submap resolution")
    if len(texture.cells) > 2 * pixel_limit + 1024:
        raise ValueError("Compressed submap exceeds byte limit")
    decoder = zlib.decompressobj(16 + zlib.MAX_WBITS)
    raw = decoder.decompress(bytes(texture.cells), 2 * count + 1)
    if (
        len(raw) != 2 * count
        or not decoder.eof
        or decoder.unused_data
        or decoder.unconsumed_tail
    ):
        raise ValueError("Invalid or oversized submap gzip payload")
    rgba = bytearray(4 * count)
    light = bytearray(4 * count)
    for i in range(count):
        intensity, alpha = raw[2 * i : 2 * i + 2]
        rgba[4 * i : 4 * i + 4] = bytes((0, 0, 0, alpha))
        light[4 * i : 4 * i + 4] = bytes((255, 255, 255, intensity))

    def encode(data: bytearray) -> str:
        from PIL import Image
        output = io.BytesIO()
        Image.frombytes("RGBA", (width, height), bytes(data)).save(output, format="PNG")
        return "data:image/png;base64," + base64.b64encode(output.getvalue()).decode()

    return {
        "width": width,
        "height": height,
        "resolution": resolution,
        "slice_pose": pose2(texture.slice_pose),
        "image": encode(rgba),
        "light_image": encode(light),
    }


def marker_geometry(message: Any, transform: Any, point_limit: int) -> dict:
    """Preserve marker identity/deletion, colors and disconnected line segments."""
    markers = []
    remaining = point_limit
    truncated = len(message.markers) > 256
    for marker in message.markers[:256]:
        identity = f"{marker.ns}:{marker.id}"
        if marker.action in (2, 3):
            markers.append({"id": identity, "action": int(marker.action)})
            continue
        if marker.action != 0 or marker.type not in (4, 5, 8):
            continue
        tf = transform(marker.header.frame_id, marker.header.stamp)
        origin = grid_origin(pose2(marker.pose), tf)
        limit = remaining if marker.type != 5 else remaining - remaining % 2
        source = marker.points[:limit]
        if len(source) != len(marker.points):
            truncated = True
        points = [transform_point(p.x, p.y, origin) for p in source]
        if not all(math.isfinite(v) for p in points for v in p):
            raise ValueError("Non-finite SLAM marker")
        color = [float(getattr(marker.color, c)) for c in ("r", "g", "b", "a")]
        if not all(math.isfinite(c) for c in color):
            raise ValueError("Non-finite marker color")
        markers.append(
            {
                "id": identity,
                "action": 0,
                "type": int(marker.type),
                "points": points,
                "color": color,
            }
        )
        remaining -= len(points)
    return {"valid": True, "markers": markers, "truncated": truncated}


@dataclass
class SlamState:
    submap_limit: int = 64
    adjustment_m: float = 0.05
    adjustment_rad: float = 0.02
    texture_bytes_limit: int = 16 * 1024 * 1024
    epoch: str = field(default_factory=lambda: str(uuid4()))
    entries: dict[str, dict] = field(default_factory=dict)
    textures: dict[str, dict] = field(default_factory=dict)
    marker_sets: dict[str, dict] = field(default_factory=dict)
    omitted_textures: dict[str, int] = field(default_factory=dict)

    def reset(self) -> None:
        self.epoch = str(uuid4())
        self.entries.clear()
        self.textures.clear()
        self.marker_sets.clear()
        self.omitted_textures.clear()

    def update(self, message: Any, transform: list[float]) -> tuple[dict, list[dict]]:
        # Keep newest submaps in large sessions; explicitly report omitted maps.
        incoming = {}
        for entry in message.submap[-self.submap_limit :]:
            key = f"{entry.trajectory_id}/{entry.submap_index}"
            if (
                key in incoming
                or min(entry.trajectory_id, entry.submap_index, entry.submap_version)
                < 0
            ):
                raise ValueError("Invalid or duplicate submap identity")
            incoming[key] = {
                "id": key,
                "trajectory_id": int(entry.trajectory_id),
                "submap_index": int(entry.submap_index),
                "version": int(entry.submap_version),
                "pose": grid_origin(pose2(entry.pose), transform),
                "frozen": bool(getattr(entry, "is_frozen", False)),
            }
        events = []
        if any(
            key in self.entries and value["version"] < self.entries[key]["version"]
            for key, value in incoming.items()
        ):
            self.reset()
            events.append({"kind": "slam_reset"})
        for key, value in incoming.items():
            before = self.entries.get(key)
            if before:
                distance = math.dist(value["pose"][:2], before["pose"][:2])
                rotation = abs(
                    math.atan2(
                        math.sin(value["pose"][2] - before["pose"][2]),
                        math.cos(value["pose"][2] - before["pose"][2]),
                    )
                )
                if distance >= self.adjustment_m or rotation >= self.adjustment_rad:
                    events.append(
                        {
                            "kind": "slam_pose_adjustment",
                            "id": key,
                            "distance_m": distance,
                            "rotation_rad": rotation,
                            "before": before["pose"],
                            "after": value["pose"],
                        }
                    )
        self.entries = incoming
        self.textures = {
            key: value
            for key, value in self.textures.items()
            if key in incoming and value["version"] <= incoming[key]["version"]
        }
        self.omitted_textures = {
            key: version
            for key, version in self.omitted_textures.items()
            if key in incoming and incoming[key]["version"] == version
        }
        return {
            "valid": True,
            "epoch": self.epoch,
            "submaps": list(incoming.values()),
            "source_count": len(message.submap),
            "truncated": len(message.submap) > self.submap_limit,
        }, events

    def pending(self) -> list[dict]:
        return [
            value
            for key, value in self.entries.items()
            if (
                key not in self.textures
                or self.textures[key]["version"] != value["version"]
            )
            and key not in self.omitted_textures
        ]

    def accept(
        self, identity: str, epoch: str, version: int, texture: dict
    ) -> dict | None:
        current = self.entries.get(identity)
        if epoch != self.epoch or not current or current["version"] < version:
            return None
        if identity in self.textures and self.textures[identity]["version"] > version:
            return None
        result = dict(texture, id=identity, epoch=epoch, version=version, valid=True)
        size = lambda item: (
            len(item.get("image", "")) + len(item.get("light_image", ""))
        )
        if size(result) > self.texture_bytes_limit:
            self.omitted_textures[identity] = version
            raise ValueError("Submap texture exceeds total cache budget")
        self.textures.pop(identity, None)
        while (
            self.textures
            and sum(size(item) for item in self.textures.values()) + size(result)
            > self.texture_bytes_limit
        ):
            key = next(iter(self.textures))
            removed = self.textures.pop(key)
            self.omitted_textures[key] = removed["version"]
        self.textures[identity] = result
        return result

    def markers(self, key: str, data: dict, limit: int) -> dict:
        current = self.marker_sets.setdefault(key, {})
        for marker in data["markers"]:
            if marker["action"] == 3:
                current.clear()
            elif marker["action"] == 2:
                current.pop(marker["id"], None)
            else:
                current[marker["id"]] = marker
        # Bound retained markers too, including publishers sending incremental ADDs.
        kept, used = {}, 0
        for identity, marker in reversed(list(current.items())):
            if len(kept) >= 256 or used + len(marker["points"]) > limit:
                continue
            kept[identity] = marker
            used += len(marker["points"])
        truncated = data["truncated"] or len(kept) != len(current)
        self.marker_sets[key] = kept
        return {
            "valid": True,
            "epoch": self.epoch,
            "markers": list(kept.values()),
            "truncated": truncated,
        }
