"""Bounded, ROS-independent serialization and live event cache."""

import math
import threading
import time
from collections import deque
from collections.abc import Mapping
from itertools import islice
from typing import Any
from uuid import uuid4

from .recording import RecordingControls


def preview(value: Any, *, budget: int = 2048, depth: int = 10) -> Any:
    """Spend a shared element budget BEFORE expanding large ROS sequences."""
    remaining = [budget]

    def walk(item, level):
        remaining[0] -= 1
        if remaining[0] < 0 or level <= 0:
            return {"truncated": True}
        if isinstance(item, float) and not math.isfinite(item):
            return str(item)
        if isinstance(item, (str, int, float, bool)) or item is None:
            return (
                item[:4096] + "…"
                if isinstance(item, str) and len(item) > 4096
                else item
            )
        if hasattr(item, "get_fields_and_field_types"):
            return {
                key: walk(getattr(item, key), level - 1)
                for key in item.get_fields_and_field_types()
            }
        if isinstance(item, Mapping):
            return {str(k): walk(v, level - 1) for k, v in islice(item.items(), 64)}
        if hasattr(item, "__len__") and hasattr(item, "__iter__"):
            limit = min(len(item), max(0, remaining[0]), 256)
            data = [walk(item[i], level - 1) for i in range(limit)]
            return (
                {"items": data, "length": len(item), "truncated": True}
                if limit < len(item)
                else data
            )
        return str(item)[:4096]

    return walk(value, depth)


def yaw(q):
    return math.atan2(2 * (q.z * q.w + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def transform_point(x: float, y: float, transform: list[float]) -> list[float]:
    tx, ty, angle = transform
    return [
        tx + math.cos(angle) * x - math.sin(angle) * y,
        ty + math.sin(angle) * x + math.cos(angle) * y,
    ]


def grid_origin(origin: list[float], transform: list[float]) -> list[float]:
    return [*transform_point(origin[0], origin[1], transform), origin[2] + transform[2]]


class NoCameraPreview:
    """Recording never decodes images or creates browser camera leases."""
    def selected(self):
        return set()
    def receive(self, topic, message):
        pass


class ObservationBuffer:
    """Producer never waits for browser or disk. Telemetry coalesces; events have a ring."""

    def __init__(self, event_limit=2048):
        self.cameras = NoCameraPreview()
        self.recording = RecordingControls()
        self.lock = threading.Lock()
        self.latest = {}
        self.events = deque(maxlen=event_limit)
        self.source_categories = {}
        self.source_roles = {}
        self.topic_revision = 0
        self.sequence = 0
        self.evicted_through = 0
        self.selected = {}
        self.boot_id = str(uuid4())

    def reset_sources(self, config, revision):
        from .sources import resolved, SCHEMA, source_roles
        with self.lock:
            keep = {'resources', 'recording_policy', 'recording_status'}
            previous = self.latest.get(('config', 'monitor'), {}).get('data', {})
            self.latest = {key: row for key,row in self.latest.items() if row['kind'] in keep}
            self.topic_revision = revision
            self.source_roles = source_roles(config)
            self.source_categories = {topic: SCHEMA[key]['group'] for key,value in resolved(config).items() for topic in (value if isinstance(value,list) else [value])}
        self.put('config', 'monitor', {**previous, **config, 'topic_revision': revision})

    def put(
        self,
        kind: str,
        key: str,
        data: Any,
        *,
        critical: bool = False,
        ros_time: float | None = None,
        received_at: float | None = None,
    ) -> dict:
        with self.lock:
            self.sequence += 1
            row = {
                "seq": self.sequence,
                "topic_revision": self.topic_revision,
                "kind": kind,
                "key": key,
                "data": data,
                "received_at": time.time() if received_at is None else received_at,
                "received_monotonic": time.monotonic(),
                "ros_time": ros_time,
            }
            source = key if key in self.source_roles else data.get('topic', key) if isinstance(data, dict) else key
            if source in self.source_roles:
                row['source_id'] = self.source_roles[source]
            elif kind == 'pose' or (kind == 'navigation' and key == 'summary'):
                row['source_id'] = '_computed_motion'
            if source in self.source_categories and kind not in ('config', 'graph', 'recording_policy', 'recording_status') and not (kind == 'layer' and key == 'map'):
                row["category"] = self.source_categories[source]
            if critical:
                if len(self.events) == self.events.maxlen:
                    self.evicted_through = self.events[0]["seq"]
                self.events.append(row)
            else:
                self.latest[(kind, key)] = row
            return row

    def snapshot(self, since: int = 0) -> dict:
        with self.lock:
            events = [row for row in self.events if row["seq"] > since]
            return {
                "boot_id": self.boot_id,
                "topic_revision": self.topic_revision,
                "sequence": self.sequence,
                "gap": bool(since < self.evicted_through),
                "rows": sorted(
                    [*self.latest.values(), *events], key=lambda r: r["seq"]
                ),
            }

    def latest_data(self, kind: str, key: str) -> dict:
        with self.lock:
            row = self.latest.get((kind, key))
            return row["data"] if row else {}

    def forget(self, kind, key):
        with self.lock:
            self.latest.pop((kind, key), None)

    def select(self, client: str, topics: list[str], limit: int | None = None) -> bool:
        with self.lock:
            others = set().union(
                *(value for key, value in self.selected.items() if key != client)
            )
            if limit is not None and len(others | set(topics)) > limit:
                return False
            self.selected[client] = set(topics)
            return True

    def release(self, client: str) -> None:
        with self.lock:
            self.selected.pop(client, None)

    def selections(self) -> set[str]:
        with self.lock:
            return set().union(*self.selected.values()) if self.selected else set()
