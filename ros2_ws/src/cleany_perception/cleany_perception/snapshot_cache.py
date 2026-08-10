from __future__ import annotations

import threading
import time
from collections import OrderedDict
from collections.abc import Callable
from dataclasses import dataclass

from cleany_perception.core.models import (
    Detection2D,
    RgbdSnapshot,
    RigidTransform,
)


@dataclass(frozen=True)
class CachedDetectionSnapshot:
    snapshot: RgbdSnapshot
    detections: tuple[Detection2D, ...]
    capture_transform: RigidTransform
    color_frame: str


class DetectionSnapshotCache:
    def __init__(
        self,
        maximum_entries: int = 2,
        ttl_seconds: float = 120.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if maximum_entries <= 0:
            raise ValueError('Snapshot cache size must be positive')
        if ttl_seconds <= 0.0:
            raise ValueError('Snapshot cache TTL must be positive')
        self._maximum_entries = maximum_entries
        self._ttl_seconds = ttl_seconds
        self._clock = clock
        self._entries: OrderedDict[
            str,
            tuple[float, CachedDetectionSnapshot],
        ] = OrderedDict()
        self._lock = threading.Lock()

    def put(
        self,
        snapshot_id: str,
        snapshot: CachedDetectionSnapshot,
    ) -> None:
        if not snapshot_id:
            raise ValueError('Snapshot ID must not be empty')
        with self._lock:
            now = self._clock()
            self._remove_expired(now)
            self._entries.pop(snapshot_id, None)
            self._entries[snapshot_id] = (now, snapshot)
            while len(self._entries) > self._maximum_entries:
                self._entries.popitem(last=False)

    def get(self, snapshot_id: str) -> CachedDetectionSnapshot | None:
        if not snapshot_id:
            return None
        with self._lock:
            now = self._clock()
            self._remove_expired(now)
            stored = self._entries.get(snapshot_id)
            if stored is None:
                return None
            return stored[1]

    def __len__(self) -> int:
        with self._lock:
            self._remove_expired(self._clock())
            return len(self._entries)

    def _remove_expired(self, now: float) -> None:
        expired = [
            snapshot_id
            for snapshot_id, (created_at, _snapshot) in self._entries.items()
            if now - created_at >= self._ttl_seconds
        ]
        for snapshot_id in expired:
            del self._entries[snapshot_id]
