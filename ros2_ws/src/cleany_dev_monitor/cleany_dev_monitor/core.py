"""Bounded, ROS-independent serialization, event cache and durable recording."""

import hashlib
import json
import math
import sqlite3
import threading
import time
import zlib
from collections import deque
from collections.abc import Mapping
from itertools import islice
from pathlib import Path
from typing import Any
from uuid import UUID, uuid4

from .camera import Cameras


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


class Hub:
    """Producer never waits for browser or disk. Telemetry coalesces; events have a ring."""

    def __init__(self, event_limit=2048):
        self.cameras = Cameras()
        self.lock = threading.Lock()
        self.latest = {}
        self.events = deque(maxlen=event_limit)
        self.sequence = 0
        self.evicted_through = 0
        self.selected = {}
        self.boot_id = str(uuid4())

    def put(
        self,
        kind: str,
        key: str,
        data: Any,
        *,
        critical: bool = False,
        ros_time: float | None = None,
    ) -> dict:
        with self.lock:
            self.sequence += 1
            row = {
                "seq": self.sequence,
                "kind": kind,
                "key": key,
                "data": data,
                "received_at": time.time(),
                "received_monotonic": time.monotonic(),
                "ros_time": ros_time,
            }
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
                "sequence": self.sequence,
                "gap": bool(since < self.evicted_through),
                "rows": sorted(
                    [*self.latest.values(), *events], key=lambda r: r["seq"]
                ),
            }

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


class Recorder:
    """Owned by HTTP worker. Never touches MissionJournal or deletes evidence."""

    def __init__(self, directory, session_limit=100 * 1024**2, total_limit=1024**3):
        self.directory = Path(directory).expanduser()
        self.session_limit, self.total_limit = session_limit, total_limit
        self.session_id = str(uuid4())
        self.path = self.directory / f"{self.session_id}.sqlite3"
        self.db = None
        self.reason = ""
        self.last = 0
        self.bytes = 0
        self.hashes = {}
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            if self.disk_bytes() >= total_limit:
                self.reason = "Total recording limit reached; new recording stopped."
                return
            self.db = sqlite3.connect(self.path)
            self.db.execute(
                "CREATE TABLE records(seq INTEGER PRIMARY KEY, received REAL, kind TEXT, key TEXT, payload BLOB)"
            )
            self.db.execute("CREATE INDEX records_time ON records(received)")
            self.db.execute("CREATE TABLE metadata(key TEXT PRIMARY KEY, value TEXT)")
            self.db.execute(
                "INSERT INTO metadata VALUES (?, ?)", ("schema_version", "2")
            )
            self.db.commit()
        except (OSError, sqlite3.Error) as exc:
            self.reason = f"Recording unavailable: {exc}"
            if self.db:
                self.db.close()
                self.db = None

    def disk_bytes(self):
        return sum(p.stat().st_size for p in self.directory.glob("*.sqlite3*"))

    def append(self, snapshot: dict) -> None:
        if self.reason or self.db is None:
            return
        rows = [r for r in snapshot["rows"] if r["seq"] > self.last]
        if snapshot.get("gap"):
            self.db.execute(
                "INSERT OR REPLACE INTO metadata VALUES (?, ?)",
                ("gap", "Event ring overflow; recording is incomplete"),
            )
        for row in rows:
            payload = zlib.compress(
                json.dumps(row, ensure_ascii=False, allow_nan=False).encode(), level=3
            )
            digest = hashlib.sha256(
                json.dumps(row["data"], sort_keys=True).encode()
            ).hexdigest()
            key = (row["kind"], row["key"])
            if row["kind"] == "layer" and self.hashes.get(key) == digest:
                continue
            size = len(payload)
            if (
                max(self.path.stat().st_size, self.bytes) + size + 16384
                > self.session_limit
                or self.disk_bytes() + size + 16384 > self.total_limit
            ):
                self.reason = "Recording size limit reached; evidence retained."
                self.db.execute(
                    "INSERT OR REPLACE INTO metadata VALUES (?, ?)",
                    ("stopped_reason", self.reason),
                )
                break
            self.db.execute(
                "INSERT OR IGNORE INTO records VALUES (?, ?, ?, ?, ?)",
                (row["seq"], row["received_at"], row["kind"], row["key"], payload),
            )
            self.bytes += size
            self.hashes[key] = digest
        self.db.commit()
        self.last = snapshot["sequence"]

    def close(self):
        if self.db:
            self.db.close()

    def sessions(self) -> list[dict]:
        result = []
        for path in sorted(
            self.directory.glob("*.sqlite3"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        ):
            with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as db:
                count, start, end = db.execute(
                    "SELECT count(*),min(received),max(received) FROM records"
                ).fetchone()
                runtime_count, runtime_start = db.execute(
                    "SELECT count(*),min(received) FROM records WHERE kind='runtime'"
                ).fetchone()
                meta = dict(db.execute("SELECT key,value FROM metadata"))
            result.append(
                {
                    "id": path.stem,
                    "count": count,
                    "runtime_count": runtime_count,
                    "runtime_start": runtime_start,
                    "start": start,
                    "end": end,
                    "bytes": path.stat().st_size,
                    "metadata": meta,
                }
            )
        return result

    def resolve(self, session: str) -> Path:
        # UUID names only; no arbitrary filesystem paths from browser.
        if str(UUID(session)) != session:
            raise ValueError("Invalid session ID")
        path = self.directory / f"{session}.sqlite3"
        if not path.is_file():
            raise FileNotFoundError(session)
        return path

    def replay(self, session: str, at: float) -> dict:
        with sqlite3.connect(f"file:{self.resolve(session)}?mode=ro", uri=True) as db:
            latest = db.execute(
                """SELECT payload FROM records WHERE seq IN (
                SELECT max(seq) FROM records WHERE received <= ? AND kind NOT IN ('event','log','result') GROUP BY kind,key)""",
                (at,),
            )
            rows = [decode_record(r[0]) for r in latest]
            rows += [
                decode_record(r[0])
                for r in db.execute(
                    """SELECT payload FROM records
                WHERE received <= ? AND kind IN ('event','log','result') ORDER BY seq DESC LIMIT 500""",
                    (at,),
                )
            ]
            meta = dict(db.execute("SELECT key,value FROM metadata"))
        return {
            "rows": sorted(rows, key=lambda r: r["seq"]),
            "replay": True,
            "metadata": meta,
        }


def decode_record(payload: bytes | str) -> dict:
    """Version 1 was plain JSON; version 2 stores zlib-compressed UTF-8 JSON."""
    return json.loads(
        zlib.decompress(payload) if isinstance(payload, bytes) else payload
    )
