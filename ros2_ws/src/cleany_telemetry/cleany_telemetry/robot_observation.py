"""ROS 시각과 관절별 수신 시각을 보존하는 읽기 전용 관측 버퍼입니다."""
from __future__ import annotations

import math
import uuid
from collections import deque
from typing import Any


class ObservationBuffer:
    """종류별 최신 값과 제한된 이벤트를 보관하고 시간 역행 시 세대를 바꿉니다."""

    def __init__(self, provenance: str, input_timeout: float = 3.0,
                 battery_timeout: float = 30.0, supervisor_timeout: float = 5.0,
                 future_tolerance: float = 0.1, event_capacity: int = 100) -> None:
        if provenance not in ("real", "sim", "mock"):
            raise ValueError("provenance must explicitly be real, sim, or mock")
        if not all(math.isfinite(v) and v > 0 for v in (
                input_timeout, battery_timeout, supervisor_timeout, future_tolerance)):
            raise ValueError("timing parameters must be finite and positive")
        if event_capacity < 1:
            raise ValueError("event capacity must be positive")
        self.provenance = provenance
        self.timeout, self.battery_timeout = input_timeout, battery_timeout
        self.supervisor_timeout, self.future_tolerance = supervisor_timeout, future_tolerance
        self.boot = str(uuid.uuid4())
        self.sequence = 0
        self.now = 0.0
        self.latest: dict[str, tuple[str, float, dict[str, Any]]] = {}
        self.events: deque[tuple[str, float, dict[str, Any]]] = deque(maxlen=event_capacity)
        self.stamps: dict[str, float] = {}
        self.joints: dict[str, dict[str, tuple[float, float]]] = {}
        self.dropped_events = 0

    def ttl(self, kind: str) -> float:
        return self.battery_timeout if kind == "battery" else (
            self.supervisor_timeout if kind.startswith("supervisor") or kind == "safety"
            else self.timeout)

    def accept(self, topic: str, stamp: float, now: float, sample: dict[str, Any]) -> bool:
        if not all(math.isfinite(v) and v > 0 for v in (stamp, now)):
            return False
        if now < self.now:
            self.boot = str(uuid.uuid4())
            self.latest.clear()
            self.events.clear()
            self.stamps.clear()
            self.joints.clear()
        self.now = now
        kind = sample["kind"]
        if stamp > now + self.future_tolerance or now - stamp >= self.ttl(kind):
            return False
        if kind in ("joints", "joint_targets"):
            names, positions = sample["names"], sample["positions"]
            if (not names or len(names) != len(positions) or len(names) > 128
                    or len(set(names)) != len(names)
                    or not all(isinstance(n, str) and 0 < len(n) <= 128 for n in names)
                    or not all(math.isfinite(v) for v in positions)):
                return False
            cache = self.joints.setdefault(kind, {})
            changed = False
            for name, value in zip(names, positions):
                if name not in cache or stamp > cache[name][1]:
                    cache[name] = (float(value), stamp)
                    changed = True
            cache = {name: value for name, value in cache.items() if now-value[1] < self.timeout}
            self.joints[kind] = cache
            if not changed or not cache:
                return False
            newest = max(value[1] for value in cache.values())
            sample = dict(kind=kind, names=list(cache), positions=[v for v, _ in cache.values()],
                          ages_seconds=[max(0.0, newest-s) for _, s in cache.values()])
            stamp = newest
        elif kind != "supervisor_event" and stamp <= self.stamps.get(kind, 0):
            return False
        self.stamps[kind] = stamp
        if kind == "supervisor_event":
            if len(self.events) == self.events.maxlen:
                self.dropped_events += 1
            self.events.append((topic, stamp, sample))
        else:
            self.latest[kind] = (topic, stamp, sample)
        return True

    def drain(self, now: float) -> list[dict[str, Any]]:
        if now < self.now:
            self.latest.clear()
            self.events.clear()
            return []
        values = list(self.events) + list(self.latest.values())
        self.events.clear()
        self.latest.clear()
        output = []
        for topic, stamp, sample in values:
            age = max(0.0, now-stamp)
            if age >= self.ttl(sample["kind"]):
                if sample["kind"] == "supervisor_event":
                    self.dropped_events += 1
                continue
            self.sequence += 1
            output.append(dict(schema_version=1, boot_id=self.boot, sequence=self.sequence,
                               source_topic=topic, source_stamp=stamp, source_age_seconds=age,
                               provenance=self.provenance, sample=sample))
        return output


def battery_sample(percentage: float, voltage: float, status: int) -> dict[str, Any]:
    return dict(kind="battery", percentage=(percentage*100 if math.isfinite(percentage)
                and 0 <= percentage <= 1 else None),
                voltage=voltage if math.isfinite(voltage) and voltage >= 0 else None,
                charging={1: "CHARGING", 2: "DISCHARGING", 3: "NOT_CHARGING", 4: "FULL"}
                .get(status, "UNKNOWN"))
