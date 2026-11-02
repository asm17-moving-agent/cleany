"""Freshness of mapping heartbeats, independent of HTTP connectivity."""

import math
import time
from collections.abc import Callable


class MappingStatusCache:
    def __init__(
        self, timeout_s: float = 1.0, now: Callable[[], float] = time.monotonic
    ):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("mapping status timeout must be finite and positive")
        self.timeout_s, self.now = timeout_s, now
        self.latest: dict | None = None
        self.received_at: float | None = None

    def observe(self, status: dict) -> None:
        self.latest = dict(status)
        self.received_at = self.now()

    def status(self) -> dict:
        age = (
            None
            if self.received_at is None
            else max(0.0, self.now() - self.received_at)
        )
        connected = age is not None and age <= self.timeout_s
        result = dict(self.latest or {})
        result.update(
            connected=connected,
            status_age_ms=None if age is None else round(age * 1000, 1),
        )
        if not connected:
            result.update(
                last_phase=result.get("phase"),
                phase="unavailable",
                message="매핑 관리자 상태 수신 지연"
                if self.latest
                else "매핑 관리자를 기다립니다.",
            )
        return result
