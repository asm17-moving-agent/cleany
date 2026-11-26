"""관측을 종류별로 합치고 Backend의 저장 ACK 이후에만 대기열에서 제거합니다."""
from __future__ import annotations

import json
import logging
import math
import threading
import time
from pathlib import Path
from typing import Callable
from urllib.parse import urlparse

LOGGER = logging.getLogger(__name__)


class ObservationRelay:
    """제어 경로와 분리된 관측 전송입니다. 재연결 시 오래된 값은 폐기합니다."""

    def __init__(self, url: str, device_token_file: str, *, rate: float = 10.0,
                 input_timeout: float = 3.0, battery_timeout: float = 30.0,
                 supervisor_timeout: float = 5.0, reconnect_initial: float = 1.0,
                 reconnect_max: float = 30.0, event_capacity: int = 100,
                 connect: Callable | None = None, clock: Callable[[], float] = time.monotonic) -> None:
        if urlparse(url).scheme not in ("ws", "wss") or not urlparse(url).netloc:
            raise ValueError("url must be ws:// or wss://")
        values = (rate, input_timeout, battery_timeout, supervisor_timeout, reconnect_initial, reconnect_max)
        if not all(math.isfinite(v) and v > 0 for v in values) or reconnect_max < reconnect_initial or event_capacity < 1:
            raise ValueError("invalid relay configuration")
        self.url, self.token = url, device_token_file
        self.rate, self.timeout = rate, input_timeout
        self.battery_timeout, self.supervisor_timeout = battery_timeout, supervisor_timeout
        self.initial, self.maximum, self.capacity = reconnect_initial, reconnect_max, event_capacity
        self.clock, self.connect = clock, connect or self._connect
        self.pending: dict[str, tuple[dict, float]] = {}
        self.lock = threading.Lock()
        self.boot: str | None = None
        self.last_sequence = -1
        self.dropped_events = 0
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None

    def _connect(self):
        import websocket
        token = Path(self.token).expanduser().read_text().strip()
        if not token or any(c.isspace() for c in token):
            raise ValueError("invalid device token")
        return websocket.create_connection(self.url, timeout=2, suppress_origin=True,
                                            header={"Authorization": "Bearer " + token})

    def ttl(self, kind: str) -> float:
        return self.battery_timeout if kind == "battery" else (
            self.supervisor_timeout if kind.startswith("supervisor") or kind == "safety" else self.timeout)

    def update(self, frame: dict) -> bool:
        kind = frame["sample"]["kind"]
        if kind not in ("joints", "joint_targets", "battery", "base", "supervisor_state",
                        "supervisor_faults", "supervisor_recoveries", "supervisor_event", "safety"):
            return False
        if not math.isfinite(frame["source_age_seconds"]) or not 0 <= frame["source_age_seconds"] < self.ttl(kind):
            return False
        json.dumps(frame, allow_nan=False)
        with self.lock:
            if frame["boot_id"] != self.boot:
                self.pending.clear()
                self.last_sequence = -1
                self.boot = frame["boot_id"]
            if frame["sequence"] <= self.last_sequence:
                return False
            self.last_sequence = frame["sequence"]
            key = f"event:{frame['sequence']}" if kind == "supervisor_event" else kind
            events = [k for k in self.pending if k.startswith("event:")]
            if kind == "supervisor_event" and len(events) >= self.capacity:
                del self.pending[events[0]]
                self.dropped_events += 1
                LOGGER.warning("Observation event capacity reached; oldest event dropped")
            self.pending[key] = (frame, self.clock())
        return True

    def next_frame(self) -> tuple[str, dict] | None:
        with self.lock:
            for key, (frame, received) in sorted(self.pending.items(), key=lambda v: v[1][0]["sequence"]):
                age = frame["source_age_seconds"] + max(0.0, self.clock()-received)
                if age >= self.ttl(frame["sample"]["kind"]):
                    del self.pending[key]
                    if frame["sample"]["kind"] == "supervisor_event":
                        self.dropped_events += 1
                        LOGGER.warning("Expired observation event dropped")
                    continue
                return key, dict(frame, source_age_seconds=age)
        return None

    def acknowledge(self, key: str, frame: dict) -> None:
        with self.lock:
            existing = self.pending.get(key)
            if existing and existing[0]["boot_id"] == frame["boot_id"] and existing[0]["sequence"] == frame["sequence"]:
                del self.pending[key]

    def start(self) -> None:
        self.thread = threading.Thread(target=self._run, name="robot-observations", daemon=True)
        self.thread.start()

    def _run(self) -> None:
        socket = None
        backoff = self.initial
        try:
            while not self.stop_event.is_set():
                item = self.next_frame()
                if item is None:
                    self.stop_event.wait(1/self.rate)
                    continue
                try:
                    if socket is None:
                        socket = self.connect()
                    key, frame = item
                    socket.send(json.dumps(frame, allow_nan=False))
                    ack = json.loads(socket.recv())
                    if ack.get("sequence") != frame["sequence"] or not isinstance(ack.get("accepted"), bool):
                        raise ValueError("invalid observation ACK")
                    self.acknowledge(key, frame)
                    backoff = self.initial
                except Exception as error:  # 외부 연결 실패는 관측 전달만 중단합니다.
                    LOGGER.warning("Observation delivery failed (%s)", type(error).__name__)
                    if socket is not None:
                        socket.close()
                        socket = None
                    self.stop_event.wait(backoff)
                    backoff = min(self.maximum, backoff*2)
        finally:
            if socket is not None:
                socket.close()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=5)
