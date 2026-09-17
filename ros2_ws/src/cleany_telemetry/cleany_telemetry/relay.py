"""Small, dependency-free pose relay core and websocket worker."""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional, Protocol
from urllib.parse import urlparse

LOGGER = logging.getLogger(__name__)


class Transport(Protocol):
    def send(self, message: str) -> None: ...
    def close(self) -> None: ...


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    received_at: float
    sim_stamp: Optional[float]


class PoseCache:
    """Latest-only cache. ``received_at`` is always a monotonic timestamp."""

    def __init__(self, input_timeout: float) -> None:
        self.input_timeout = input_timeout
        self._pose: Optional[Pose] = None
        self._last_sim_stamp: Optional[float] = None
        self._lock = threading.Lock()

    def update(self, x: float, y: float, received_at: float, sim_stamp: Optional[float]) -> bool:
        if not (math.isfinite(x) and math.isfinite(y)):
            return False
        with self._lock:
            # A clock reset must not leak the old pose.  The reset message itself
            # is discarded; the next message is the first fresh sample.
            if (
                sim_stamp is not None
                and self._last_sim_stamp is not None
                and sim_stamp < self._last_sim_stamp
            ):
                self._pose = None
                self._last_sim_stamp = None
                return False
            if (
                sim_stamp is not None
                and self._last_sim_stamp is not None
                and sim_stamp == self._last_sim_stamp
            ):
                # Repeated messages from a paused simulator are not fresh input.
                return False
            self._last_sim_stamp = sim_stamp
            self._pose = Pose(x, y, received_at, sim_stamp)
            return True

    def fresh(self, now: float) -> Optional[Pose]:
        with self._lock:
            if self._pose is None or now - self._pose.received_at > self.input_timeout:
                return None
            return self._pose

    def invalidate(self) -> None:
        with self._lock:
            self._pose = None


class PoseRelay:
    """Wall-clock worker; callbacks only update ``PoseCache``."""

    def __init__(
        self,
        url: str,
        rate: float = 5.0,
        input_timeout: float = 1.5,
        reconnect_initial: float = 1.0,
        reconnect_max: float = 30.0,
        transport_factory: Optional[Callable[[str], Transport]] = None,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        parsed = urlparse(url)
        if parsed.scheme not in ("ws", "wss") or not parsed.netloc:
            raise ValueError("url must be a ws:// or wss:// URL")
        if not all(math.isfinite(value) for value in (
            rate, input_timeout, reconnect_initial, reconnect_max,
        )):
            raise ValueError("relay timing parameters must be finite")
        if rate <= 0 or input_timeout <= 0 or reconnect_initial <= 0 or reconnect_max < reconnect_initial:
            raise ValueError("invalid relay timing parameters")
        self.url, self.period = url, 1.0 / rate
        self.cache = PoseCache(input_timeout)
        self._connect = transport_factory or _websocket_transport
        self._clock = monotonic
        self._initial, self._maximum = reconnect_initial, reconnect_max
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def update_pose(self, x: float, y: float, sim_stamp: Optional[float] = None) -> bool:
        return self.cache.update(x, y, self._clock(), sim_stamp)

    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="pose-websocket", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    def _run(self) -> None:
        transport: Optional[Transport] = None
        backoff = self._initial
        next_send = self._clock()
        next_connect = next_send
        while not self._stop.is_set():
            if transport is None:
                if self._stop.wait(max(0.0, next_connect - self._clock())):
                    break
                try:
                    transport = self._connect(self.url)
                    backoff = self._initial
                    next_send = self._clock()
                except Exception as error:
                    LOGGER.warning(
                        "Pose WebSocket connect failed (%s); retrying in %.1fs",
                        type(error).__name__, backoff,
                    )
                    self._stop.wait(backoff)
                    next_connect = self._clock()
                    backoff = min(backoff * 2.0, self._maximum)
                    continue
            now = self._clock()
            if now >= next_send:
                pose = self.cache.fresh(now)
                if pose is not None:
                    try:
                        transport.send(json.dumps({"x": pose.x, "y": pose.y}, separators=(",", ":")))
                    except Exception as error:
                        LOGGER.warning(
                            "Pose WebSocket send failed (%s); reconnecting",
                            type(error).__name__,
                        )
                        try:
                            transport.close()
                        except Exception:
                            pass
                        transport = None
                        next_connect = self._clock() + backoff
                        backoff = min(backoff * 2.0, self._maximum)
                        continue
                next_send = now + self.period
            self._stop.wait(min(max(next_send - self._clock(), 0.001), 0.1))
        if transport is not None:
            try:
                transport.close()
            except Exception:
                pass


def _websocket_transport(url: str) -> Transport:
    import websocket  # type: ignore[import-not-found]

    return _WebsocketTransport(
        websocket.create_connection(url, timeout=5, suppress_origin=True), websocket
    )


class _WebsocketTransport:
    """Read frames so websocket-client can answer ping and observe close."""

    def __init__(self, socket: object, module: object) -> None:
        self._socket = socket
        self._module = module
        self._closed = threading.Event()
        self._reader = threading.Thread(
            target=self._read, name="pose-websocket-reader", daemon=True
        )
        self._reader.start()

    def _read(self) -> None:
        try:
            while not self._closed.is_set():
                try:
                    self._socket.recv()  # type: ignore[attr-defined]
                except self._module.WebSocketTimeoutException:
                    continue
                except Exception:
                    break
        finally:
            self._closed.set()

    def send(self, message: str) -> None:
        if self._closed.is_set():
            raise ConnectionError("websocket is closed")
        self._socket.send(message)  # type: ignore[attr-defined]

    def close(self) -> None:
        self._closed.set()
        try:
            self._socket.close()  # type: ignore[attr-defined]
        finally:
            self._reader.join(timeout=1.0)
