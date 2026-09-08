"""Single-target, latest-frame worker. No ROS or model imports in this module."""
from dataclasses import dataclass
import threading
import time
from typing import Any, Callable

from cleany_perception.core.models import MaskArray, RgbArray
from cleany_perception.core.ports import StreamingSessionPort


@dataclass(frozen=True)
class TrackingFrame:
    stamp_ns: int
    rgb: RgbArray
    metadata: Any = None


@dataclass(frozen=True)
class TrackingResult:
    frame: TrackingFrame
    mask: MaskArray
    inference_seconds: float
    processed_frames: int


class ContinuousTracking:
    """No input FIFO: next_frame selects the newest pair after each inference."""

    def __init__(self, start: Callable[[], StreamingSessionPort],
                 next_frame: Callable[[int], TrackingFrame], seed_stamp_ns: int,
                 on_result: Callable[[TrackingResult], None] = lambda result: None,
                 on_error: Callable[[Exception], None] = lambda error: None):
        self._start, self._next_frame, self._on_result = start, next_frame, on_result
        self._seed_stamp = seed_stamp_ns
        self._on_error = on_error
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._result: TrackingResult | None = None
        self._error: Exception | None = None
        self._thread = threading.Thread(target=self._run, name='wrist-sam2-tracking', daemon=True)
        self._thread.start()

    def _run(self) -> None:
        session = None
        try:
            session = self._start()
            stamp, count = self._seed_stamp, 0
            while not self._stop.is_set():
                frame = self._next_frame(stamp)
                if self._stop.is_set():
                    break
                if frame.stamp_ns <= stamp:
                    raise ValueError('Streaming frame timestamp did not advance')
                begin = time.monotonic()
                mask = session.track(frame.rgb)
                stamp, count = frame.stamp_ns, count + 1
                result = TrackingResult(frame, mask, time.monotonic()-begin, count)
                with self._condition:
                    if self._stop.is_set():
                        break
                    self._result = result
                    self._condition.notify_all()
                self._on_result(result)
        except Exception as error:
            with self._condition:
                self._error = error
                self._condition.notify_all()
            if not self._stop.is_set():
                self._on_error(error)
        finally:
            if session is not None:
                session.close()

    def wait_after(self, stamp_ns: int, *, timeout_seconds: float,
                   now_ns: Callable[[], int], maximum_age_seconds: float) -> TrackingResult:
        deadline = time.monotonic() + timeout_seconds
        with self._condition:
            while not self._stop.is_set():
                if self._error is not None:
                    raise ValueError(f'Continuous wrist tracking failed: {self._error}') from self._error
                result = self._result
                if result is not None:
                    age = (now_ns() - result.frame.stamp_ns) / 1e9
                    if result.frame.stamp_ns > stamp_ns and 0 <= age <= maximum_age_seconds:
                        return result
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ValueError('No fresh post-request wrist tracking result before deadline')
                self._condition.wait(timeout=min(remaining, .05))
        raise ValueError('Wrist tracking reference was cleared')

    def close(self, *, timeout_seconds: float = 0.) -> None:
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        if timeout_seconds > 0:
            self._thread.join(timeout_seconds)
