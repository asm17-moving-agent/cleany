"""Reuse stable *acquisition* windows, never repeated broadcaster timestamps."""
from collections import deque
import math
import threading
from collections.abc import Mapping, Sequence


class JointStabilityHistory:
    def __init__(self, maximum_samples: int = 4096) -> None:
        self._samples: deque = deque(maxlen=maximum_samples)
        self._generation: int | None = None
        self._lock = threading.RLock()

    def clear(self) -> None:
        with self._lock:
            self._samples.clear()

    def observe(self, stamp_ns: int, positions: Mapping[str, float], generation: int) -> None:
        with self._lock:
            if (stamp_ns <= 0 or not positions
                    or any(not math.isfinite(q) for q in positions.values())):
                self._samples.clear()
                return
            if generation != self._generation:
                self._samples.clear()
                self._generation = generation
            if self._samples:
                last_stamp, last_positions = self._samples[-1]
                if stamp_ns == last_stamp and positions == last_positions:
                    return
                if stamp_ns <= last_stamp or positions.keys() != last_positions.keys():
                    self._samples.clear()
            self._samples.append((stamp_ns, dict(positions)))

    def stable(self, names: Sequence[str], positions: Mapping[str, float], *,
               now_ns: int, duration_sec: float, maximum_age_sec: float,
               velocity_rad_s: float) -> bool:
        with self._lock:
            if not self._samples or not names:
                return False
            latest = self._samples[-1][0]
            if not 0 <= now_ns-latest <= maximum_age_sec*1e9:
                return False
            window = []
            for sample in reversed(self._samples):
                window.append(sample)
                if latest-sample[0] >= duration_sec*1e9:
                    break
            if len(window) < 2 or latest-window[-1][0] < duration_sec*1e9:
                return False
            if any(a[0]-b[0] > maximum_age_sec*1e9 for a, b in zip(window, window[1:])):
                return False
            span_limit = velocity_rad_s * ((latest-window[-1][0])/1e9)
            for name in names:
                values = [q.get(name, math.nan) for _, q in window]
                values.append(positions.get(name, math.nan))
                if (not all(math.isfinite(q) for q in values)
                        or max(values)-min(values) > span_limit):
                    return False
            return True

    def unchanged_since(self, positions: Mapping[str, float], *, now_ns: int,
                        duration_sec: float, maximum_age_sec: float) -> int | None:
        """Return the start of an exact, fresh full-robot acquisition window.

        This is a sensor capture barrier, so every joint must match exactly.
        A velocity-tolerant settling window cannot authorize an older scene.
        """
        with self._lock:
            if (not self._samples or not positions
                    or not all(math.isfinite(value) and value > 0
                               for value in (duration_sec, maximum_age_sec))
                    or any(not math.isfinite(q) for q in positions.values())):
                return None
            latest, latest_positions = self._samples[-1]
            if (not 0 <= now_ns-latest <= maximum_age_sec*1e9
                    or latest_positions != positions):
                return None
            earliest = latest
            count = 0
            for stamp, sample_positions in reversed(self._samples):
                if sample_positions != positions or earliest-stamp > maximum_age_sec*1e9:
                    break
                earliest = stamp
                count += 1
            if count < 2 or latest-earliest < duration_sec*1e9:
                return None
            return earliest
