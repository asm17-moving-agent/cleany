"""Continuous encoder stability windows for measurement capture."""
import numpy as np


class StillWindow:
    """Require a continuous window with small absolute encoder variation."""
    def __init__(self, hold_seconds: float, max_span_ticks: int, max_gap_seconds: float):
        self.hold = hold_seconds
        self.span = max_span_ticks
        self.gap = max_gap_seconds
        self.samples: list[dict] = []

    def update(self, record: dict | None) -> bool:
        if record is None:
            self.samples.clear()
            return False
        now = record['host_monotonic_end_ns'] / 1e9
        if self.samples:
            previous = self.samples[-1]['host_monotonic_end_ns'] / 1e9
            if not 0 < now - previous <= self.gap:
                self.samples.clear()
        self.samples.append(record)
        while len(self.samples) > 2 and now - self.samples[1]['host_monotonic_end_ns']/1e9 >= self.hold:
            self.samples.pop(0)
        values = np.array([[j['raw_tick'] for j in r['joints'].values()] for r in self.samples])
        if np.any(np.ptp(values, axis=0) > self.span):
            self.samples = [record]
            return False
        return now - self.samples[0]['host_monotonic_end_ns']/1e9 >= self.hold
