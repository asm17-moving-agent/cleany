"""Per-zone meaning is separate from the sensor session's availability."""
from dataclasses import dataclass
from enum import IntEnum
import numpy as np


class ZoneState(IntEnum):
    UNRELIABLE = 0
    RANGE = 1
    NO_RETURN = 2
    NOT_UPDATED = 3


def classify_zones(distance, sigma, status, count, valid, *, max_range_m=4., max_sigma_m=.03):
    arrays = [np.asarray(a) for a in (distance, sigma, status, count, valid)]
    if any(a.shape != (64,) for a in arrays):
        raise ValueError('expected five 64-zone arrays')
    d, s, st, n, v = arrays
    states = np.full(64, ZoneState.UNRELIABLE, dtype=np.uint8)
    # Firmware valid is deliberately narrower than the ULD status vocabulary.
    measured = (v.astype(bool) & np.isin(st, (5, 9)) & (n > 0) & np.isfinite(d)
                & (d > 0) & (d <= max_range_m) & np.isfinite(s) & (s >= 0) & (s <= max_sigma_m))
    states[measured] = ZoneState.RANGE
    # Distance/sigma slots are not measurements when no target exists.
    states[(n == 0) & (st == 255) & ~v.astype(bool)] = ZoneState.NO_RETURN
    states[st == 0] = ZoneState.NOT_UPDATED
    return states


@dataclass(frozen=True)
class Observation:
    points: np.ndarray
    states: np.ndarray

    def __post_init__(self):
        if np.shape(self.points) != (64, 3) or np.shape(self.states) != (64,):
            raise ValueError('invalid observation shape')
        if not np.isin(self.states, list(ZoneState)).all():
            raise ValueError('unknown zone state')
        if not np.isfinite(self.points[self.ranges]).all():
            raise ValueError('invalid measured point')

    @property
    def ranges(self):
        return self.states == ZoneState.RANGE

    @property
    def available(self) -> bool:
        # All unupdated/bad-quality zones are unusable; an all-NO_RETURN frame
        # is a live observation, never a free-space certificate.
        return bool(np.isin(self.states, (ZoneState.RANGE, ZoneState.NO_RETURN)).any())

    def counts(self) -> dict:
        return {state.name.lower(): int(np.count_nonzero(self.states == state)) for state in ZoneState}
