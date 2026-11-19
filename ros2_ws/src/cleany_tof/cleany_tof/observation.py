"""zone별 의미와 센서 session의 가용성은 별개입니다."""
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
    # 펌웨어 valid는 의도적으로 ULD 상태 종류보다 좁게 정의합니다.
    measured = (v.astype(bool) & np.isin(st, (5, 9)) & (n > 0) & np.isfinite(d)
                & (d > 0) & (d <= max_range_m) & np.isfinite(s) & (s >= 0) & (s <= max_sigma_m))
    states[measured] = ZoneState.RANGE
    # 타깃이 없으면 distance/sigma 칸은 측정값이 아닙니다.
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
        # 미갱신 또는 저품질 zone은 모두 사용할 수 없습니다. 전체 NO_RETURN 프레임은
        # 살아 있는 관측이지만 빈 공간 검증은 아닙니다.
        return bool(np.isin(self.states, (ZoneState.RANGE, ZoneState.NO_RETURN)).any())

    def counts(self) -> dict:
        return {state.name.lower(): int(np.count_nonzero(self.states == state)) for state in ZoneState}
