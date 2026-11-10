"""Bounded, exact-input geometry reuse owned by one RGB-D snapshot."""
from __future__ import annotations

from collections import OrderedDict
from hashlib import blake2b
from typing import Callable, TypeVar, cast

import numpy as np

from .models import PipelineConfig, RgbdSnapshot

T = TypeVar('T')


class ReconstructionCache:
    def __init__(self, maximum_entries: int = 128) -> None:
        if maximum_entries < 1:
            raise ValueError('Geometry cache capacity must be positive')
        self.maximum_entries = maximum_entries
        self._signature: tuple | None = None
        self._entries: OrderedDict[tuple, object] = OrderedDict()
        self.hits = 0
        self.misses = 0

    def bind(self, snapshot: RgbdSnapshot, config: PipelineConfig) -> None:
        # Arrays are mutable despite the frozen snapshot dataclass. Include the
        # actual depth bytes, calibration and settings, not just its timestamp.
        signature = (snapshot.stamp_ns, snapshot.source_frame, snapshot.intrinsics,
                     repr(config), self.array_key(snapshot.depth_m))
        if signature != self._signature:
            self._entries.clear()
            self._signature = signature

    @staticmethod
    def array_key(value: np.ndarray) -> tuple:
        array = np.asarray(value)
        data = np.packbits(array).tobytes() if array.dtype == np.bool_ else array.tobytes()
        return array.shape, array.dtype.str, blake2b(data, digest_size=16).digest()

    def get_or_compute(self, key: tuple, compute: Callable[[], T]) -> T:
        if key in self._entries:
            self.hits += 1
            self._entries.move_to_end(key)
            return cast(T, self._entries[key])
        self.misses += 1
        value = compute()  # Failed estimates are never cached as successes.
        self._entries[key] = value
        while len(self._entries) > self.maximum_entries:
            self._entries.popitem(last=False)
        return value
