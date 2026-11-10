from dataclasses import replace

import numpy as np
import pytest

from cleany_perception.core.models import PipelineConfig
from cleany_perception.core.reconstruction_cache import ReconstructionCache


@pytest.mark.parametrize('change', ['depth', 'intrinsics', 'stamp', 'frame', 'config'])
def test_cache_invalidates_when_geometry_inputs_change(synthetic_scene, change):
    snapshot = synthetic_scene['snapshot']
    config = PipelineConfig()
    cache = ReconstructionCache()
    cache.bind(snapshot, config)
    old = cache.get_or_compute(('test',), lambda: object())
    assert cache.get_or_compute(('test',), lambda: None) is old
    if change == 'depth':
        snapshot.depth_m[0, 0] += .1
    elif change == 'intrinsics':
        snapshot = replace(snapshot, intrinsics=replace(snapshot.intrinsics, fx=401.))
    elif change == 'stamp':
        snapshot = replace(snapshot, stamp_ns=snapshot.stamp_ns + 1)
    elif change == 'frame':
        snapshot = replace(snapshot, source_frame='different_frame')
    else:
        config = replace(config, plane_distance_threshold_m=.02)
    cache.bind(snapshot, config)
    assert cache.get_or_compute(('test',), lambda: object()) is not old


def test_masks_are_exact_and_geometry_cache_is_bounded():
    cache = ReconstructionCache(maximum_entries=2)
    first = np.zeros((10, 10), dtype=bool)
    second = first.copy()
    second[5, 5] = True
    assert cache.array_key(first) != cache.array_key(second)
    for i in range(3):
        assert cache.get_or_compute((i,), lambda: i) == i
    assert len(cache._entries) == 2
    assert cache.get_or_compute((0,), lambda: 'recomputed') == 'recomputed'


def test_failed_estimate_is_not_cached():
    cache = ReconstructionCache()
    def fail():
        raise ValueError('no support plane')
    with pytest.raises(ValueError):
        cache.get_or_compute(('plane',), fail)
    assert cache.get_or_compute(('plane',), lambda: 'valid') == 'valid'
