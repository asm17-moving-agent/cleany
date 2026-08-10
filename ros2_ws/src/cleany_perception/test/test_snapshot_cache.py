from cleany_perception.snapshot_cache import (
    CachedDetectionSnapshot,
    DetectionSnapshotCache,
)


def _cached_scene(synthetic_scene):
    return CachedDetectionSnapshot(
        snapshot=synthetic_scene['snapshot'],
        detections=(synthetic_scene['detection'],),
        capture_transform=synthetic_scene['transform'],
        color_frame='rgb_optical_frame',
    )


def test_snapshot_cache_expires_entries(synthetic_scene):
    now = [10.0]
    cache = DetectionSnapshotCache(
        ttl_seconds=2.0,
        clock=lambda: now[0],
    )
    cached = _cached_scene(synthetic_scene)
    cache.put('first', cached)

    assert cache.get('first') is cached
    now[0] = 12.0
    assert cache.get('first') is None
    assert len(cache) == 0


def test_snapshot_cache_evicts_oldest_entry(synthetic_scene):
    cache = DetectionSnapshotCache(maximum_entries=2)
    cached = _cached_scene(synthetic_scene)

    cache.put('first', cached)
    cache.put('second', cached)
    cache.put('third', cached)

    assert cache.get('first') is None
    assert cache.get('second') is cached
    assert cache.get('third') is cached
