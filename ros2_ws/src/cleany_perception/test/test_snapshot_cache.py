from cleany_perception.snapshot_cache import (
    CachedDetectionSnapshot,
    DetectionSnapshotCache,
)


def _cached_scene(synthetic_scene):
    return CachedDetectionSnapshot(
        snapshot=synthetic_scene['snapshot'],
        detections=(synthetic_scene['detection'],),
        detection_distances_m=(None,),
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


def test_periodic_expiry_preserves_live_request_and_does_not_refresh_ttl(synthetic_scene):
    import weakref
    from dataclasses import replace
    now = [0.]
    cache = DetectionSnapshotCache(ttl_seconds=2., clock=lambda: now[0])
    cached = replace(_cached_scene(synthetic_scene))
    held = weakref.ref(cached)
    cache.put('source', cached)
    request_snapshot = cache.get('source')
    del cached
    now[0] = 1.99
    assert cache.expire() == 0
    now[0] = 2.
    assert cache.expire() == 1
    assert not cache._entries and held() is request_snapshot
    assert request_snapshot.snapshot.stamp_ns == synthetic_scene['snapshot'].stamp_ns
    del request_snapshot
    assert held() is None


def test_read_only_service_preserves_capture_time_and_does_not_extend_ttl(synthetic_scene):
    from cleany_interfaces.srv import GetSceneSnapshot
    from cleany_perception.inspection_node import InspectionNode
    now = [10.0]
    node = object.__new__(InspectionNode)
    node._snapshot_cache = DetectionSnapshotCache(ttl_seconds=2., clock=lambda: now[0])
    cached = _cached_scene(synthetic_scene)
    node._snapshot_cache.put('approved', cached)
    request = GetSceneSnapshot.Request(snapshot_id='approved')
    for wall in (10.5, 11.5):
        now[0] = wall
        response = node._get_scene_snapshot(request, GetSceneSnapshot.Response())
        stamp = response.detections.header.stamp
        assert response.found and response.detections.snapshot_id == 'approved'
        assert stamp.sec*10**9+stamp.nanosec == cached.snapshot.stamp_ns
    now[0] = 12.
    assert not node._get_scene_snapshot(request, GetSceneSnapshot.Response()).found
