from dataclasses import replace
from types import SimpleNamespace

import numpy as np
from cleany_interfaces.srv import ObserveObjectReference as Service

from cleany_perception.core.reference_observation import ReferenceObservationConfig
from cleany_perception.reference_service import ReferenceService
from cleany_perception.rgbd_snapshot import SynchronizedRgbdMessages
from cleany_perception.snapshot_cache import CachedDetectionSnapshot, DetectionSnapshotCache
from test_inspection_node import _sensor_messages


def make_service(scene, clock=lambda: 0.):
    cache = DetectionSnapshotCache(maximum_entries=1)
    cached = CachedDetectionSnapshot(
        snapshot=scene['snapshot'], detections=(scene['detection'],),
        detection_distances_m=(None,), capture_transform=scene['transform'],
        color_frame=scene['snapshot'].source_frame)
    cache.put('source', cached)
    events = []
    def new_messages(sequence, timeout):
        events.append('capture')
        return SynchronizedRgbdMessages(3, 2_000_000_000, *_sensor_messages(scene, 2_000_000_000))
    def transform(snapshot):
        assert snapshot.stamp_ns == 2_000_000_000
        events.append('tf')
        return scene['transform']
    def track(source_rgb, detection, current_rgb):
        events.append('reference_mask')
        assert detection == scene['detection']
        return scene['mask']
    service = ReferenceService(cache=cache,
        buffer=SimpleNamespace(sequence=2, wait_for_new=new_messages),
        tracker=SimpleNamespace(track=track), lookup_transform=transform,
        target_frame='base_link', depth_scale=.001, timeout_seconds=1.,
        ttl_seconds=120., config=ReferenceObservationConfig(), clock=clock)
    pin = service.execute(Service.Request(operation=Service.Request.PIN,
        source_snapshot_id='source', source_object_id=1), Service.Response())
    assert pin.success
    return service, cache, pin, events


def test_pinned_seed_survives_cache_eviction_and_current_capture_precedes_inference(synthetic_scene):
    handler, cache, pin, events = make_service(synthetic_scene)
    cache.put('replacement', replace(cache.get('source')))
    assert cache.get('source') is None
    result = handler.execute(Service.Request(operation=Service.Request.OBSERVE,
        reference_id=pin.reference_id, after_stamp_ns=1_800_000_000), Service.Response())
    assert result.success, result.message
    assert events == ['capture', 'tf', 'reference_mask']
    assert result.source_label == synthetic_scene['detection'].label
    assert result.source_confidence == synthetic_scene['detection'].confidence
    assert result.source_snapshot_id == 'source'
    assert result.source_capture_stamp_ns == synthetic_scene['snapshot'].stamp_ns
    assert result.header.stamp.sec == 2
    assert result.header.frame_id == 'base_link'
    assert result.mask.header.frame_id == synthetic_scene['snapshot'].source_frame
    assert result.observed_cloud.width == result.valid_depth_points == 2000
    assert not np.shares_memory(handler._reference.rgb, synthetic_scene['snapshot'].rgb)


def test_stale_capture_is_rejected_before_inference(synthetic_scene):
    handler, _, pin, events = make_service(synthetic_scene)
    result = handler.execute(Service.Request(operation=Service.Request.OBSERVE,
        reference_id=pin.reference_id, after_stamp_ns=2_000_000_000), Service.Response())
    assert not result.success and result.error_code == result.ERROR_RGBD_TIMEOUT
    # The first frame is stale; this fixture then repeats the same sequence.
    assert events == ['capture', 'capture']


def test_in_flight_frame_is_skipped_until_capture_is_newer(synthetic_scene):
    handler, _, pin, events = make_service(synthetic_scene)
    calls = []
    stamps = (1_900_000_000, 2_000_000_000)

    def new_messages(sequence, timeout):
        calls.append((sequence, timeout))
        value = stamps[len(calls)-1]
        return SynchronizedRgbdMessages(sequence+1, value, *_sensor_messages(synthetic_scene, value))

    handler._buffer.wait_for_new = new_messages
    result = handler.execute(Service.Request(operation=Service.Request.OBSERVE,
        reference_id=pin.reference_id, after_stamp_ns=1_950_000_000), Service.Response())
    assert result.success, result.message
    assert result.header.stamp.sec == 2
    assert [sequence for sequence, _ in calls] == [2, 3]
    assert 0 < calls[1][1] <= calls[0][1] <= 1.
    assert events == ['tf', 'reference_mask']


def test_expired_reference_cannot_be_used(synthetic_scene):
    now = [0.]
    handler, _, pin, events = make_service(synthetic_scene, lambda: now[0])
    now[0] = 120.
    result = handler.execute(Service.Request(operation=Service.Request.OBSERVE,
        reference_id=pin.reference_id), Service.Response())
    assert not result.success and result.error_code == result.ERROR_REFERENCE
    assert not events


def test_clear_reference_and_unknown_identity_fail_closed(synthetic_scene):
    handler, _, pin, _ = make_service(synthetic_scene)
    result = handler.execute(Service.Request(operation=Service.Request.CLEAR,
        reference_id=pin.reference_id), Service.Response())
    assert result.success
    result = handler.execute(Service.Request(operation=Service.Request.OBSERVE,
        reference_id=pin.reference_id), Service.Response())
    assert not result.success and result.error_code == result.ERROR_REFERENCE


def test_pin_does_not_keep_unused_source_depth_after_cache_eviction(synthetic_scene):
    import weakref
    source = replace(synthetic_scene['snapshot'], depth_m=synthetic_scene['snapshot'].depth_m.copy())
    depth = weakref.ref(source.depth_m)
    scene = dict(synthetic_scene, snapshot=source)
    handler, cache, pin, events = make_service(scene)
    assert np.array_equal(handler._reference.rgb, source.rgb)
    assert not handler._reference.rgb.flags.writeable
    del source, scene
    cache.put('replacement', CachedDetectionSnapshot(
        snapshot=synthetic_scene['snapshot'], detections=(synthetic_scene['detection'],),
        detection_distances_m=(None,), capture_transform=synthetic_scene['transform'],
        color_frame=synthetic_scene['snapshot'].source_frame))
    # The fixture's injected closures own scene; isolate the actual seed's ownership.
    handler._buffer = None
    handler._tracker = None
    handler._lookup_transform = None
    assert depth() is None
    assert handler._reference.reference_id == pin.reference_id


def test_periodic_expiry_keeps_error_and_clear_contract_without_image_buffers(synthetic_scene):
    import weakref
    now = [0.]
    handler, _, pin, _ = make_service(synthetic_scene, lambda: now[0])
    rgb = weakref.ref(handler._reference.rgb)
    now[0] = 119.99
    assert not handler.expire() and rgb() is not None
    now[0] = 120.
    assert handler.expire() and rgb() is None
    result = handler.execute(Service.Request(operation=Service.Request.OBSERVE,
        reference_id=pin.reference_id), Service.Response())
    assert not result.success and result.error_code == result.ERROR_REFERENCE
    assert 'expired' in result.message
    cleared = handler.execute(Service.Request(operation=Service.Request.CLEAR,
        reference_id=pin.reference_id), Service.Response())
    assert cleared.success


def test_periodic_expiry_never_interrupts_an_in_flight_observation(synthetic_scene):
    import threading
    now = [0.]
    handler, _, pin, events = make_service(synthetic_scene, lambda: now[0])
    entered, release = threading.Event(), threading.Event()
    capture = handler._buffer.wait_for_new
    def blocked_capture(sequence, timeout):
        entered.set()
        assert release.wait(3.)
        return capture(sequence, timeout)
    handler._buffer.wait_for_new = blocked_capture
    results = []
    thread = threading.Thread(target=lambda: results.append(handler.execute(
        Service.Request(operation=Service.Request.OBSERVE, reference_id=pin.reference_id),
        Service.Response())))
    thread.start()
    try:
        assert entered.wait(3.)
        now[0] = 120.
        assert not handler.expire() and handler._reference is not None
    finally:
        release.set(); thread.join(timeout=3.)
    assert not thread.is_alive() and results[0].success
    assert results[0].valid_depth_points == 2000
    assert handler.expire()


def test_inspector_timer_cleans_idle_cache_and_reference(synthetic_scene):
    import threading
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from cleany_perception.inspection_node import InspectionNode
    from test_inspection_node import _Detector, _Segmenter, _Transformer, _wait_until
    now = [0.]
    node = executor = thread = None
    rclpy.init()
    try:
        node = InspectionNode(detector=_Detector(synthetic_scene['detection']),
            segmenter=_Segmenter(synthetic_scene['mask']),
            transformer=_Transformer(synthetic_scene['transform']),
            parameter_overrides=[Parameter('enable_reference_observation', value=True),
                                 Parameter('snapshot_cleanup_period_seconds', value=.02)])
        node._snapshot_cache._clock = lambda: now[0]
        node._observations.reference._clock = lambda: now[0]
        node._snapshot_cache.put('source', CachedDetectionSnapshot(
            snapshot=synthetic_scene['snapshot'], detections=(synthetic_scene['detection'],),
            detection_distances_m=(None,), capture_transform=synthetic_scene['transform'],
            color_frame=synthetic_scene['snapshot'].source_frame))
        pin = node._observations.reference.execute(Service.Request(operation=Service.Request.PIN,
            source_snapshot_id='source', source_object_id=1), Service.Response())
        assert pin.success
        now[0] = 121.
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
        assert _wait_until(lambda: not node._snapshot_cache._entries
                           and node._observations.reference._reference is None)
    finally:
        if executor:
            executor.shutdown()
        if thread:
            thread.join(timeout=3.)
        if node:
            node.destroy_node()
        rclpy.shutdown()


def test_reference_service_ros_roundtrip_and_inspection_mutual_exclusion(synthetic_scene):
    import threading
    import time

    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from cleany_perception.inspection_node import InspectionNode
    from test_inspection_node import _Detector, _Segmenter, _Transformer, _wait_until

    s = synthetic_scene
    rclpy.init()
    node = client_node = executor = thread = None
    try:
        node = InspectionNode(detector=_Detector(s['detection']), segmenter=_Segmenter(s['mask']),
            transformer=_Transformer(s['transform']),
            parameter_overrides=[Parameter('enable_reference_observation', value=True)])
        node._observations.reference._tracker = SimpleNamespace(track=lambda *args: s['mask'])
        node._snapshot_cache.put('source', CachedDetectionSnapshot(
            snapshot=s['snapshot'], detections=(s['detection'],),
            detection_distances_m=(None,), capture_transform=s['transform'],
            color_frame=s['snapshot'].source_frame))
        client_node = rclpy.create_node('reference_observation_test_client')
        client = client_node.create_client(Service, '/perception/observe_object_reference')
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.add_node(client_node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        assert client.wait_for_service(timeout_sec=3.)
        request = Service.Request(operation=Service.Request.PIN,
                                  source_snapshot_id='source', source_object_id=1)
        with node._busy_lock:
            node._busy = True
        future = client.call_async(request)
        assert _wait_until(future.done)
        assert not future.result().success and future.result().error_code == Service.Response.ERROR_BUSY
        with node._busy_lock:
            node._busy = False
        future = client.call_async(request)
        assert _wait_until(future.done)
        pin = future.result()
        assert pin.success
        future = client.call_async(Service.Request(operation=Service.Request.OBSERVE,
            reference_id=pin.reference_id, after_stamp_ns=1_800_000_000))
        assert _wait_until(lambda: node._busy)
        # Fill the exact-timestamp sensor buffer after the service starts waiting.
        time.sleep(.02)
        color, info, depth, depth_info = _sensor_messages(s, 2_000_000_000)
        node._snapshot_buffer.add_color(color)
        node._snapshot_buffer.add_color_info(info)
        node._snapshot_buffer.add_depth(depth)
        node._snapshot_buffer.add_depth_info(depth_info)
        assert _wait_until(future.done)
        result = future.result()
        assert result.success, result.message
        assert result.valid_depth_points == 2000 and result.source_label == 'box'
        assert result.header.stamp.sec == 2
        assert not node._busy
    finally:
        if executor:
            executor.shutdown()
        if thread:
            thread.join(timeout=3.)
        if client_node:
            client_node.destroy_node()
        if node:
            node.destroy_node()
        rclpy.shutdown()
