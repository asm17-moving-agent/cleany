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
    cached = CachedDetectionSnapshot(scene['snapshot'], (scene['detection'],), (.8,),
                                     scene['transform'], scene['snapshot'].source_frame)
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
        events.append('sam2')
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
    assert events == ['capture', 'tf', 'sam2']
    assert result.source_label == synthetic_scene['detection'].label
    assert result.source_confidence == synthetic_scene['detection'].confidence
    assert result.source_snapshot_id == 'source'
    assert result.source_capture_stamp_ns == synthetic_scene['snapshot'].stamp_ns
    assert result.header.stamp.sec == 2
    assert result.header.frame_id == 'base_link'
    assert result.mask.header.frame_id == synthetic_scene['snapshot'].source_frame
    assert result.observed_cloud.width == result.valid_depth_points == 2000
    assert not np.shares_memory(handler._reference.cached.snapshot.rgb, synthetic_scene['snapshot'].rgb)


def test_stale_capture_is_rejected_before_inference(synthetic_scene):
    handler, _, pin, events = make_service(synthetic_scene)
    result = handler.execute(Service.Request(operation=Service.Request.OBSERVE,
        reference_id=pin.reference_id, after_stamp_ns=2_000_000_000), Service.Response())
    assert not result.success and result.error_code == result.ERROR_RGBD_TIMEOUT
    assert events == ['capture']


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
            reference_tracker=SimpleNamespace(track=lambda *args: s['mask']),
            parameter_overrides=[Parameter('enable_reference_observation', value=True)])
        node._snapshot_cache.put('source', CachedDetectionSnapshot(
            s['snapshot'], (s['detection'],), (.8,), s['transform'], s['snapshot'].source_frame))
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
