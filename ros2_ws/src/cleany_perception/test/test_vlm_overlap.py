from concurrent.futures import Future
from dataclasses import replace
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from cleany_perception.adapters.yoloe_gemini import YoloeGeminiDetector
from cleany_perception.adapters.yoloe_detector import YoloeMaskSegmenter
from cleany_perception.core.models import PipelineConfig
from cleany_perception.core.pipeline import InspectionPipeline
from cleany_perception.core.reconstruction_cache import ReconstructionCache
from cleany_perception.vlm_warmup import VlmWarmupClient


def test_geometry_runs_during_vlm_and_selected_reconstruction_reuses_it(synthetic_scene, monkeypatch):
    from cleany_perception.core import pipeline as module
    started, geometry_done = threading.Event(), threading.Event()
    notifications, timings, fits = [], [], []
    raw = replace(synthetic_scene['detection'], segmentation_mask=synthetic_scene['mask'])
    def classify(rgb, detections):
        started.set()
        assert geometry_done.wait(5.), 'Geometry must run while VLM is waiting'
        return [replace(detections[0], label='classified box', sorting_category='lost_item')]
    original = module.fit_plane_ransac
    def fit(*args, **kwargs):
        assert started.wait(5.)
        fits.append(True)
        result = original(*args, **kwargs)
        geometry_done.set()
        return result
    monkeypatch.setattr(module, 'fit_plane_ransac', fit)
    detector = YoloeGeminiDetector(SimpleNamespace(detect=lambda *_: [raw]),
        SimpleNamespace(classify=classify), vlm_active=notifications.append,
        timing=lambda stage, seconds: timings.append(stage))
    pipeline = InspectionPipeline(detector, YoloeMaskSegmenter(), None, 'base_link',
        PipelineConfig(plane_distance_threshold_m=.001, plane_minimum_inliers=100,
                       plane_minimum_inlier_ratio=.9))
    cache = ReconstructionCache()
    snapshot, transform = synthetic_scene['snapshot'], synthetic_scene['transform']
    detections = pipeline.detect(snapshot, '', capture_transform=transform, geometry_cache=cache)
    measured = pipeline.estimate_detection_footprints(snapshot, detections, transform, geometry_cache=cache)
    output = pipeline.inspect_selected(snapshot, measured, measured[0], transform, geometry_cache=cache)
    assert notifications == [True, False]
    assert set(timings) == {'yoloe', 'vlm_classification', 'geometry_during_vlm'}
    assert len(fits) == 1
    assert measured[0].label == output.objects[0].label == 'classified box'
    assert measured[0].sorting_category == 'lost_item'
    assert measured[0].estimated_width_m > 0
    assert cache.hits >= 4
    # Compare geometry with the uncached path; classification never changes it.
    baseline = pipeline.inspect_selected(snapshot, measured, measured[0], transform)
    np.testing.assert_allclose(output.objects[0].box.center, baseline.objects[0].box.center)
    np.testing.assert_allclose(output.objects[0].box.size, baseline.objects[0].box.size)


def test_vlm_error_closes_window_and_does_not_publish_speculative_detection(synthetic_scene):
    events = []
    def fail(*_):
        raise RuntimeError('remote failure')
    detector = YoloeGeminiDetector(SimpleNamespace(detect=lambda *_: [synthetic_scene['detection']]),
        SimpleNamespace(classify=fail), vlm_active=events.append)
    with pytest.raises(RuntimeError, match='remote failure'):
        detector.detect_with_preprocessing(synthetic_scene['snapshot'].rgb, '', lambda _: None)
    assert events == [True, False]


def test_empty_scene_does_not_start_gpu_or_vlm():
    detector = YoloeGeminiDetector(SimpleNamespace(detect=lambda *_: []),
        SimpleNamespace(classify=lambda *_: pytest.fail('VLM called for empty scene')),
        vlm_active=lambda _: pytest.fail('Warmup called for empty scene'))
    assert detector.detect_with_preprocessing(None, '', lambda _: pytest.fail('Geometry called')) == ()


def test_warmup_stop_waits_for_start_acknowledgement_without_blocking():
    calls, futures = [], []
    def call(request):
        calls.append(request.data)
        futures.append(Future())
        return futures[-1]
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=call)
    node = SimpleNamespace(create_client=lambda *a, **kw: client, create_timer=lambda *a, **kw: None,
        get_logger=lambda: SimpleNamespace(warning=lambda _: None, info=lambda _: None))
    notifier = VlmWarmupClient(node, '/test')
    notifier.set_active(True)
    notifier.set_active(False)
    assert calls == [True]
    futures[0].set_result(SimpleNamespace(success=True))
    assert calls == [True, False]
    futures[1].set_result(SimpleNamespace(success=True))
    assert not notifier._pending


def test_missing_warmup_service_never_waits_or_blocks_perception():
    client = SimpleNamespace(service_is_ready=lambda: False,
        call_async=lambda _: pytest.fail('Absent service must not be called'))
    notifier = VlmWarmupClient(SimpleNamespace(create_client=lambda *a, **kw: client,
        create_timer=lambda *a, **kw: None), '/missing')
    notifier.set_active(True)
    notifier.set_active(False)
    assert not notifier._pending


def test_anygrasp_prepare_finishes_before_ik_and_fast_vlm_skips_ik():
    calls=[];futures=[]
    def client(name):
        def call(request):
            calls.append((name,request.data));futures.append(Future());return futures[-1]
        return SimpleNamespace(service_is_ready=lambda:True,call_async=call)
    node=SimpleNamespace(create_client=lambda _,name,**kw:client(name),
        create_timer=lambda *a,**kw:None,
        get_logger=lambda:SimpleNamespace(warning=lambda _:None,info=lambda _:None))
    notifier=VlmWarmupClient(node,'/ik',prepare_service='/prepare')
    notifier.set_active(True)
    assert calls==[('/prepare',True)]
    notifier.set_active(False)
    futures[0].set_result(SimpleNamespace(success=True))
    assert calls==[('/prepare',True),('/prepare',False)]
    futures[1].set_result(SimpleNamespace(success=True))
    assert not notifier._pending


def test_ordered_prepare_then_ik_then_reverse_cleanup():
    calls=[];futures=[]
    def create(_,name,**kw):
        def call(request):
            calls.append((name,request.data));futures.append(Future());return futures[-1]
        return SimpleNamespace(service_is_ready=lambda:True,call_async=call)
    notifier=VlmWarmupClient(SimpleNamespace(create_client=create,create_timer=lambda *a,**kw:None),
        '/ik',prepare_service='/prepare')
    notifier.set_active(True)
    futures[0].set_result(SimpleNamespace(success=True))
    assert calls==[('/prepare',True),('/ik',True)]
    notifier.set_active(False)
    futures[1].set_result(SimpleNamespace(success=True))
    assert calls[-1]==('/ik',False)
    futures[2].set_result(SimpleNamespace(success=True))
    assert calls[-1]==('/prepare',False)
    futures[3].set_result(SimpleNamespace(success=True))
    assert not notifier._prepared and not notifier._sent


@pytest.mark.parametrize('ends_before_discovery', [False, True])
def test_late_service_discovery_retries_only_during_vlm(ends_before_discovery):
    ready, calls, timers = [False], [], []
    def call(request):
        calls.append(request.data)
        future = Future()
        future.set_result(SimpleNamespace(success=True))
        return future
    client = SimpleNamespace(service_is_ready=lambda: ready[0], call_async=call)
    node = SimpleNamespace(create_client=lambda *a, **kw: client,
        create_timer=lambda period, callback, **kw: timers.append(callback))
    notifier = VlmWarmupClient(node, '/late')
    notifier.set_active(True)
    assert calls == []
    if ends_before_discovery:notifier.set_active(False)
    ready[0] = True
    timers[0]()
    assert calls == ([] if ends_before_discovery else [True])
    notifier.set_active(False)
    assert calls == ([] if ends_before_discovery else [True, False])


def test_vlm_window_notifications_cross_ros_in_order():
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import SetBool

    context = Context()
    rclpy.init(context=context, domain_id=178)
    server = rclpy.create_node('vlm_window_test_server', context=context)
    client = rclpy.create_node('vlm_window_test_client', context=context)
    executor = SingleThreadedExecutor(context=context)
    received = []
    def handle(request, response):
        received.append(request.data)
        response.success = True
        return response
    service = server.create_service(SetBool, '/test/vlm_window', handle)
    notifier = VlmWarmupClient(client, '/test/vlm_window')
    executor.add_node(server)
    executor.add_node(client)
    try:
        assert notifier._client.wait_for_service(timeout_sec=3.)
        notifier.set_active(True)
        notifier.set_active(False)
        # The API caller has already returned; ROS replies are processed later.
        assert received == []
        deadline = time.monotonic() + 5.
        while (len(received) < 2 or notifier._pending) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert received == [True, False]
        assert not notifier._pending
    finally:
        executor.shutdown()
        server.destroy_service(service)
        client.destroy_node()
        server.destroy_node()
        context.shutdown()
