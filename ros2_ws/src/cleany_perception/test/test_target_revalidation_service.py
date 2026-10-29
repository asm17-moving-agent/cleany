from dataclasses import replace
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest
from cleany_interfaces.srv import RevalidateTarget as Service

from cleany_perception.core.object_ranking import ObjectRankingConfig
from cleany_perception.core.tracking import ObjectTracker
from cleany_perception.inspection_node import InspectionNode
from cleany_perception.rgbd_snapshot import SynchronizedRgbdMessages
from cleany_perception.snapshot_cache import CachedDetectionSnapshot, DetectionSnapshotCache
from cleany_perception.target_revalidation_service import TargetRevalidationService
from test_inspection_node import _sensor_messages


def fixture(scene):
    source = replace(scene['detection'], label='eye drops', detector_label='box',
                     sorting_category='lost_item', segmentation_mask=scene['mask'])
    tracker = ObjectTracker()
    tracking = tracker.update('session', 'source', [source.label], [(0., 0., .2)], detector_labels=['box'])
    cache = DetectionSnapshotCache()
    cache.put('source', CachedDetectionSnapshot(snapshot=scene['snapshot'], detections=(source,),
        detection_distances_m=(.2,), capture_transform=scene['transform'],
        color_frame=scene['snapshot'].source_frame, tracking=tracking, representative_frame='base_link'))
    events = []
    def capture(sequence, timeout):
        events.append('capture')
        return SynchronizedRgbdMessages(sequence + 1, 2_000_000_000, *_sensor_messages(scene, 2_000_000_000))
    def transform(snapshot):
        events.append('tf')
        return scene['transform']
    def detect(*_):
        events.append('local_yoloe')
        return (replace(scene['detection'], confidence=.7, segmentation_mask=scene['mask']),)
    node = NS(_snapshot_cache=cache, _busy_lock=threading.Lock(), _busy=False,
        _snapshot_buffer=NS(sequence=1, wait_for_new=capture), _snapshot_timeout_seconds=.01,
        _depth_16u_scale_m=.001, _target_frame='base_link', _lookup_capture_transform=transform,
        _object_ranking_config=ObjectRankingConfig(), _action_callback_group=None,
        _pipeline=NS(estimate_detection_footprints=lambda snapshot, detections, transform: tuple(
            replace(d, estimated_width_m=.05, estimated_height_m=.07) for d in detections)),
        get_parameter=lambda key: NS(value=.2), create_service=lambda *_args, **_kwargs: None,
        _detections_message=InspectionNode._detections_message, _snapshot_id=InspectionNode._snapshot_id)
    service = TargetRevalidationService(node, NS(detect=detect))
    request = Service.Request(source_snapshot_id='source', source_object_id=1,
        tracking_session_id='session', track_id=tracking.detections[0].track_id, after_stamp_ns=1_800_000_000)
    return service, request, events, tracker


def test_fresh_local_observation_retains_approval_without_mutating_cached_scene_or_tracker(synthetic_scene):
    service, request, events, tracker = fixture(synthetic_scene)
    result = service.execute(request, Service.Response())
    assert result.success, result.message
    assert events == ['capture', 'tf', 'local_yoloe']
    fresh = result.detections.detections[0]
    assert (fresh.label, fresh.sorting_category, fresh.detector_label) == ('eye drops', 'lost_item', 'box')
    assert fresh.track_id == request.track_id and fresh.object_id == request.source_object_id
    assert fresh.confidence == pytest.approx(.7) and fresh.position_valid
    assert result.detections.header.stamp.sec == 2
    assert result.source_snapshot_id == 'source'
    assert service.node._snapshot_cache.get('source').detections[0].confidence == .95
    assert tracker.sessions['session'].last_snapshot == 'source'
    assert not service.node._busy


def test_support_plane_measurement_keeps_neighbor_boxes_but_returns_only_target(synthetic_scene):
    service, request, _, _ = fixture(synthetic_scene)
    detect = service.detector.detect
    service.detector.detect = lambda *args: detect(*args) + (replace(synthetic_scene['detection'], label='cup'),)
    estimate = service.node._pipeline.estimate_detection_footprints
    def measure(snapshot, detections, transform):
        assert len(detections) == 2 and detections[1].label == 'cup'
        return estimate(snapshot, detections, transform)
    service.node._pipeline.estimate_detection_footprints = measure
    result = service.execute(request, Service.Response())
    assert result.success and len(result.detections.detections) == 1


@pytest.mark.parametrize('case', ['expired', 'object', 'session', 'track', 'stale', 'camera', 'depth', 'busy', 'duplicate'])
def test_revalidation_rejects_unverifiable_target(synthetic_scene, case):
    service, request, events, _ = fixture(synthetic_scene)
    if case == 'expired': request.source_snapshot_id = 'missing'
    if case == 'object': request.source_object_id = 2
    if case == 'session': request.tracking_session_id = 'other'
    if case == 'track': request.track_id = 'other'
    if case == 'stale': request.after_stamp_ns = 2_000_000_000
    if case == 'camera':
        cached = service.node._snapshot_cache.get('source')
        service.node._snapshot_cache.put('source', replace(cached, snapshot=replace(cached.snapshot, source_frame='other')))
    if case == 'depth': synthetic_scene['snapshot'].depth_m[:] = np.nan
    if case == 'busy': service.node._busy = True
    if case == 'duplicate':
        detect = service.detector.detect
        service.detector.detect = lambda *args: detect(*args) * 2
    result = service.execute(request, Service.Response())
    assert not result.success
    assert not result.detections.detections
    assert service.node._busy == (case == 'busy')
    if case in ('expired', 'object', 'session', 'track', 'stale', 'camera', 'busy'):
        assert 'local_yoloe' not in events
