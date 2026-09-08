from types import SimpleNamespace
import threading
from copy import deepcopy
import numpy as np
import pytest
from sensor_msgs.msg import Image, CameraInfo
from cleany_interfaces.srv import ObserveWristTarget as Service
from cleany_perception.core.models import BoundingBox2D, Detection2D, ObjectMask, RigidTransform
from cleany_perception.core.wrist_observation import WristObservationConfig, project_box, verify_mask
from cleany_perception.wrist_service import WristService


def identity():
    return RigidTransform(translation=np.zeros(3),rotation=np.eye(3))


@pytest.mark.parametrize('center,size', [((0,0,0),(0.2,)*3), ((3,0,1),(0.2,)*3),
    ((0,0,1),(-1,1,1)), ((float('nan'),0,1),(.2,)*3)])
def test_projection_rejects_unobservable_geometry(center,size):
    with pytest.raises(ValueError):
        project_box(center,np.eye(3),size,identity(),np.array([[100,0,50],[0,100,50],[0,0,1]]),100,100)


def test_projection_and_mask_consistency():
    box=project_box((0,0,1),np.eye(3),(.2,)*3,identity(),np.array([[100,0,50],[0,100,50],[0,0,1]]),100,100)
    mask=np.zeros((100,100),bool); mask[40:60,40:60]=True
    verify_mask(mask,box)
    with pytest.raises(ValueError): verify_mask(~mask,box)
    with pytest.raises(ValueError): verify_mask(np.zeros_like(mask),box)
    with pytest.raises(ValueError): verify_mask(np.roll(mask,35,axis=1),box)


@pytest.fixture
def service():
    node=SimpleNamespace(_busy_lock=threading.Lock(),_busy=False,
        _sensor_callback_group=None,_action_callback_group=None,
        create_subscription=lambda *a,**k: None, create_service=lambda *a,**k: None,
        create_publisher=lambda *a,**k: SimpleNamespace(publish=lambda message: None),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=2_100_000_000)),
        _transformer=SimpleNamespace(lookup=lambda *a: identity()),
        _debug_publisher=SimpleNamespace(publish=lambda x: None))
    detection=Detection2D('cup',.8,BoundingBox2D(40,40,60,60))
    mask=np.zeros((100,100),bool); mask[40:60,40:60]=True
    service=WristService(node,SimpleNamespace(detect=lambda *a: (detection,)),
        SimpleNamespace(segment=lambda *a: (ObjectMask(detection,mask,.9),)),
        SimpleNamespace(track_sequence=lambda *a, **k: mask), require_redetection=True)
    image=Image(width=100,height=100,encoding='rgb8',step=300,data=bytes(30000))
    image.header.frame_id='left_wrist_rgb_optical_frame'; image.header.stamp.sec=2
    info=CameraInfo(width=100,height=100,k=[100.,0.,50.,0.,100.,50.,0.,0.,1.],d=[0.]*5)
    info.header=deepcopy(image.header)
    service.receive(service.images['left'],image); service.receive(service.infos['left'],info)
    request=Service.Request(arm='left',source_snapshot_id='head-1',source_object_id=1,label='cup',source_confidence=.8)
    request.expected_pose.header.frame_id='base_link'; request.expected_pose.pose.position.z=1.
    request.expected_pose.header.stamp.sec=1
    request.expected_pose.pose.orientation.w=1.; request.size.x=request.size.y=request.size.z=.2
    return service,request


def test_rgb_only_handoff_preserves_source_and_stamped_mask(service):
    s,req=service
    out=s.execute(req,Service.Response())
    assert out.success and out.reference_id
    assert out.source_snapshot_id=='head-1' and out.source_object_id==1
    assert out.header.stamp.sec==2 and out.mask.encoding=='mono8'
    assert not hasattr(out,'observed_center')
    assert not s.node._busy


@pytest.mark.parametrize('pixels,visible', [(0, False), (400, True), (10000, False)])
def test_background_tracking_publishes_source_capture_and_visibility(service, monkeypatch, pixels, visible):
    from cleany_perception.core.continuous_tracking import TrackingFrame, TrackingResult
    s, req = service
    callbacks, messages = {}, []
    s.continuous_tracking = True
    s.node.get_logger = lambda: SimpleNamespace(info=lambda _: None)
    s.tracking_publisher.publish = messages.append
    def worker(start, next_frame, stamp, report, error):
        callbacks.update(report=report, error=error)
        return SimpleNamespace(close=lambda **k: None)
    monkeypatch.setattr('cleany_perception.wrist_service.ContinuousTracking', worker)
    result = s.execute(req, Service.Response())
    assert result.success
    msg = s.images['left'][2_000_000_000]
    mask = np.zeros((100, 100), bool)
    mask.flat[:pixels] = True
    callbacks['report'](TrackingResult(TrackingFrame(2_000_000_000, s.reference['rgb'], (msg, None)), mask, .1, 1))
    status = messages[-1]
    assert status.header == msg.header and status.valid and status.visible is visible
    assert status.reference_id == result.reference_id
    assert status.source_snapshot_id == req.source_snapshot_id and status.source_object_id == 1
    assert status.arm == 'left' and status.mask_pixels == pixels and status.image_pixels == 10000
    callbacks['error'](ValueError('camera stopped'))
    assert not messages[-1].valid and not messages[-1].visible
    assert messages[-1].reason == 'camera stopped' and messages[-1].header.stamp.sec == 0
    s.close()
    callbacks['error'](ValueError('expected clear'))
    assert len(messages) == 2


def test_projected_handoff_uses_head_prior_without_new_semantic_detection(service):
    s,req=service
    s.require_redetection=False
    s.detector.detect=lambda *a: pytest.fail('Projected handoff must not redetect')
    out=s.execute(req,Service.Response())
    assert out.success and out.source_snapshot_id==req.source_snapshot_id
    assert 'original head label retained' in out.message


def test_fresh_continuous_tracking_renews_lease_but_stale_or_missing_does_not(service, monkeypatch):
    from cleany_perception.core.continuous_tracking import TrackingFrame, TrackingResult
    s,req=service
    wall=[100.]
    monkeypatch.setattr('cleany_perception.wrist_service.time.monotonic',lambda:wall[0])
    callbacks={}
    s.continuous_tracking=True
    s.node.get_logger=lambda:SimpleNamespace(info=lambda _:None)
    def worker(start,next_frame,stamp,report,error):
        callbacks.update(report=report)
        return SimpleNamespace(close=lambda **k:None)
    monkeypatch.setattr('cleany_perception.wrist_service.ContinuousTracking',worker)
    assert s.execute(req,Service.Response()).success
    msg=deepcopy(s.images['left'][2_000_000_000]);msg.header.stamp.sec=3
    s.node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=3_100_000_000))
    mask=np.zeros((100,100),bool);mask[40:60,40:60]=True
    wall[0]=200.
    callbacks['report'](TrackingResult(TrackingFrame(3_000_000_000,s.reference['rgb'],(msg,None)),mask,.1,1))
    wall[0]=250.
    assert s.reference['created']==100. and not s._reference_expired(s.reference)
    callbacks['report'](TrackingResult(TrackingFrame(3_000_000_000,s.reference['rgb'],(msg,None)),mask,.1,2))
    assert s.reference['last_valid_at']==200.  # Replay does not renew.
    callbacks['report'](TrackingResult(TrackingFrame(4_000_000_000,s.reference['rgb'],(msg,None)),np.zeros_like(mask),.1,3))
    wall[0]=321.
    assert s._reference_expired(s.reference)


def test_check_requires_same_arm_reference_and_source(service):
    s,req=service; out=s.execute(req,Service.Response())
    req.operation=req.CHECK; req.reference_id=out.reference_id
    req.arm='right'
    assert not s.execute(req,Service.Response()).success
    req.arm='left'; req.source_object_id=2
    assert not s.execute(req,Service.Response()).success


def test_busy_or_wrong_frame_fails_without_handoff(service):
    s,req=service
    s.node._busy=True
    assert not s.execute(req,Service.Response()).success
    s.node._busy=False
    s.images['left'][2_000_000_000].header.frame_id='head_camera_rgb_optical_frame'
    assert not s.execute(req,Service.Response()).success
    assert s.reference is None


def test_handoff_rejects_ambiguous_detection(service):
    s,req=service
    original=s.detector.detect(None,None)
    s.detector.detect=lambda *a: original*2
    out=s.execute(req,Service.Response())
    assert not out.success and 'found 2' in out.message


def test_check_rejects_reused_frame_and_clear_releases_reference(service):
    s,req=service; out=s.execute(req,Service.Response())
    req.operation=req.CHECK; req.reference_id=out.reference_id
    out=s.execute(req,Service.Response())
    assert not out.success and 'timestamp changed' in out.message
    req.operation=req.CLEAR
    assert s.execute(req,Service.Response()).success
    assert s.reference is None


@pytest.mark.parametrize('stamp,now', [(0, 2_100_000_000), (3, 2_100_000_000), (1, 62_000_000_000)])
def test_handoff_rejects_missing_future_or_stale_head_prior(service, stamp, now):
    s, req = service
    req.expected_pose.header.stamp.sec = stamp
    s.node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now))
    out = s.execute(req, Service.Response())
    assert not out.success and 'head observation timestamp' in out.message
    assert s.reference is None


@pytest.mark.parametrize('fault', ['nan_score', 'low_score', 'wrong_shape', 'nan_distortion', 'zero_focal'])
def test_invalid_wrist_inference_or_calibration_is_not_success(service, fault):
    s, req = service
    if fault in ('nan_score', 'low_score', 'wrong_shape'):
        mask = np.ones((10, 10), bool) if fault == 'wrong_shape' else np.zeros((100, 100), bool)
        score = float('nan') if fault == 'nan_score' else (0.2 if fault == 'low_score' else 0.9)
        s.segmenter.segment = lambda *a: (SimpleNamespace(mask=mask, score=score),)
    else:
        info = s.infos['left'][2_000_000_000]
        if fault == 'nan_distortion':
            info.d[0] = float('nan')
        else:
            info.k[0] = 0.
    assert not s.execute(req, Service.Response()).success
    assert s.reference is None and not s.node._busy


@pytest.mark.parametrize('changes', [dict(maximum_frame_age_seconds=0.),
    dict(minimum_segmentation_score=float('nan')), dict(minimum_visible_fraction=1.1)])
def test_wrist_limits_reject_invalid_configuration(changes):
    with pytest.raises(ValueError):
        WristObservationConfig(**changes)


def test_wrist_history_is_bounded_and_bridges_only_selected_arm_frames(service):
    s, req = service
    s.config = WristObservationConfig(history_maximum_frames=3, tracking_support_frames=2)
    out = s.execute(req, Service.Response())
    image = deepcopy(s.images['left'][2_000_000_000])
    info = deepcopy(s.infos['left'][2_000_000_000])
    for second in (4, 6, 8, 10, 12):
        frame = deepcopy(image)
        frame.header.stamp.sec = second
        frame.data = bytes([second])*len(frame.data)
        s.receive(s.images['left'], frame)
        wrong_arm = deepcopy(frame)
        wrong_arm.header.frame_id = 'right_wrist_rgb_optical_frame'
        s.receive(s.images['right'], wrong_arm)
    assert list(s.history) == [8_000_000_000, 10_000_000_000, 12_000_000_000]
    info.header.stamp.sec = 12
    s.receive(s.infos['left'], info)
    s.node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=12_100_000_000))
    def track(frames, detection, *, reference_mask):
        assert [int(frame.mean()) for frame in frames] == [0, 8, 10, 12]
        np.testing.assert_array_equal(reference_mask, s.reference['mask'])
        return s.segmenter.segment(None, None)[0].mask
    s.tracker.track_sequence = track
    req.operation = req.CHECK
    req.reference_id = out.reference_id
    assert s.execute(req, Service.Response()).success
    req.operation = req.CLEAR
    assert s.execute(req, Service.Response()).success
    assert not s.history


@pytest.mark.parametrize('invalid_mask', [False, True])
def test_continuous_check_uses_post_request_result_without_batch_replay(service, monkeypatch, invalid_mask):
    from cleany_perception.core.continuous_tracking import TrackingFrame, TrackingResult
    s, req = service
    s.continuous_tracking = True
    s.node.get_logger = lambda: SimpleNamespace(info=lambda message: None)
    s.tracker.start_stream = lambda *a, **k: None
    s.tracker.track_sequence = lambda *a, **k: pytest.fail('Must not replay batch video during CHECK')
    calls = []
    class Worker:
        def __init__(self, start, next_frame, stamp, report, on_error):
            calls.append(('start', stamp))
        def wait_after(self, after, **kwargs):
            assert after == 2_500_000_000
            calls.append(('check', after))
            msg = deepcopy(s.images['left'][2_000_000_000])
            info = deepcopy(s.infos['left'][2_000_000_000])
            msg.header.stamp.sec = 3
            info.header = deepcopy(msg.header)
            mask = s.reference['mask'].copy()
            if invalid_mask:
                mask[:] = False
            return TrackingResult(TrackingFrame(3_000_000_000, s.reference['rgb'], (msg, info)), mask, .1, 8)
        def close(self, **kwargs):
            calls.append(('close',))
    monkeypatch.setattr('cleany_perception.wrist_service.ContinuousTracking', Worker)
    handoff = s.execute(req, Service.Response())
    assert handoff.success
    req.operation, req.reference_id, req.after_stamp_ns = req.CHECK, handoff.reference_id, 2_500_000_000
    out = s.execute(req, Service.Response())
    assert out.success is not invalid_mask
    if not invalid_mask:
        assert out.header.stamp.sec == 3 and out.reference_id == handoff.reference_id
    req.operation = req.CLEAR
    assert s.execute(req, Service.Response()).success
    assert calls == [('start', 2_000_000_000), ('check', 2_500_000_000), ('close',)]
    assert s.reference is None and s.worker is None
