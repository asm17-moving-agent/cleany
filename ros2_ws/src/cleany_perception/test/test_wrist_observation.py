from types import SimpleNamespace
import threading
from copy import deepcopy
import numpy as np
import pytest
from PIL import Image as PilImage
from sensor_msgs.msg import Image, CameraInfo
from cleany_interfaces.srv import ObserveWristTarget as Service
from cleany_perception.core.models import BoundingBox2D, Detection2D, RigidTransform
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


def instance(label: str = 'cup') -> Detection2D:
    mask = np.zeros((100, 100), bool)
    mask[40:60, 40:60] = True
    return Detection2D(label, .8, BoundingBox2D(40, 40, 60, 60), segmentation_mask=mask)


def receive_frame(service: WristService, arm: str, stamp: int) -> None:
    # HANDOFF clears the opposite arm's cache; subsequent CHECK uses this arm.
    source = arm if 2_000_000_000 in service.images[arm] else 'left'
    image = deepcopy(service.images[source][2_000_000_000])
    info = deepcopy(service.infos[source][2_000_000_000])
    image.header.frame_id = f'{arm}_wrist_rgb_optical_frame'
    image.header.stamp.sec = stamp
    info.header = deepcopy(image.header)
    service.node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=stamp * 10**9 + 100_000_000))
    service.receive(service.images[arm], image)
    service.receive(service.infos[arm], info)


def prepare_check(service: WristService, request: Service.Request, handoff: Service.Response) -> None:
    receive_frame(service, request.arm, 3)
    request.operation = request.CHECK
    request.reference_id = handoff.reference_id
    request.after_stamp_ns = 2_000_000_000


@pytest.fixture
def service():
    node=SimpleNamespace(_busy_lock=threading.Lock(),_busy=False,
        _sensor_callback_group=None,_action_callback_group=None,
        create_subscription=lambda *a,**k: None, create_service=lambda *a,**k: None,
        create_publisher=lambda *a,**k: SimpleNamespace(publish=lambda message: None),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=2_100_000_000)),
        _transformer=SimpleNamespace(lookup=lambda *a: identity()),
        _debug_publisher=SimpleNamespace(publish=lambda x: None))
    mask=np.zeros((100,100),bool); mask[40:60,40:60]=True
    detection=Detection2D('cup',.8,BoundingBox2D(40,40,60,60), segmentation_mask=mask)
    service=WristService(node,SimpleNamespace(detect=lambda *a: (detection,)))
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
    assert out.status == out.OK
    assert out.source_snapshot_id=='head-1' and out.source_object_id==1
    assert out.header.stamp.sec==2 and out.mask.encoding=='mono8'
    assert not hasattr(out,'observed_center')
    assert not s.node._busy


def test_yoloe_wrist_handoff_and_check_use_instance_masks(service):
    s, req = service
    detection = instance('cup')
    s.detector.detect = lambda *args: (detection,)

    req.label = 'red paper cup'
    handoff = s.execute(req, Service.Response())
    assert handoff.success and 'YOLOE-seg' in handoff.message
    assert 'rgb' not in s.reference and 'mask' not in s.reference

    s.check_detector = SimpleNamespace(detect=lambda *args: (detection,))
    s.detector.detect = lambda *args: pytest.fail('CHECK must use the held-object detector')

    prepare_check(s, req, handoff)
    checked = s.execute(req, Service.Response())
    assert checked.success and checked.header.stamp.sec == 3
    assert checked.reference_id == handoff.reference_id


def test_yoloe_wrist_fails_closed_on_wrong_or_ambiguous_instance(service):
    s, req = service
    detection = instance('cup')
    shifted = np.zeros((100, 100), bool)
    shifted[40:60, 45:65] = True
    second = Detection2D('cup', .7, BoundingBox2D(45, 40, 65, 60),
                         segmentation_mask=shifted)
    s.detector.detect = lambda *args: (detection, second)
    result = s.execute(req, Service.Response())
    assert not result.success and 'found 2' in result.message
    assert s.reference is None
    s.detector.detect = lambda *args: (Detection2D(
        'computer mouse', .8, detection.bbox, segmentation_mask=detection.segmentation_mask),)
    result = s.execute(req, Service.Response())
    assert not result.success and 'found 0' in result.message
    s.detector.detect = lambda *args: (Detection2D(
        'cupboard', .8, detection.bbox, segmentation_mask=detection.segmentation_mask),)
    result = s.execute(req, Service.Response())
    assert not result.success and 'found 0' in result.message


def test_yoloe_missing_wrist_target_can_save_diagnostic_rgb(service, tmp_path):
    s, req = service
    s.detector.detect = lambda *args: ()
    s.failure_image_directory = tmp_path
    result = s.execute(req, Service.Response())
    files = list(tmp_path.glob('wrist_missing_*.png'))
    assert not result.success and len(files) == 1
    assert PilImage.open(files[0]).size == (100, 100)


def test_missing_check_preserves_provenance_and_requires_new_frame_for_recovery(service):
    s, req = service
    handoff = s.execute(req, Service.Response())
    prepare_check(s, req, handoff)
    s.check_detector = SimpleNamespace(detect=lambda *args: ())
    missing = s.execute(req, Service.Response())
    assert not missing.success and missing.status == missing.NOT_DETECTED
    assert missing.reference_id == handoff.reference_id
    assert (missing.source_snapshot_id, missing.source_object_id) == ('head-1', 1)
    assert missing.header.stamp.sec == 3 and missing.header.frame_id == 'left_wrist_rgb_optical_frame'
    assert not missing.mask.data
    reused = s.execute(req, Service.Response())
    assert not reused.success and reused.status == reused.ERROR
    assert 'timestamp changed' in reused.message
    receive_frame(s, req.arm, 4)
    req.after_stamp_ns = 3 * 10**9
    s.check_detector = SimpleNamespace(detect=lambda *args: (instance(),))
    recovered = s.execute(req, Service.Response())
    assert recovered.success and recovered.status == recovered.OK
    assert recovered.reference_id == handoff.reference_id and recovered.header.stamp.sec == 4


@pytest.mark.parametrize('fault', ['busy', 'capture', 'model', 'calibration', 'ambiguous', 'confidence'])
def test_check_sensor_and_contract_errors_are_not_missing_detections(service, fault):
    s, req = service
    handoff = s.execute(req, Service.Response())
    prepare_check(s, req, handoff)

    def failed(*args):
        raise RuntimeError('sensor/model unavailable')

    if fault == 'busy': s.node._busy = True
    if fault == 'capture': s.capture = failed
    if fault == 'model': s.check_detector = SimpleNamespace(detect=failed)
    if fault == 'calibration': s.infos['left'][3 * 10**9].k[0] = 0.
    if fault == 'ambiguous':
        a = instance()
        b = Detection2D('cup', .7, a.bbox,
                        segmentation_mask=np.roll(a.segmentation_mask, 5, axis=1))
        s.check_detector = SimpleNamespace(detect=lambda *args: (a, b))
    if fault == 'confidence':
        s.check_detector = SimpleNamespace(detect=lambda *args: (SimpleNamespace(confidence=float('nan')),))
    result = s.execute(req, Service.Response())
    assert not result.success and result.status == result.ERROR


def test_yoloe_wrist_collapses_identical_instance_predictions(service):
    s, req = service
    detection = instance('cup')
    duplicate = Detection2D('cup', .4, detection.bbox,
                            segmentation_mask=detection.segmentation_mask.copy())
    s.detector.detect = lambda *args: (detection, duplicate)
    assert s.execute(req, Service.Response()).success


def test_yoloe_left_mouse_check_uses_mouse_detector(service):
    s, req = service
    req.label = 'computer mouse'
    detection = instance('computer mouse')
    s.detector.detect = lambda *args: (detection,)
    handoff = s.execute(req, Service.Response())
    assert handoff.success
    s.check_detector = SimpleNamespace(
        detect=lambda *args: pytest.fail('general CHECK detector must not run'))
    s.mouse_check_detector = SimpleNamespace(detect=lambda *args: (detection,))
    prepare_check(s, req, handoff)
    assert s.execute(req, Service.Response()).success


def test_yoloe_right_handoff_can_reuse_head_detector(service):
    s, req = service
    receive_frame(s, 'right', 2)
    req.arm = 'right'
    detection = instance('cup')
    s.detector.detect = lambda *args: pytest.fail('left wrist detector must not run')
    s.right_detector = SimpleNamespace(detect=lambda *args: (detection,))
    assert s.execute(req, Service.Response()).success


def test_yoloe_right_handoff_falls_back_to_shared_wrist_detector(service):
    s, req = service
    receive_frame(s, 'right', 2)
    req.arm = 'right'
    detection = instance('cup')
    s.right_detector = SimpleNamespace(detect=lambda *args: ())
    s.detector.detect = lambda *args: (detection,)
    s.node.get_logger = lambda: SimpleNamespace(info=lambda _: None)
    assert s.execute(req, Service.Response()).success


def test_yoloe_right_check_uses_held_detector(service):
    s, req = service
    receive_frame(s, 'right', 2)
    req.arm = 'right'
    detection = instance('cup')
    s.right_detector = SimpleNamespace(detect=lambda *args: (detection,))
    handoff = s.execute(req, Service.Response())
    assert handoff.success
    s.check_detector = SimpleNamespace(
        detect=lambda *args: pytest.fail('left CHECK detector must not run'))
    s.right_detector = SimpleNamespace(
        detect=lambda *args: pytest.fail('right HANDOFF detector must not run'))
    s.right_check_detector = SimpleNamespace(detect=lambda *args: (detection,))
    prepare_check(s, req, handoff)
    assert s.execute(req, Service.Response()).success


def test_yoloe_right_check_falls_back_to_general_held_detector(service):
    s, req = service
    receive_frame(s, 'right', 2)
    req.arm = 'right'
    detection = instance('crumpled tissue')
    req.label = 'crumpled tissue'
    s.right_detector = SimpleNamespace(detect=lambda *args: (detection,))
    handoff = s.execute(req, Service.Response())
    assert handoff.success
    s.right_check_detector = SimpleNamespace(detect=lambda *args: ())
    s.check_detector = SimpleNamespace(detect=lambda *args: (detection,))
    s.node.get_logger = lambda: SimpleNamespace(info=lambda _: None)
    prepare_check(s, req, handoff)
    assert s.execute(req, Service.Response()).success


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
    s.detector.detect=lambda *a: (original[0], Detection2D('cup',.8,BoundingBox2D(40,40,60,60), segmentation_mask=np.roll(original[0].segmentation_mask,5,axis=1)))
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


@pytest.mark.parametrize('fault', ['nan_confidence', 'low_confidence', 'wrong_shape', 'nan_distortion', 'zero_focal'])
def test_invalid_wrist_inference_or_calibration_is_not_success(service, fault):
    s, req = service
    if fault in ('nan_confidence', 'low_confidence', 'wrong_shape'):
        mask = np.ones((10, 10), bool) if fault == 'wrong_shape' else np.zeros((100, 100), bool)
        mask[40:60,40:60] = True
        score = float('nan') if fault == 'nan_confidence' else (0.2 if fault == 'low_confidence' else 0.9)
        detection = SimpleNamespace(label='cup', confidence=score, bbox=BoundingBox2D(40,40,60,60), segmentation_mask=mask)
        s.detector.detect = lambda *a: (detection,)
    else:
        info = s.infos['left'][2_000_000_000]
        if fault == 'nan_distortion':
            info.d[0] = float('nan')
        else:
            info.k[0] = 0.
    assert not s.execute(req, Service.Response()).success
    assert s.reference is None and not s.node._busy


@pytest.mark.parametrize('changes', [dict(maximum_frame_age_seconds=0.),
    dict(minimum_detection_confidence=float('nan')), dict(minimum_visible_fraction=1.1)])
def test_wrist_limits_reject_invalid_configuration(changes):
    with pytest.raises(ValueError):
        WristObservationConfig(**changes)


def test_handoff_drops_opposite_wrist_cache(service):
    s, req = service
    assert s.execute(req, Service.Response()).success
    image = deepcopy(s.images['left'][2_000_000_000])
    image.header.frame_id = 'right_wrist_rgb_optical_frame'
    image.header.stamp.sec = 4
    s.receive(s.images['right'], image)
    assert not s.images['right']
    assert not s.infos['right']
