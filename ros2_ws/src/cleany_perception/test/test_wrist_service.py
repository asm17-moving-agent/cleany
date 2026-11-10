from copy import deepcopy
from types import SimpleNamespace
import threading

import numpy as np
from cleany_interfaces.srv import ObserveWristTarget
from sensor_msgs.msg import Image, CameraInfo

from cleany_perception.core.models import BoundingBox2D, Detection2D, RigidTransform
from builtin_interfaces.msg import Time
from cleany_perception.wrist_service import WristService


def messages(second):
    image = Image(width=100, height=100, step=300, encoding='rgb8', data=bytes(30000))
    image.header.frame_id = 'left_wrist_rgb_optical_frame'
    image.header.stamp.sec = second
    info = CameraInfo(header=deepcopy(image.header), width=100, height=100,
                      k=[200., 0., 49.5, 0., 200., 49.5, 0., 0., 1.])
    return image, info


def test_handoff_and_fresh_check_preserve_identity_and_clear_reference():
    clock, statuses = [10_100_000_000], []
    mask = np.zeros((100, 100), dtype=bool)
    mask[30:70, 30:70] = True
    detector = SimpleNamespace(detect=lambda *_: (Detection2D(
        label='beverage can', confidence=.99, bbox=BoundingBox2D(30., 30., 70., 70.),
        segmentation_mask=mask.copy()),))
    node = SimpleNamespace(
        _busy=False, _busy_lock=threading.Lock(), _sensor_callback_group=None, _action_callback_group=None,
        _transformer=SimpleNamespace(lookup=lambda *_: RigidTransform(translation=np.zeros(3), rotation=np.eye(3))),
        _publish_debug_image=lambda _: None,
        create_publisher=lambda *_: SimpleNamespace(publish=statuses.append),
        create_subscription=lambda *_, **__: None, create_service=lambda *_, **__: None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=clock[0], to_msg=lambda: Time(
            sec=clock[0]//10**9, nanosec=clock[0]%10**9))),
        get_logger=lambda: SimpleNamespace(info=lambda _: None))
    service = WristService(node, detector)
    request = ObserveWristTarget.Request(arm='left', source_snapshot_id='source', source_object_id=7,
        label='beverage can', source_confidence=.9, after_stamp_ns=9_000_000_000)
    request.expected_pose.header.frame_id = 'base_link'
    request.expected_pose.header.stamp.sec = 10
    request.expected_pose.pose.position.z = 1.
    request.expected_pose.pose.orientation.w = 1.
    request.size.x = request.size.y = request.size.z = .2
    image, info = messages(10)
    service.receive(service.images['left'], image)
    service.receive(service.infos['left'], info)
    try:
        handoff = service.execute(request, ObserveWristTarget.Response())
        assert handoff.success, handoff.message
        assert handoff.source_object_id == 7
        request.reference_id = handoff.reference_id
        request.operation = request.CHECK
        request.after_stamp_ns = 10_000_000_000
        clock[0] = 11_100_000_000
        image, info = messages(11)
        service.receive(service.images['left'], image)
        service.receive(service.infos['left'], info)
        checked = service.execute(request, ObserveWristTarget.Response())
        assert checked.success, checked.message
        assert checked.reference_id == handoff.reference_id
        assert checked.header.stamp.sec == 11
        assert statuses[-1].valid and statuses[-1].visible
        request.operation = request.CLEAR
        assert service.execute(request, ObserveWristTarget.Response()).success
    finally:
        service.close(wait=True)
    assert service.reference is None
