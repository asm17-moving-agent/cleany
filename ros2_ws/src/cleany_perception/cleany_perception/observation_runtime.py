"""Fresh head reference observations using the selected YOLOE instance model."""
from __future__ import annotations
from dataclasses import fields
from functools import wraps
from cleany_interfaces.srv import ObserveObjectReference
from cleany_perception.adapters.yoloe_reference_tracker import YoloeReferenceTracker
from cleany_perception.core.reference_observation import ReferenceObservationConfig
from cleany_perception.reference_service import ReferenceService


class SerializedInference:
    def __init__(self, adapter, lock):
        self._adapter, self._lock = adapter, lock

    def __getattr__(self, name):
        member = getattr(self._adapter, name)
        if not callable(member):
            return member
        @wraps(member)
        def call(*args, **kwargs):
            with self._lock:
                return member(*args, **kwargs)
        return call


def declare_observation_parameters(node) -> None:
    node.declare_parameter('enable_reference_observation', False)
    node.declare_parameter('reference_ttl_seconds', 120.0)
    node.declare_parameter('reference_minimum_detection_iou', 0.2)
    config = ReferenceObservationConfig()
    for field in fields(config):
        node.declare_parameter('reference_' + field.name, getattr(config, field.name))


class ObservationRuntime:
    def __init__(self, node, detector, segmenter=None, tracker=None):
        self.node, self.reference, self.service = node, None, None
        get = lambda key: node.get_parameter(key).value
        if not get('enable_reference_observation'):
            return
        tracker = tracker or YoloeReferenceTracker(
            detector, minimum_iou=float(get('reference_minimum_detection_iou')))
        self.reference = ReferenceService(
            cache=node._snapshot_cache, buffer=node._snapshot_buffer,
            tracker=SerializedInference(tracker, node._inference_lock),
            lookup_transform=node._lookup_capture_transform,
            target_frame=node._target_frame, depth_scale=node._depth_16u_scale_m,
            timeout_seconds=node._snapshot_timeout_seconds,
            ttl_seconds=get('reference_ttl_seconds'),
            config=ReferenceObservationConfig(**{
                field.name: get('reference_' + field.name)
                for field in fields(ReferenceObservationConfig())}),
        )
        self.service = node.create_service(
            ObserveObjectReference, 'perception/observe_object_reference', self.execute,
            callback_group=node._action_callback_group)

    def execute(self, request, response):
        with self.node._busy_lock:
            if self.node._busy:
                response.error_code = response.ERROR_BUSY
                response.message = 'Inspector is busy'
                return response
            self.node._busy = True
        try:
            return self.reference.execute(request, response)
        finally:
            with self.node._busy_lock:
                self.node._busy = False

    def close(self):
        self.reference = None

    def expire_reference(self) -> bool:
        return self.reference.expire() if self.reference is not None else False
