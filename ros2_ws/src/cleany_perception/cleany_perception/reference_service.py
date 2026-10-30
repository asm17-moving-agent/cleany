"""Reference lifecycle and fresh RGB-D observation, serialized by the inspector."""
from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Callable
from uuid import uuid4

import numpy as np
from cleany_interfaces.srv import ObserveObjectReference
from rclpy.time import Time
from sensor_msgs.msg import Image

from cleany_perception.core.models import (
    CameraIntrinsics, Detection2D, InspectionFailure, FailureKind, RgbArray,
)
from cleany_perception.core.ports import ReferenceTrackerPort
from cleany_perception.core.point_cloud import ColoredPointCloud
from cleany_perception.core.reference_observation import (
    ReferenceObservationConfig, observe_surface,
)
from cleany_perception.point_cloud_message import colored_point_cloud_message
from cleany_perception.rgbd_snapshot import snapshot_from_messages
from cleany_perception.byte_buffer import numpy_byte_array


@dataclass(frozen=True)
class PinnedReference:
    reference_id: str
    snapshot_id: str
    object_id: int
    rgb: RgbArray
    intrinsics: CameraIntrinsics
    stamp_ns: int
    source_frame: str
    detection: Detection2D
    created_at: float


class ReferenceService:
    def __init__(self, *, cache, buffer, tracker: ReferenceTrackerPort, lookup_transform,
                 target_frame: str, depth_scale: float, timeout_seconds: float,
                 ttl_seconds: float, config: ReferenceObservationConfig,
                 clock: Callable[[], float] = time.monotonic):
        if not math.isfinite(ttl_seconds) or ttl_seconds <= 0.:
            raise ValueError('Reference lifetime must be finite and positive')
        self._cache, self._buffer, self._tracker = cache, buffer, tracker
        self._lookup_transform = lookup_transform
        self._target_frame, self._depth_scale = target_frame, depth_scale
        self._timeout, self._ttl, self._config = timeout_seconds, ttl_seconds, config
        self._clock = clock
        self._reference: PinnedReference | None = None
        self._expired_reference_id: str | None = None
        self._reference_lock = threading.Lock()

    def expire(self) -> bool:
        """Release an idle expired seed; never interrupt an ongoing observation."""
        if not self._reference_lock.acquire(blocking=False):
            return False
        try:
            if (self._reference is not None
                    and self._clock() - self._reference.created_at >= self._ttl):
                self._expired_reference_id = self._reference.reference_id
                self._reference = None
                return True
            return False
        finally:
            self._reference_lock.release()

    def execute(self, request: ObserveObjectReference.Request,
                response: ObserveObjectReference.Response) -> ObserveObjectReference.Response:
        with self._reference_lock:
            return self._execute(request, response)

    def _execute(self, request: ObserveObjectReference.Request,
                 response: ObserveObjectReference.Response) -> ObserveObjectReference.Response:
        try:
            if request.operation == request.PIN:
                if request.reference_id or request.after_stamp_ns:
                    raise ValueError('PIN accepts only a source snapshot and object ID')
                cached = self._cache.get(request.source_snapshot_id)
                if cached is None or not 1 <= request.source_object_id <= len(cached.detections):
                    raise ValueError('Source snapshot expired or object ID is invalid')
                # Own just one reference independently of the detector's bounded
                # cache; later fresh detector requests cannot evict this seed.
                rgb = cached.snapshot.rgb.copy()
                rgb.flags.writeable = False
                self._reference = PinnedReference(
                    uuid4().hex, request.source_snapshot_id, request.source_object_id,
                    rgb, cached.snapshot.intrinsics, cached.snapshot.stamp_ns,
                    cached.snapshot.source_frame,
                    cached.detections[request.source_object_id - 1], self._clock())
                self._expired_reference_id = None
            elif request.operation in (request.OBSERVE, request.CLEAR):
                if request.source_snapshot_id or request.source_object_id:
                    raise ValueError('OBSERVE/CLEAR accepts only the opaque reference ID')
                reference = self._reference
                if (reference is None and request.reference_id
                        and request.reference_id == self._expired_reference_id):
                    if request.operation == request.CLEAR:
                        self._expired_reference_id = None
                        response.success = True
                        response.message = 'Reference cleared'
                        return response
                    raise ValueError('Reference expired; a new detector seed is required')
                if (reference is None or not request.reference_id
                        or request.reference_id != reference.reference_id):
                    raise ValueError('Unknown reference ID')
                if request.operation == request.CLEAR:
                    self._reference = None
                    self._expired_reference_id = None
                    response.success = True
                    response.message = 'Reference cleared'
                    return response
                if self._clock() - reference.created_at >= self._ttl:
                    self._expired_reference_id = reference.reference_id
                    self._reference = None
                    raise ValueError('Reference expired; a new detector seed is required')
            else:
                raise ValueError('Unknown reference operation')
            reference = self._reference
            detection = reference.detection
            response.reference_id = reference.reference_id
            response.source_snapshot_id = reference.snapshot_id
            response.source_object_id = reference.object_id
            response.source_capture_stamp_ns = reference.stamp_ns
            response.source_label = detection.label
            response.source_detector_label = detection.detector_label
            response.source_confidence = detection.confidence
            if request.operation == request.OBSERVE:
                self._observe(request, response, reference, detection)
            response.success = True
            response.message = ('Reference pinned' if request.operation == request.PIN else
                                'YOLOE reference association + current RGB-D visible surface')
        except ValueError as error:
            response.error_code = response.ERROR_REFERENCE
            response.message = str(error)
        except InspectionFailure as error:
            response.error_code = {
                FailureKind.RGBD_TIMEOUT: response.ERROR_RGBD_TIMEOUT,
                FailureKind.MASK: response.ERROR_MASK,
                FailureKind.DEPTH: response.ERROR_DEPTH,
                FailureKind.TF: response.ERROR_TF,
            }.get(error.kind, response.ERROR_INTERNAL)
            response.message = str(error)
        except Exception as error:
            response.error_code = response.ERROR_INTERNAL
            response.message = f'Reference observation failed: {error}'
        return response

    def _observe(self, request, response, reference, detection):
        if request.after_stamp_ns < 0:
            raise ValueError('Observation lower capture bound cannot be negative')
        # A newly received RealSense frame may have been exposed before the
        # request. Wait within the original timeout for an actually newer
        # capture instead of failing on the first in-flight frame.
        lower_capture_ns = max(request.after_stamp_ns, reference.stamp_ns)
        deadline = time.monotonic() + self._timeout
        sequence = self._buffer.sequence
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise InspectionFailure(FailureKind.RGBD_TIMEOUT, 'No capture newer than requested/reference before timeout')
            messages = self._buffer.wait_for_new(sequence, remaining)
            if messages.sequence <= sequence:
                raise InspectionFailure(FailureKind.RGBD_TIMEOUT, 'RGB-D sequence did not advance')
            sequence = messages.sequence
            if messages.stamp_ns > lower_capture_ns:
                break
        snapshot = snapshot_from_messages(messages, depth_16u_scale_m=self._depth_scale)
        if (snapshot.source_frame != reference.source_frame
                or snapshot.intrinsics != reference.intrinsics):
            raise ValueError('Camera calibration/frame changed since the reference')
        transform = self._lookup_transform(snapshot)  # Capture TF before model inference.
        mask = self._tracker.track(reference.rgb, detection, snapshot.rgb)
        stamp = Time(nanoseconds=snapshot.stamp_ns).to_msg()
        response.mask = Image(height=mask.shape[0], width=mask.shape[1],
                              encoding='mono8', step=mask.shape[1])
        response.mask.data = numpy_byte_array(mask.astype(np.uint8)*255)
        response.mask.header.stamp = stamp
        response.mask.header.frame_id = snapshot.source_frame
        response.header.stamp = stamp
        response.header.frame_id = self._target_frame
        surface = observe_surface(snapshot, mask, transform, self._config)
        for index, axis in enumerate(('x', 'y', 'z')):
            setattr(response.observed_center, axis, float(surface.center[index]))
            setattr(response.observed_extent, axis, float(surface.extent[index]))
        response.mask_pixels = surface.mask_pixels
        response.valid_depth_points = len(surface.points)
        response.valid_depth_fraction = surface.valid_depth_fraction
        valid = (mask & np.isfinite(snapshot.depth_m)
                 & (snapshot.depth_m >= self._config.minimum_depth_m)
                 & (snapshot.depth_m <= self._config.maximum_depth_m))
        response.observed_cloud = colored_point_cloud_message(
            ColoredPointCloud(surface.points, snapshot.rgb[valid]), stamp, self._target_frame)
