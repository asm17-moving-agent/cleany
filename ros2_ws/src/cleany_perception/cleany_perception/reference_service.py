"""Reference lifecycle and fresh RGB-D observation, serialized by the inspector."""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
import time
from typing import Callable
from uuid import uuid4

import numpy as np
from cleany_interfaces.srv import ObserveObjectReference
from rclpy.time import Time
from sensor_msgs.msg import Image

from cleany_perception.core.models import InspectionFailure, FailureKind
from cleany_perception.core.ports import ReferenceTrackerPort
from cleany_perception.core.point_cloud import ColoredPointCloud
from cleany_perception.core.reference_observation import (
    ReferenceObservationConfig, observe_surface,
)
from cleany_perception.point_cloud_message import colored_point_cloud_message
from cleany_perception.rgbd_snapshot import snapshot_from_messages
from cleany_perception.snapshot_cache import CachedDetectionSnapshot


@dataclass(frozen=True)
class PinnedReference:
    reference_id: str
    snapshot_id: str
    object_id: int
    cached: CachedDetectionSnapshot
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

    def execute(self, request: ObserveObjectReference.Request,
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
                snapshot = replace(cached.snapshot, rgb=cached.snapshot.rgb.copy(),
                                   depth_m=cached.snapshot.depth_m.copy())
                snapshot.rgb.flags.writeable = snapshot.depth_m.flags.writeable = False
                self._reference = PinnedReference(
                    uuid4().hex, request.source_snapshot_id, request.source_object_id,
                    replace(cached, snapshot=snapshot), self._clock())
            elif request.operation in (request.OBSERVE, request.CLEAR):
                if request.source_snapshot_id or request.source_object_id:
                    raise ValueError('OBSERVE/CLEAR accepts only the opaque reference ID')
                reference = self._reference
                if (reference is None or not request.reference_id
                        or request.reference_id != reference.reference_id):
                    raise ValueError('Unknown reference ID')
                if request.operation == request.CLEAR:
                    self._reference = None
                    response.success = True
                    response.message = 'Reference cleared'
                    return response
                if self._clock() - reference.created_at >= self._ttl:
                    self._reference = None
                    raise ValueError('Reference expired; a new detector seed is required')
            else:
                raise ValueError('Unknown reference operation')
            reference = self._reference
            detection = reference.cached.detections[reference.object_id - 1]
            response.reference_id = reference.reference_id
            response.source_snapshot_id = reference.snapshot_id
            response.source_object_id = reference.object_id
            response.source_capture_stamp_ns = reference.cached.snapshot.stamp_ns
            response.source_label = detection.label
            response.source_confidence = detection.confidence
            if request.operation == request.OBSERVE:
                self._observe(request, response, reference, detection)
            response.success = True
            response.message = ('Reference pinned' if request.operation == request.PIN else
                                'SAM2 reference mask + current RGB-D visible surface; no fresh detection')
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
        messages = self._buffer.wait_for_new(self._buffer.sequence, self._timeout)
        snapshot = snapshot_from_messages(messages, depth_16u_scale_m=self._depth_scale)
        if snapshot.stamp_ns <= max(request.after_stamp_ns, reference.cached.snapshot.stamp_ns):
            raise InspectionFailure(FailureKind.RGBD_TIMEOUT, 'Current capture is not newer than requested/reference')
        if (snapshot.source_frame != reference.cached.snapshot.source_frame
                or snapshot.intrinsics != reference.cached.snapshot.intrinsics):
            raise ValueError('Camera calibration/frame changed since the reference')
        transform = self._lookup_transform(snapshot)  # Capture TF before model inference.
        mask = self._tracker.track(reference.cached.snapshot.rgb, detection, snapshot.rgb)
        stamp = Time(nanoseconds=snapshot.stamp_ns).to_msg()
        response.mask = Image(height=mask.shape[0], width=mask.shape[1],
                              encoding='mono8', step=mask.shape[1], data=(mask.astype(np.uint8)*255).tobytes())
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
