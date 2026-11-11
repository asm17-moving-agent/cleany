"""Read-only approval observations, independent of the manipulation reference."""
from __future__ import annotations

import time
from typing import TYPE_CHECKING

from cleany_interfaces.srv import RevalidateTarget

from .core.models import FailureKind, InspectionFailure
from .core.ports import DetectorPort
from .core.object_ranking import rank_detections_by_distance
from .core.target_revalidation import match_approved_target
from .core.tracking import TrackedDetection, TrackingOutput, TrackingState
from .rgbd_snapshot import snapshot_from_messages

if TYPE_CHECKING:
    from .inspection_node import InspectionNode


class TargetRevalidationService:
    """Freeze cached semantics while measuring one target in newer RGB-D."""

    def __init__(self, node: InspectionNode, detector: DetectorPort) -> None:
        self.node, self.detector = node, detector
        self.service = node.create_service(
            RevalidateTarget, 'perception/revalidate_target', self.execute,
            callback_group=node._action_callback_group)

    def execute(self, request: RevalidateTarget.Request,
                response: RevalidateTarget.Response) -> RevalidateTarget.Response:
        with self.node._busy_lock:
            if self.node._busy:
                response.error_code = response.ERROR_BUSY
                response.message = '인식기가 다른 관측을 처리하고 있습니다.'
                return response
            self.node._busy = True
        try:
            self._observe(request, response)
            response.success = True
            response.message = '승인한 분류를 유지하고 최신 YOLOE 및 깊이 관측으로 대상을 확인했습니다.'
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
            response.message = f'승인 대상 재확인 실패: {error}'
        finally:
            with self.node._busy_lock:
                self.node._busy = False
        return response

    def _observe(self, request: RevalidateTarget.Request,
                 response: RevalidateTarget.Response) -> None:
        node = self.node
        cached = node._snapshot_cache.get(request.source_snapshot_id)
        index = request.source_object_id - 1
        if cached is None or not 0 <= index < len(cached.detections):
            raise ValueError('승인한 관측이 만료되었거나 대상 번호가 유효하지 않습니다.')
        tracking = cached.tracking
        if (not request.tracking_session_id or not request.track_id or tracking is None
                or tracking.session_id != request.tracking_session_id
                or tracking.detections[index].track_id != request.track_id
                or tracking.detections[index].state != TrackingState.TRACKED
                or cached.representative_frame != node._target_frame
                or request.after_stamp_ns < 0):
            raise ValueError('승인한 물체의 추적 기준을 확인할 수 없습니다.')
        source = cached.detections[index]
        if not source.detector_label:
            raise ValueError('승인 관측에 YOLOE 원래 클래스가 없습니다. 새로 제안받으세요.')
        lower_stamp = max(cached.snapshot.stamp_ns, request.after_stamp_ns)
        deadline = time.monotonic() + node._snapshot_timeout_seconds
        sequence = node._snapshot_buffer.sequence
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise InspectionFailure(FailureKind.RGBD_TIMEOUT, '승인 이후의 새 RGB-D 영상을 받지 못했습니다.')
            messages = node._snapshot_buffer.wait_for_new(sequence, remaining)
            if messages.sequence <= sequence:
                raise InspectionFailure(FailureKind.RGBD_TIMEOUT, 'RGB-D 영상이 갱신되지 않았습니다.')
            sequence = messages.sequence
            if messages.stamp_ns > lower_stamp:
                break
        snapshot = snapshot_from_messages(messages, depth_16u_scale_m=node._depth_16u_scale_m)
        if (snapshot.source_frame != cached.snapshot.source_frame
                or snapshot.intrinsics != cached.snapshot.intrinsics):
            raise ValueError('카메라 좌표계 또는 보정값이 변경되었습니다.')
        transform = node._lookup_capture_transform(snapshot)
        detections = self.detector.detect(snapshot.rgb, '')
        # Retain all boxes when excluding objects from support-plane fitting,
        # exactly as in the proposal's full-scene geometric measurement.
        measured = node._pipeline.estimate_detection_footprints(snapshot, detections, transform)
        fresh = match_approved_target(source, measured, snapshot.rgb.shape[:2],
            float(node.get_parameter('reference_minimum_detection_iou').value))
        ranked = rank_detections_by_distance(snapshot, (fresh,), transform, node._object_ranking_config)[0]
        if ranked.position is None:
            raise InspectionFailure(FailureKind.DEPTH, '승인 물체의 최신 깊이 또는 3D 위치가 유효하지 않습니다.')
        observed_tracking = TrackingOutput(tracking.session_id, tracking.epoch,
            (TrackedDetection(request.track_id, TrackingState.TRACKED, ranked.position),))
        response.source_snapshot_id = request.source_snapshot_id
        response.source_object_id = request.source_object_id
        response.detections = node._detections_message((ranked.detection,), snapshot.stamp_ns,
            snapshot.source_frame, node._snapshot_id(snapshot.stamp_ns, sequence),
            (ranked.distance_m,), observed_tracking, node._target_frame)
        response.detections.detections[0].object_id = request.source_object_id
