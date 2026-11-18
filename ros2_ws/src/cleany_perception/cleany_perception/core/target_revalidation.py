"""Associate one approved instance without revising its semantic decision."""
from collections.abc import Callable
from dataclasses import replace
import math
from typing import Sequence

import numpy as np

from .models import Detection2D, FailureKind, InspectionFailure
from .object_ranking import RankedDetection
from .tracking import TrackedDetection, TrackingState
from .wrist_observation import bbox_iou


def _class_key(item: Detection2D) -> str:
    return (item.detector_label or item.label).strip().casefold()


def overlapping_instances(region: Detection2D, detections: Sequence[Detection2D],
                          minimum_iou: float) -> list[Detection2D]:
    return [item for item in detections if _class_key(item) == _class_key(region)
            and bbox_iou(region.bbox, item.bbox) >= minimum_iou]


def identify_tracked_candidate(
    observations: Sequence[RankedDetection], references: Sequence[Detection2D],
    tracks: Sequence[TrackedDetection], approved_track_id: str,
    image_shape: tuple[int, int], minimum_iou: float, maximum_distance_m: float,
) -> Detection2D | None:
    """Require independent one-to-one 3D and mask evidence for every competing region."""
    if len(references) != len(tracks):
        return None
    identified = {}
    for observed in observations:
        current = observed.detection
        mask = current.segmentation_mask
        if observed.position is None or mask is None or mask.shape != image_shape or not mask.any():
            return None
        nearby = []
        for reference, track in zip(references, tracks):
            if _class_key(reference) != _class_key(current) or bbox_iou(reference.bbox, current.bbox) < minimum_iou:
                continue
            if track.state != TrackingState.TRACKED or not track.track_id or track.position is None:
                return None
            if math.dist(observed.position, track.position) <= maximum_distance_m:
                nearby.append((reference, track.track_id))
        if len(nearby) != 1:
            return None
        reference, track_id = nearby[0]
        previous = reference.segmentation_mask
        if track_id in identified or previous is None or previous.shape != image_shape:
            return None
        overlap = np.count_nonzero(previous & mask)
        if not overlap or overlap < minimum_iou * np.count_nonzero(previous | mask):
            return None
        identified[track_id] = current
    return identified.get(approved_track_id)


def match_approved_target(
    source: Detection2D, detections: Sequence[Detection2D], image_shape: tuple[int, int],
    minimum_iou: float, *,
    disambiguate: Callable[[Sequence[Detection2D]], Detection2D | None] | None = None,
) -> Detection2D:
    """Require one same-class instance and retain only the approved semantics.

    Class changes and multiple plausible instances remain failures. Fresh
    confidence, mask and geometry are never replaced by the source values.
    """
    if not source.detector_label.strip():
        raise InspectionFailure(FailureKind.MASK, '승인 관측에 YOLOE 원래 클래스가 없습니다.')
    candidates = overlapping_instances(source, detections, minimum_iou)
    if not candidates:
        raise InspectionFailure(FailureKind.MASK, '승인한 물체를 최신 영상에서 찾지 못했습니다.')
    if disambiguate is not None:
        fresh = disambiguate(candidates)
    else:
        fresh = candidates[0] if len(candidates) == 1 else None
    if fresh is None:
        if len(candidates) == 1:
            message = '새 영상의 검출 영역이 승인한 물체와 일치하는지 확인할 수 없습니다.'
        else:
            message = f'새 영상의 {len(candidates)}개 검출 영역 중에서 승인한 물체 1개를 식별하지 못했습니다.'
        raise InspectionFailure(FailureKind.MASK, message)
    mask = fresh.segmentation_mask
    if (mask is None or mask.shape != image_shape or mask.dtype != np.bool_
            or not np.any(mask)):
        raise InspectionFailure(FailureKind.MASK, '승인 물체의 최신 마스크가 유효하지 않습니다.')
    return replace(fresh, label=source.label, detector_label=source.detector_label,
                   sorting_category=source.sorting_category, sorting_reason=source.sorting_reason,
                   semantic_assessment=source.semantic_assessment)
