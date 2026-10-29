"""Associate one approved instance without revising its semantic decision."""
from dataclasses import replace
from typing import Sequence

import numpy as np

from .models import Detection2D, FailureKind, InspectionFailure
from .wrist_observation import bbox_iou


def match_approved_target(source: Detection2D, detections: Sequence[Detection2D],
                          image_shape: tuple[int, int], minimum_iou: float) -> Detection2D:
    """Require one same-class instance and retain only the approved semantics.

    Class changes and multiple plausible instances remain failures. Fresh
    confidence, mask and geometry are never replaced by the source values.
    """
    key = source.detector_label.casefold().strip()
    if not key:
        raise InspectionFailure(FailureKind.MASK, '승인 관측에 YOLOE 원래 클래스가 없습니다.')
    candidates = [item for item in detections
                  if (item.detector_label or item.label).casefold().strip() == key
                  and bbox_iou(source.bbox, item.bbox) >= minimum_iou]
    if not candidates:
        raise InspectionFailure(FailureKind.MASK, '승인한 물체를 최신 영상에서 찾지 못했습니다.')
    if len(candidates) != 1:
        raise InspectionFailure(FailureKind.MASK,
                                f'승인 물체의 후보가 {len(candidates)}개여서 구별할 수 없습니다.')
    fresh = candidates[0]
    mask = fresh.segmentation_mask
    if (mask is None or mask.shape != image_shape or mask.dtype != np.bool_
            or not np.any(mask)):
        raise InspectionFailure(FailureKind.MASK, '승인 물체의 최신 마스크가 유효하지 않습니다.')
    return replace(fresh, label=source.label, detector_label=source.detector_label,
                   sorting_category=source.sorting_category)
