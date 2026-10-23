"""Associate a pinned head-camera instance with fresh YOLOE segmentation."""
from __future__ import annotations
import math
import numpy as np
from cleany_perception.core.models import Detection2D, FailureKind, InspectionFailure, MaskArray, RgbArray
from cleany_perception.core.ports import DetectorPort
from cleany_perception.core.wrist_observation import bbox_iou


class YoloeReferenceTracker:
    def __init__(self, detector: DetectorPort, *, minimum_iou: float = 0.2):
        if not math.isfinite(minimum_iou) or not 0 < minimum_iou <= 1:
            raise ValueError('Reference association IoU must be in (0, 1]')
        self.detector, self.minimum_iou = detector, minimum_iou

    def track(self, reference_rgb: RgbArray, detection: Detection2D,
              current_rgb: RgbArray) -> MaskArray:
        if current_rgb.shape != reference_rgb.shape:
            raise InspectionFailure(FailureKind.MASK, 'Reference camera dimensions changed')
        label = ' ' + (detection.detector_label or detection.label).casefold().strip() + ' '
        candidates = [item for item in self.detector.detect(current_rgb, '')
                      if ' ' + item.label.casefold().strip() + ' ' in label
                      and bbox_iou(item.bbox, detection.bbox) >= self.minimum_iou]
        if len(candidates) != 1:
            raise InspectionFailure(FailureKind.MASK,
                                    f'Fresh reference association requires one instance, found {len(candidates)}')
        mask = candidates[0].segmentation_mask
        if mask is None or mask.shape != current_rgb.shape[:2] or mask.dtype != np.bool_:
            raise InspectionFailure(FailureKind.MASK, 'Fresh YOLOE instance mask is missing or invalid')
        return np.array(mask, copy=True)
