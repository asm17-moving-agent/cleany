"""Bounded image-orientation fallback for the public, untuned wrist detector."""
from dataclasses import replace

import numpy as np

from cleany_perception.core.models import BoundingBox2D


class WristDetector:
    """Retry an empty detection on a half-turned image, preserving sensor coordinates."""

    def __init__(self, detector):
        self.detector = detector

    def prepare(self):
        self.detector.prepare()

    def detect(self, rgb, query):
        detections = self.detector.detect(rgb, query)
        if detections:
            return detections
        height, width = rgb.shape[:2]
        rotated = self.detector.detect(np.ascontiguousarray(rgb[::-1,::-1]), query)
        restored = []
        for detection in rotated:
            box = detection.bbox
            mask = detection.segmentation_mask
            restored.append(replace(detection,
                bbox=BoundingBox2D(width-box.x_max, height-box.y_max,
                                   width-box.x_min, height-box.y_min),
                segmentation_mask=None if mask is None else np.array(mask[::-1,::-1],copy=True)))
        return tuple(restored)
