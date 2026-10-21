"""One local instance-segmentation pass followed by one semantic API request."""
from __future__ import annotations

from collections.abc import Sequence

from cleany_perception.adapters.gemini_detector import GeminiClassifier
from cleany_perception.adapters.yoloe_detector import YoloeDetector
from cleany_perception.core.models import Detection2D, RgbArray


class YoloeGeminiDetector:
    def __init__(self, yoloe: YoloeDetector, classifier: GeminiClassifier) -> None:
        self._yoloe = yoloe
        self._classifier = classifier

    def prepare(self) -> None:
        self._yoloe.prepare()
        self._classifier.prepare()

    def detect(self, rgb: RgbArray, query: str) -> Sequence[Detection2D]:
        detections = self._yoloe.detect(rgb, query)
        return self._classifier.classify(rgb, detections)
