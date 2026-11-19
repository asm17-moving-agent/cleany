"""One local instance-segmentation pass followed by one semantic API request."""
from __future__ import annotations

from collections.abc import Callable, Sequence
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import time
from typing import Any

from cleany_perception.core.ports import SemanticClassifierPort
from cleany_perception.adapters.yoloe_detector import YoloeDetector
from cleany_perception.core.models import Detection2D, RgbArray


def make_semantic_classifier(*, model: str, object_types: Sequence[str],
                             prompt_file: str | Path,
                             audit: Callable[[dict], None] | None = None,
                             **kwargs: Any) -> SemanticClassifierPort:
    """Choose explicitly configured semantics; never fall back after a failure."""
    if model.removeprefix('models/').startswith('gemini-decision-'):
        from cleany_perception.adapters.decision_classifier import DecisionClassifier
        return DecisionClassifier(model=model, object_types=object_types,
                                  prompt_file=prompt_file, audit=audit, **kwargs)
    from cleany_perception.adapters.gemini_detector import GeminiClassifier
    # The deployed legacy adapter predates progress and cancellation callbacks.
    kwargs.pop('progress', None)
    kwargs.pop('cancelled', None)
    return GeminiClassifier(model=model, **kwargs)


class YoloeGeminiDetector:
    def __init__(self, yoloe: YoloeDetector, classifier: SemanticClassifierPort, *,
                 vlm_active: Callable[[bool], None] | None = None,
                 timing: Callable[[str, float], None] | None = None) -> None:
        self._yoloe = yoloe
        self._classifier = classifier
        self._vlm_active = vlm_active or (lambda active: None)
        self._timing = timing or (lambda stage, seconds: None)

    def prepare(self) -> None:
        self._yoloe.prepare()
        self._classifier.prepare()

    def detect(self, rgb: RgbArray, query: str) -> Sequence[Detection2D]:
        return self._classify(rgb, self._detect_local(rgb, query))

    def _detect_local(self, rgb, query):
        began = time.monotonic()
        try:
            return tuple(self._yoloe.detect_scene(rgb, query))
        finally:
            self._timing('yoloe', time.monotonic() - began)

    def _classify(self, rgb, detections):
        if not detections:
            return ()
        began = time.monotonic()
        try:
            self._vlm_active(True)
            return self._classifier.classify(rgb, detections)
        finally:
            self._vlm_active(False)
            self._timing('vlm_classification', time.monotonic() - began)

    def detect_with_preprocessing(self, rgb: RgbArray, query: str,
                                  preprocess: Callable[[Sequence[Detection2D]], None]):
        detections = self._detect_local(rgb, query)
        if not detections:
            return ()
        # Only the remote request runs in the worker. Local GPU inference has
        # completed; geometry preprocessing uses CPU and publishes no target.
        with ThreadPoolExecutor(max_workers=1, thread_name_prefix='vlm-classify') as executor:
            classified = executor.submit(self._classify, rgb, detections)
            began = time.monotonic()
            try:
                preprocess(detections)
            finally:
                self._timing('geometry_during_vlm', time.monotonic() - began)
            return classified.result()
