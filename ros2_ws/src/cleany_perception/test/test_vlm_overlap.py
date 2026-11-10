from dataclasses import replace
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from cleany_perception.adapters.yoloe_gemini import YoloeGeminiDetector
from cleany_perception.adapters.yoloe_detector import YoloeMaskSegmenter
from cleany_perception.core.models import PipelineConfig
from cleany_perception.core.pipeline import InspectionPipeline
from cleany_perception.core.reconstruction_cache import ReconstructionCache


def test_geometry_runs_during_vlm_and_selected_reconstruction_reuses_it(synthetic_scene, monkeypatch):
    from cleany_perception.core import pipeline as module
    started, geometry_done = threading.Event(), threading.Event()
    notifications, timings, fits = [], [], []
    raw = replace(synthetic_scene['detection'], segmentation_mask=synthetic_scene['mask'])
    def classify(rgb, detections):
        started.set()
        assert geometry_done.wait(5.), 'Geometry must run while VLM is waiting'
        return [replace(detections[0], label='classified box', sorting_reason='Visible test evidence', sorting_category='lost_item')]
    original = module.fit_plane_ransac
    def fit(*args, **kwargs):
        assert started.wait(5.)
        fits.append(True)
        result = original(*args, **kwargs)
        geometry_done.set()
        return result
    monkeypatch.setattr(module, 'fit_plane_ransac', fit)
    detector = YoloeGeminiDetector(SimpleNamespace(detect=lambda *_: [raw]),
        SimpleNamespace(classify=classify), vlm_active=notifications.append,
        timing=lambda stage, seconds: timings.append(stage))
    pipeline = InspectionPipeline(detector, YoloeMaskSegmenter(), None, 'base_link',
        PipelineConfig(plane_distance_threshold_m=.001, plane_minimum_inliers=100,
                       plane_minimum_inlier_ratio=.9))
    cache = ReconstructionCache()
    snapshot, transform = synthetic_scene['snapshot'], synthetic_scene['transform']
    detections = pipeline.detect(snapshot, '', capture_transform=transform, geometry_cache=cache)
    measured = pipeline.estimate_detection_footprints(snapshot, detections, transform, geometry_cache=cache)
    output = pipeline.inspect_selected(snapshot, measured, measured[0], transform, geometry_cache=cache)
    assert notifications == [True, False]
    assert set(timings) == {'yoloe', 'vlm_classification', 'geometry_during_vlm'}
    assert len(fits) == 1
    assert measured[0].label == output.objects[0].label == 'classified box'
    assert measured[0].sorting_category == 'lost_item'
    assert measured[0].estimated_width_m > 0
    assert cache.hits >= 4
    # Compare geometry with the uncached path; classification never changes it.
    baseline = pipeline.inspect_selected(snapshot, measured, measured[0], transform)
    np.testing.assert_allclose(output.objects[0].box.center, baseline.objects[0].box.center)
    np.testing.assert_allclose(output.objects[0].box.size, baseline.objects[0].box.size)


def test_vlm_error_closes_window_and_does_not_publish_speculative_detection(synthetic_scene):
    events = []
    def fail(*_):
        raise RuntimeError('remote failure')
    detector = YoloeGeminiDetector(SimpleNamespace(detect=lambda *_: [synthetic_scene['detection']]),
        SimpleNamespace(classify=fail), vlm_active=events.append)
    with pytest.raises(RuntimeError, match='remote failure'):
        detector.detect_with_preprocessing(synthetic_scene['snapshot'].rgb, '', lambda _: None)
    assert events == [True, False]


def test_empty_scene_does_not_start_gpu_or_vlm():
    detector = YoloeGeminiDetector(SimpleNamespace(detect=lambda *_: []),
        SimpleNamespace(classify=lambda *_: pytest.fail('VLM called for empty scene')),
        vlm_active=lambda _: pytest.fail('Warmup called for empty scene'))
    assert detector.detect_with_preprocessing(None, '', lambda _: pytest.fail('Geometry called')) == ()
