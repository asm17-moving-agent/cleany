import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from cleany_perception.adapters.yoloe_detector import YoloeDetector, YoloeModelPool
from cleany_perception.adapters.yoloe_tensorrt import CudaSynchronizationError
from test_yoloe_detector import _Boxes
from cleany_perception.adapters.gemini_detector import parse_gemini_classifications
from cleany_perception.core.models import BoundingBox2D, Detection2D, InspectionFailure
from cleany_perception.model_contract import SCHEMA, sha256
from cleany_perception.model_runtime import resolve_model_assets


def engine_files(tmp_path: Path) -> Path:
    engine = tmp_path / 'test.engine'
    engine.write_bytes(b'engine')
    lock = tmp_path / 'input.lock.json'
    lock.write_text(json.dumps({'classes': ['cup'], 'export': {'imgsz': 640}}))
    validation = tmp_path / 'validation.json'
    validation.write_text(json.dumps({'passed': True, 'candidate_sha256': sha256(engine),
                                      'lock_sha256': sha256(lock)}))
    manifest = {'schema': SCHEMA, 'task': 'segment', 'status': 'validated',
                'classes': ['cup'], 'export': {'imgsz': 640}, 'lock_sha256': sha256(lock),
                'inputs': {'images': [1, 3, 640, 640]},
                'outputs': {'output0': [1, 300, 38], 'output1': [1, 32, 160, 160]},
                'artifacts': {engine.name: {'sha256': sha256(engine)}},
                'validation': {'filename': validation.name, 'sha256': sha256(validation)}}
    (tmp_path / 'manifest.json').write_text(json.dumps(manifest))
    return engine


class EngineModel:
    task = 'segment'
    names = {0: 'cup'}

    def __init__(self):
        shapes = {'images': [1, 3, 640, 640], 'output0': [1, 300, 38],
                  'output1': [1, 32, 160, 160]}
        self.predictor = SimpleNamespace(model=SimpleNamespace(
            dynamic=False, bindings={name: SimpleNamespace(shape=shape) for name, shape in shapes.items()}))
        self.calls = []
        self.empty = False

    def predict(self, image, **options):
        self.calls.append(options)
        boxes = _Boxes([] if self.empty else [[-3., 2., 150., 75., 0.8, 0]])
        return [SimpleNamespace(boxes=boxes, names=self.names,
                                masks=SimpleNamespace(data=np.ones((len(boxes), *image.shape[:2]))))]


def test_engine_uses_segment_loader_without_text_encoder_or_mutation(tmp_path):
    engine = engine_files(tmp_path)
    calls = []
    model = EngineModel()

    def factory(path, **options):
        calls.append((path, options))
        return model
    detector = YoloeDetector(str(engine), ['cup'], require_masks=True, model_factory=factory)
    detector.prepare()
    detections = detector.detect(np.zeros((80, 160, 3), dtype=np.uint8), '')
    detector.prepare()
    assert calls == [(str(engine), {'task': 'segment'})]
    assert len(model.calls) == 2
    assert all(not options['rect'] and options['retina_masks'] for options in model.calls)
    assert detections[0].bbox == BoundingBox2D(0., 2., 150., 75.)
    assert detections[0].segmentation_mask.shape == (80, 160)
    assert detections[0].segmentation_mask.dtype == np.bool_
    assert not detections[0].segmentation_mask.flags.writeable
    assert resolve_model_assets({'yoloe_model_path': str(engine), 'yoloe_text_encoder_directory': ''},
                                str(tmp_path), 'yoloe') == {'yoloe_model_path': str(engine)}


@pytest.mark.parametrize('change', ['hash', 'classes', 'input', 'prototype', 'coefficients',
                                    'status', 'validation', 'bindings'])
def test_engine_mismatch_fails_without_checkpoint_fallback(tmp_path, change):
    engine = engine_files(tmp_path)
    manifest_path = tmp_path / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    model = EngineModel()
    if change == 'hash':
        engine.write_bytes(b'corrupt')
    elif change == 'classes':
        manifest['classes'] = ['bottle']
    elif change == 'input':
        manifest['inputs']['images'] = [1, 3, 480, 640]
    elif change == 'prototype':
        manifest['outputs'].pop('output1')
    elif change == 'coefficients':
        manifest['outputs']['output0'] = [1, 300, 6]
    elif change == 'status':
        manifest['status'] = 'built_requires_validation'
    elif change == 'validation':
        (tmp_path / 'validation.json').write_text('{}')
    else:
        model.predictor.model.bindings['images'].shape = [1, 3, 320, 320]
    manifest_path.write_text(json.dumps(manifest))
    calls = []

    def factory(*args, **kwargs):
        calls.append(args)
        return model
    detector = YoloeDetector(str(engine), ['cup'], require_masks=True, model_factory=factory)
    with pytest.raises(InspectionFailure, match='Failed to load YOLOE TensorRT'):
        detector.prepare()
    assert len(calls) <= 1


def test_engine_empty_scene_and_original_mask_coordinates(tmp_path):
    model = EngineModel()
    model.empty = True
    detector = YoloeDetector(str(engine_files(tmp_path)), ['cup'], require_masks=True,
                             model_factory=lambda *args, **kwargs: model)
    assert detector.detect(np.zeros((720, 1280, 3), dtype=np.uint8), '') == ()
    model.empty = False
    original = model.predict

    def bad_mask(image, **kwargs):
        result = original(image, **kwargs)[0]
        result.masks.data = np.ones((1, 640, 640))
        return [result]
    model.predict = bad_mask
    with pytest.raises(InspectionFailure, match='image shape differs'):
        detector.detect(np.zeros((720, 1280, 3), dtype=np.uint8), '')


def test_async_runner_is_shared_and_receives_per_call_settings(monkeypatch, tmp_path):
    import cleany_perception.adapters.yoloe_detector as module
    calls = []
    class Runner:
        def __init__(self, model):
            self.model = model

        def predict(self, image, **options):
            calls.append(options)
            return self.model.predict(image)

    monkeypatch.setattr(module, 'TensorRTAsyncRunner', Runner)
    pool = YoloeModelPool()
    models = []
    def factory(*args, **kwargs):
        model = EngineModel()
        models.append(model)
        return model
    options = dict(model_path=str(engine_files(tmp_path)), classes=['cup'],
                   require_masks=True, model_factory=factory, model_pool=pool)
    first = YoloeDetector(**options)
    second = YoloeDetector(**options, confidence_threshold=0.4, maximum_detections=4)
    image = np.zeros((80, 160, 3), np.uint8)
    first.detect(image, '')
    second.detect(image, '')
    assert len(models) == 1
    assert first._execution is second._execution
    assert first.execution_mode == second.execution_mode == 'async'
    assert calls == [dict(confidence=.25, iou=.5, maximum_detections=10),
                     dict(confidence=.4, iou=.5, maximum_detections=4)]
    legacy = YoloeDetector(**options, engine_execution='ultralytics')
    legacy.prepare()
    assert len(models) == 2  # A legacy context must not alias the async pool entry.
    assert legacy.execution_mode == 'ultralytics'


@pytest.mark.parametrize('fatal', [False, True])
def test_async_failure_is_sticky_and_only_drained_failures_retry(monkeypatch, tmp_path, fatal):
    import cleany_perception.adapters.yoloe_detector as module
    attempts = []
    class Runner:
        def __init__(self, model):
            pass

        def predict(self, image, **options):
            attempts.append(True)
            if fatal:
                raise CudaSynchronizationError('stream failed')
            raise RuntimeError('enqueue refused after draining stream')

    monkeypatch.setattr(module, 'TensorRTAsyncRunner', Runner)
    model = EngineModel()
    detector = YoloeDetector(str(engine_files(tmp_path)), ['cup'], require_masks=True,
                             model_factory=lambda *args, **kwargs: model)
    detector.prepare()
    image = np.zeros((80, 160, 3), np.uint8)
    if fatal:
        for _ in range(2):
            with pytest.raises(InspectionFailure, match='stream failed'):
                detector.detect(image, '')
        assert len(model.calls) == 1  # Only startup validation used the original path.
        assert detector.execution_mode == 'failed'
    else:
        assert detector.detect(image, '')
        assert detector.detect(image, '')
        assert len(model.calls) == 3
        assert detector.execution_mode == 'ultralytics'
    assert attempts == [True]


def test_async_setup_failure_keeps_validated_engine(monkeypatch, tmp_path, caplog):
    import cleany_perception.adapters.yoloe_detector as module
    def unavailable(model):
        raise RuntimeError('unsupported backend')
    monkeypatch.setattr(module, 'TensorRTAsyncRunner', unavailable)
    model = EngineModel()
    detector = YoloeDetector(str(engine_files(tmp_path)), ['cup'], require_masks=True,
                             model_factory=lambda *args, **kwargs: model)
    detector.prepare()
    assert detector.detect(np.zeros((80, 160, 3), np.uint8), '')
    assert detector.execution_mode == 'ultralytics'
    assert 'unsupported backend' in caplog.text


def test_prepared_pt_keeps_export_embeddings_without_encoder(tmp_path):
    engine = engine_files(tmp_path)
    checkpoint = engine.with_suffix('.pt')
    checkpoint.write_bytes(b'prepared')
    manifest_path = tmp_path / 'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    manifest['artifacts'][checkpoint.name] = {'sha256': sha256(checkpoint)}
    manifest_path.write_text(json.dumps(manifest))
    model = EngineModel()
    model.to = lambda device: model
    detector = YoloeDetector(str(checkpoint), ['cup'], require_masks=True,
                             model_factory=lambda path: model)
    detector.prepare()
    assert model.calls == []
    assert len(detector.detect(np.zeros((80, 160, 3), np.uint8), '')) == 1


@pytest.mark.parametrize(('detail', 'expected'), [('paper cup', 'cup'), ('Cup', 'cup')])
def test_gemini_normalizes_type_and_preserves_detector_class_and_geometry(detail, expected):
    mask = np.ones((20, 30), dtype=np.bool_)
    detection = Detection2D('cup', .8, BoundingBox2D(2., 3., 12., 15.), segmentation_mask=mask)
    response = json.dumps({'objects': [{'id': 1, 'label': detail,
                                       'sorting_category': 'trash'}]})
    item = parse_gemini_classifications(response, [detection])[0]
    assert item.label == expected
    assert item.detector_label == 'cup'
    assert item.bbox is detection.bbox
    assert item.segmentation_mask is mask
    assert item.confidence == detection.confidence


def test_real_profile_head_reference_and_wrists_share_one_loaded_model(monkeypatch, tmp_path):
    import rclpy
    from rclpy.parameter import Parameter
    import cleany_perception.inspection_node as module
    from cleany_perception.model_runtime import load_model_profile
    profile = load_model_profile(Path(__file__).resolve().parents[1] / 'config/yoloe_real32.yaml')
    instances = []

    class Adapter:
        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self.prepared = 0
            instances.append(self)

        def prepare(self):
            self.prepared += 1
    monkeypatch.setattr(module, 'YoloeDetector', Adapter)
    monkeypatch.setattr(module.InspectionNode, '_resolve_local_model', lambda *args: None)
    monkeypatch.setattr(module, 'GeminiClassifier', lambda **kwargs: SimpleNamespace(prepare=lambda: None))
    profile.update(yoloe_model_path=str(tmp_path / 'test.engine'), preload_models=False,
                   enable_reference_observation=True, enable_wrist_observation=True)
    rclpy.init(args=[])
    node = None
    try:
        node = module.InspectionNode(transformer=object(),
                                     parameter_overrides=[Parameter(name, value=value)
                                                          for name, value in profile.items()])
        assert len(instances) == 1
        assert instances[0].prepared == 1  # Engine always validates before advertising services.
        assert instances[0].kwargs['classes'] == tuple(profile['yoloe_classes'])
        for wrapped in (node._wrist_service.detector, node._wrist_service.check_detector,
                        node._wrist_service.right_detector, node._wrist_service.right_check_detector):
            assert wrapped._adapter is instances[0]
            assert wrapped._lock is node._inference_lock
        assert node._observations.reference._tracker.detector._adapter is instances[0]
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
