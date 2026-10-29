from pathlib import Path
from types import SimpleNamespace
import json

import numpy as np
import pytest

from cleany_perception.adapters.yoloe_detector import YoloeDetector, YoloeMaskSegmenter
from cleany_perception.adapters.yoloe_detector import YoloeModelPool
from cleany_perception.adapters.gemini_detector import GeminiClassifier
from cleany_perception.adapters.yoloe_gemini import YoloeGeminiDetector
from cleany_perception.core.models import FailureKind, InspectionFailure


class _Boxes:
    def __init__(self, data):
        self.data = np.asarray(data, dtype=np.float32).reshape(-1, 6)

    def __len__(self):
        return len(self.data)


class _Model:
    def to(self, device):
        self.device = device
        return self

    def __init__(self) -> None:
        self.classes = []
        self.names = {0: 'cup', 1: 'wallet'}
        self.calls = []

    def set_classes(self, classes):
        self.classes = list(classes)

    def predict(self, image, **kwargs):
        self.calls.append((image.copy(), kwargs))
        boxes = _Boxes([[2.0, 3.0, 12.0, 15.0, 0.82, 1.0]])
        return [SimpleNamespace(boxes=boxes, names=self.names)]


def _model_files(tmp_path: Path) -> tuple[Path, Path]:
    checkpoint = tmp_path / 'yoloe-26n-seg.pt'
    checkpoint.write_bytes(b'checkpoint')
    encoder_directory = tmp_path / 'encoder'
    encoder_directory.mkdir()
    (encoder_directory / 'mobileclip2_b.ts').write_bytes(b'encoder')
    return checkpoint, encoder_directory


def test_shared_model_preserves_per_camera_thresholds_and_nms(tmp_path):
    checkpoint, encoder = _model_files(tmp_path)
    model = _Model(); loads = []
    def factory(path):
        loads.append(path)
        return model
    common = dict(model_path=str(checkpoint), classes=['cup','wallet'], device='cpu',
                  text_encoder_directory=str(encoder), model_factory=factory,
                  model_pool=YoloeModelPool())
    head = YoloeDetector(**common, confidence_threshold=.1)
    wrist = YoloeDetector(**common, confidence_threshold=.2, iou_threshold=.7,
                         maximum_detections=3, class_confidence_thresholds=[.2,.9])
    head.prepare(); wrist.prepare()
    image = np.zeros((20,30,3),dtype=np.uint8)
    assert len(head.detect(image,'')) == 1
    assert not wrist.detect(image,'')
    assert len(loads) == 1
    assert model.calls[-1][1]['conf'] == .2
    assert model.calls[-1][1]['iou'] == .7
    assert model.calls[-1][1]['max_det'] == 3


def test_yoloe_detector_loads_once_and_converts_boxes(tmp_path):
    checkpoint, encoder_directory = _model_files(tmp_path)
    model = _Model()
    factory_calls = []

    def factory(path):
        factory_calls.append((path, Path.cwd()))
        return model

    detector = YoloeDetector(
        str(checkpoint),
        ['cup', 'wallet'],
        device='cpu',
        image_size=512,
        confidence_threshold=0.1,
        iou_threshold=0.6,
        maximum_detections=4,
        text_encoder_directory=str(encoder_directory),
        model_factory=factory,
    )
    rgb = np.zeros((20, 30, 3), dtype=np.uint8)
    rgb[4, 5] = [17, 83, 201]
    rgb.setflags(write=False)
    original_directory = Path.cwd()

    detector.prepare()
    assert model.calls == []
    detections = detector.detect(rgb, 'ignored query')
    detector.detect(rgb, 'ignored again')

    assert Path.cwd() == original_directory
    assert factory_calls == [(str(checkpoint), encoder_directory)]
    assert model.classes == ['cup', 'wallet']
    assert model.device == 'cpu'
    np.testing.assert_array_equal(model.calls[0][0][4, 5], [201, 83, 17])
    np.testing.assert_array_equal(rgb[4, 5], [17, 83, 201])
    assert model.calls[0][0].flags.c_contiguous
    assert len(detections) == 1
    assert detections[0].label == 'wallet'
    assert detections[0].confidence == pytest.approx(0.82)
    assert detections[0].bbox.x_min == pytest.approx(2.0)
    assert detections[0].bbox.y_max == pytest.approx(15.0)
    assert model.calls[0][1] == {
        'device': 'cpu',
        'imgsz': 512,
        'conf': 0.1,
        'iou': 0.6,
        'max_det': 4,
        'verbose': False,
        'rect': False,
    }


def test_yoloe_detector_reports_missing_assets(tmp_path):
    detector = YoloeDetector(
        str(tmp_path / 'missing.pt'),
        ['cup'],
        text_encoder_directory=str(tmp_path),
    )

    with pytest.raises(InspectionFailure) as raised:
        detector.detect(np.zeros((10, 10, 3), dtype=np.uint8), 'cup')

    assert raised.value.kind == FailureKind.DETECTOR_API


def test_box_results_cross_to_cpu_once_and_masks_remain_owned(tmp_path):
    checkpoint, encoder = _model_files(tmp_path)
    model = _Model()
    transfers = []
    data = np.array([[2, 3, 12, 15, .8, 1], [1, 2, 8, 10, .9, 0]], dtype=np.float32)
    class Tensor:
        def detach(self):
            return self

        def cpu(self):
            transfers.append(True)
            return data
    class TwoBoxes:
        data = Tensor()

        def __len__(self):
            return 2
    boxes = TwoBoxes()
    masks = np.ones((2, 20, 30), np.uint8)
    model.predict = lambda *args, **kwargs: [SimpleNamespace(
        boxes=boxes, names=model.names, masks=SimpleNamespace(data=masks))]
    detector = YoloeDetector(str(checkpoint), ['cup', 'wallet'], device='cpu',
        text_encoder_directory=str(encoder), require_masks=True, model_factory=lambda _: model)
    result = detector.detect(np.zeros((20, 30, 3), np.uint8), '')
    assert transfers == [True]
    assert [item.label for item in result] == ['wallet', 'cup']
    masks[:] = 0
    data[:] = 0
    assert result[0].segmentation_mask.all()
    assert not result[0].segmentation_mask.flags.writeable
    assert result[0].bbox.x_max == 12


def test_yoloe_detector_applies_class_confidence_thresholds(tmp_path):
    checkpoint, encoder_directory = _model_files(tmp_path)
    detector = YoloeDetector(
        str(checkpoint), ['cup', 'wallet'], device='cpu',
        confidence_threshold=0.08,
        class_confidence_thresholds=(0.25, 0.9),
        text_encoder_directory=str(encoder_directory),
        model_factory=lambda _path: _Model(),
    )
    assert detector.detect(np.zeros((20, 30, 3), dtype=np.uint8), '') == ()

    with pytest.raises(ValueError, match='must match classes'):
        YoloeDetector(str(checkpoint), ['cup', 'wallet'],
                      class_confidence_thresholds=(0.25,))


def test_yoloe_detector_rejects_invalid_image(tmp_path):
    checkpoint, encoder_directory = _model_files(tmp_path)
    detector = YoloeDetector(
        str(checkpoint),
        ['cup'],
        text_encoder_directory=str(encoder_directory),
        model_factory=lambda _path: _Model(),
    )

    with pytest.raises(InspectionFailure) as raised:
        detector.detect(np.zeros((10, 10), dtype=np.uint8), 'cup')

    assert raised.value.kind == FailureKind.DETECTOR_RESPONSE


def test_yoloe_detector_maps_invalid_result_to_response_failure(tmp_path):
    checkpoint, encoder_directory = _model_files(tmp_path)
    model = _Model()
    model.predict = lambda _image, **_kwargs: []
    detector = YoloeDetector(
        str(checkpoint),
        ['cup'],
        text_encoder_directory=str(encoder_directory),
        model_factory=lambda _path: model,
    )

    with pytest.raises(InspectionFailure) as raised:
        detector.detect(np.zeros((10, 10, 3), dtype=np.uint8), 'cup')

    assert raised.value.kind == FailureKind.DETECTOR_RESPONSE


def test_yoloe_seg_uses_one_inference_and_preserves_instance_mask(tmp_path):
    checkpoint, encoder_directory = _model_files(tmp_path)
    model = _Model()
    original_predict = model.predict

    def predict(image, **kwargs):
        result = original_predict(image, **kwargs)[0]
        mask = np.zeros((1, 20, 30), dtype=np.uint8)
        mask[:, 4:12, 5:10] = 1
        result.masks = SimpleNamespace(data=mask)
        return [result]

    model.predict = predict
    detector = YoloeDetector(
        str(checkpoint), ['cup', 'wallet'], device='cpu',
        text_encoder_directory=str(encoder_directory),
        model_factory=lambda _path: model, require_masks=True,
    )
    rgb = np.zeros((20, 30, 3), dtype=np.uint8)
    detections = detector.detect(rgb, '')
    masks = YoloeMaskSegmenter().segment(rgb, detections)
    assert len(model.calls) == 1
    assert model.calls[0][1]['retina_masks'] is True
    assert masks[0].mask.shape == (20, 30)
    assert masks[0].mask[5, 6]
    assert not masks[0].mask[0, 0]
    assert not detections[0].segmentation_mask.flags.writeable


def test_yoloe_gemini_classifies_same_segmentation_instance(tmp_path):
    checkpoint, encoder_directory = _model_files(tmp_path)
    model = _Model()
    original_predict = model.predict

    def predict(image, **kwargs):
        result = original_predict(image, **kwargs)[0]
        result.masks = SimpleNamespace(data=np.ones((1, 20, 30), dtype=np.uint8))
        return [result]

    model.predict = predict
    requests = []

    def classify(image_bytes, prompt, schema):
        requests.append((image_bytes, prompt, schema))
        return json.dumps({'objects': [{
            'id': 1, 'label': 'computer mouse',
            'sorting_category': 'lost_item',

        }]})

    detector = YoloeGeminiDetector(
        YoloeDetector(str(checkpoint), ['cup', 'wallet'], device='cpu',
                      text_encoder_directory=str(encoder_directory),
                      model_factory=lambda _path: model, require_masks=True),
        GeminiClassifier('gemini-test', response_provider=classify),
    )
    rgb = np.zeros((20, 30, 3), dtype=np.uint8)
    detections = detector.detect(rgb, 'old detection prompt')
    masks = YoloeMaskSegmenter().segment(rgb, detections)
    assert len(model.calls) == len(requests) == len(masks) == 1
    assert detections[0].label == 'computer mouse'
    assert detections[0].detector_label == 'wallet'
    assert detections[0].sorting_category == 'lost_item'
    assert detections[0].confidence == pytest.approx(0.82)
    assert detections[0].bbox.x_min == pytest.approx(2.0)
    assert masks[0].mask.all()
    assert 'old detection prompt' not in requests[0][1]
    assert requests[0][2]['properties']['objects']['items']['properties']['id']['type'] == 'INTEGER'


def test_yoloe_seg_rejects_box_without_mask(tmp_path):
    checkpoint, encoder_directory = _model_files(tmp_path)
    detector = YoloeDetector(
        str(checkpoint), ['cup'], device='cpu',
        text_encoder_directory=str(encoder_directory),
        model_factory=lambda _path: _Model(), require_masks=True,
    )
    with pytest.raises(InspectionFailure) as raised:
        detector.detect(np.zeros((20, 30, 3), dtype=np.uint8), '')
    assert raised.value.kind == FailureKind.DETECTOR_RESPONSE
