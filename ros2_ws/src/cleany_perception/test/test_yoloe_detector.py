from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from cleany_perception.adapters.yoloe_detector import YoloeDetector
from cleany_perception.core.models import FailureKind, InspectionFailure


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
        boxes = [
            SimpleNamespace(
                xyxy=np.array([[2.0, 3.0, 12.0, 15.0]]),
                cls=np.array([1.0]),
                conf=np.array([0.82]),
            )
        ]
        return [SimpleNamespace(boxes=boxes, names=self.names)]


def _model_files(tmp_path: Path) -> tuple[Path, Path]:
    checkpoint = tmp_path / 'yoloe-26n-seg.pt'
    checkpoint.write_bytes(b'checkpoint')
    encoder_directory = tmp_path / 'encoder'
    encoder_directory.mkdir()
    (encoder_directory / 'mobileclip2_b.ts').write_bytes(b'encoder')
    return checkpoint, encoder_directory


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
