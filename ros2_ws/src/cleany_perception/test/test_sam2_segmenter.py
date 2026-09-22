import numpy as np
import pytest

from cleany_perception.adapters.sam2_segmenter import Sam2Segmenter
from cleany_perception.adapters.sam2_reference_tracker import Sam2ReferenceTracker
from cleany_perception.adapters.sam2_runtime import Sam2SharedPredictors
from cleany_perception.core.models import (
    BoundingBox2D,
    Detection2D,
    FailureKind,
    InspectionFailure,
)


class _Predictor:
    def __init__(self, shape=(20, 30)) -> None:
        self.shape = shape
        self.images = []
        self.boxes = []

    def set_image(self, image):
        self.images.append(image.copy())

    def predict(self, box, multimask_output):
        self.boxes.append((box.copy(), multimask_output))
        mask = np.zeros(self.shape, dtype=np.bool_)
        mask[3:8, 4:10] = True
        return mask[None, :, :], np.array([0.88]), None


def _detection():
    return Detection2D(
        label='box',
        confidence=0.9,
        bbox=BoundingBox2D(4.0, 3.0, 10.0, 8.0),
    )


def test_sam2_segmenter_preserves_detection_order(tmp_path):
    checkpoint = tmp_path / 'sam2.pt'
    checkpoint.write_bytes(b'placeholder')
    predictor = _Predictor()
    factory_calls = []

    def factory(config, checkpoint_path, device):
        factory_calls.append((config, checkpoint_path, device))
        return predictor

    segmenter = Sam2Segmenter(
        'sam2-config',
        str(checkpoint),
        device='cpu',
        predictor_factory=factory,
    )
    rgb = np.zeros((20, 30, 3), dtype=np.uint8)

    segmenter.prepare()
    assert predictor.images == []
    masks = segmenter.segment(rgb, [_detection()])

    assert len(masks) == 1
    assert masks[0].detection == _detection()
    assert masks[0].score == pytest.approx(0.88)
    assert np.count_nonzero(masks[0].mask) == 30
    assert predictor.boxes[0][0] == pytest.approx((4.0, 3.0, 10.0, 8.0))
    assert factory_calls == [('sam2-config', str(checkpoint), 'cpu')]

    segmenter.segment(rgb, [_detection()])
    assert len(factory_calls) == 1


def test_sam2_segmenter_reports_missing_checkpoint():
    segmenter = Sam2Segmenter('config', '/missing/sam2.pt')

    with pytest.raises(InspectionFailure) as raised:
        segmenter.segment(
            np.zeros((20, 30, 3), dtype=np.uint8),
            [_detection()],
        )

    assert raised.value.kind == FailureKind.MASK


def test_sam2_segmenter_rejects_wrong_mask_shape(tmp_path):
    checkpoint = tmp_path / 'sam2.pt'
    checkpoint.write_bytes(b'placeholder')
    segmenter = Sam2Segmenter(
        'config',
        str(checkpoint),
        predictor_factory=lambda _config, _checkpoint, _device: _Predictor(
            shape=(5, 5)
        ),
    )

    with pytest.raises(InspectionFailure) as raised:
        segmenter.segment(
            np.zeros((20, 30, 3), dtype=np.uint8),
            [_detection()],
        )

    assert raised.value.kind == FailureKind.MASK


def test_image_and_video_predictors_share_one_model_and_lock(tmp_path):
    checkpoint = tmp_path / 'sam2.pt'
    checkpoint.write_bytes(b'placeholder')
    model = object()
    image_predictor = _Predictor()
    model_calls = []

    def build_model(config, checkpoint_path, device):
        model_calls.append((config, checkpoint_path, device))
        return model

    def build_image(received_model):
        assert received_model is model
        return image_predictor

    shared = Sam2SharedPredictors(
        'sam2-config',
        str(checkpoint),
        'cpu',
        model_factory=build_model,
        image_predictor_factory=build_image,
    )
    segmenter = Sam2Segmenter(
        'sam2-config',
        str(checkpoint),
        'cpu',
        shared_predictors=shared,
    )
    tracker = Sam2ReferenceTracker(
        'sam2-config',
        str(checkpoint),
        'cpu',
        shared_predictors=shared,
    )

    assert model_calls == []
    segmenter.prepare()
    tracker.prepare()
    segmenter.prepare()

    assert segmenter._predictor is image_predictor
    assert tracker._predictor is model
    assert segmenter._lock is tracker._lock
    assert model_calls == [('sam2-config', str(checkpoint), 'cpu')]
