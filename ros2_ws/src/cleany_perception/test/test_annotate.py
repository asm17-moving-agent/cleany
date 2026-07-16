import numpy as np
import pytest

from cleany_perception.annotate import draw_detections
from cleany_perception.detector import Detection

pytest.importorskip('cv2')  # annotation is optional; skip if OpenCV is absent


def test_draw_detections_returns_same_shape_copy_without_mutating_input():
    image = np.zeros((20, 30, 3), dtype=np.uint8)
    dets = [Detection(label='cup', score=0.9, x1=2.0, y1=3.0, x2=15.0, y2=18.0)]

    out = draw_detections(image, dets)

    assert out.shape == image.shape
    assert out.dtype == np.uint8
    # input untouched
    assert np.count_nonzero(image) == 0
    # something was drawn
    assert np.count_nonzero(out) > 0


def test_draw_detections_overlays_mask_region():
    image = np.zeros((20, 30, 3), dtype=np.uint8)
    mask = np.zeros((20, 30), dtype=bool)
    mask[10:15, 20:25] = True
    dets = [Detection(label='cup', score=0.9, x1=2.0, y1=3.0, x2=15.0, y2=18.0, mask=mask)]

    out = draw_detections(image, dets)

    # mask region got tinted (green channel raised) without touching the input
    assert out[12, 22, 1] > 0
    assert np.count_nonzero(image) == 0


def test_draw_detections_ignores_mask_with_mismatched_shape():
    image = np.zeros((20, 30, 3), dtype=np.uint8)
    dets = [
        Detection(
            label='cup', score=0.9, x1=2.0, y1=3.0, x2=15.0, y2=18.0,
            mask=np.ones((5, 5), dtype=bool),
        )
    ]

    out = draw_detections(image, dets)  # must not raise

    assert out.shape == image.shape


def test_draw_detections_empty_is_noop_copy():
    image = np.zeros((8, 8, 3), dtype=np.uint8)

    out = draw_detections(image, [])

    assert out.shape == image.shape
    assert np.count_nonzero(out) == 0
