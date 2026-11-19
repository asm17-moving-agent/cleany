import numpy as np
import pytest

from cleany_perception.adapters.simulation_color import (
    SimulationColorDetector,
    SimulationColorSegmenter,
)


def test_simulation_color_adapter_measures_rendered_masks_and_bboxes():
    rgb = np.full((80, 100, 3), 40, dtype=np.uint8)
    rgb[10:30, 15:35] = (220, 30, 20)
    rgb[40:70, 60:90] = (20, 50, 230)
    detector = SimulationColorDetector(minimum_pixels=100)

    detections = detector.detect(rgb, 'ignored simulation query')
    masks = SimulationColorSegmenter().segment(rgb, detections)

    assert [item.label for item in detections] == ['red_can', 'blue_box']
    assert (
        detections[0].bbox.x_min,
        detections[0].bbox.y_min,
        detections[0].bbox.x_max,
        detections[0].bbox.y_max,
    ) == (15.0, 10.0, 35.0, 30.0)
    assert [int(item.mask.sum()) for item in masks] == [400, 900]


def test_simulation_color_detector_ignores_small_regions():
    rgb = np.zeros((20, 20, 3), dtype=np.uint8)
    rgb[1:3, 1:3] = (255, 0, 0)

    assert SimulationColorDetector(minimum_pixels=5).detect(rgb, '') == ()


def test_study_cafe_profile_detects_and_segments_all_tabletop_objects():
    rgb = np.full((120, 160, 3), 240, dtype=np.uint8)
    expected = {
        'cup': ((10, 10, 40, 40), (15, 52, 112)),
        'wallet': ((50, 10, 80, 40), (99, 40, 15)),
        'phone': ((90, 10, 120, 40), (12, 14, 16)),
        # The raised rubber body is this dark under the head light.  Channel
        # ratios, rather than brightness, keep it distinct from the phone.
        'box': ((125, 10, 155, 40), (7, 1, 2)),
    }
    for (x_min, y_min, x_max, y_max), color in expected.values():
        rgb[y_min:y_max, x_min:x_max] = color

    detector = SimulationColorDetector(
        minimum_pixels=100,
        profile='study_cafe',
    )
    detections = detector.detect(rgb, 'ignored simulation query')
    masks = SimulationColorSegmenter(profile='study_cafe').segment(
        rgb,
        detections,
    )

    assert [item.label for item in detections] == list(expected)
    assert [int(item.mask.sum()) for item in masks] == [900] * 4


def test_simulation_color_adapter_rejects_unknown_profile():
    with pytest.raises(ValueError, match='unsupported'):
        SimulationColorDetector(profile='unknown')


def test_simulation_color_segmenter_rejects_unknown_label(synthetic_scene):
    with pytest.raises(ValueError, match='unsupported'):
        SimulationColorSegmenter().segment(
            synthetic_scene['snapshot'].rgb,
            (synthetic_scene['detection'],),
        )
