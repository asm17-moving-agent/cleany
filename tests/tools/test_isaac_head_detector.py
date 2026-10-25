import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'tools/validation/isaac'))
from head_detector import HeadDetector
from cleany_perception.core.models import BoundingBox2D, Detection2D


@pytest.mark.parametrize('turns', range(4))
def test_all_quarter_turns_restore_mask_and_box_on_odd_sensor_dimensions(turns):
    rgb = np.arange(13*19*3, dtype=np.uint8).reshape(13, 19, 3)
    tile = rgb[6:, 9:]
    mask = np.zeros(tile.shape[:2], dtype=bool)
    mask[2:5, 3:7] = True
    rotated_mask = np.rot90(mask, turns)
    ys, xs = np.where(rotated_mask)
    detection = Detection2D('cup', .61,
        BoundingBox2D(float(xs.min()), float(ys.min()), float(xs.max()+1), float(ys.max()+1)),
        segmentation_mask=rotated_mask)

    class Detector:
        def detect(self, image, query):
            assert query == 'cup'
            return (detection,) if np.array_equal(image, np.rot90(tile, turns)) else ()

    result, = HeadDetector(Detector()).detect(rgb, 'cup')
    expected = np.zeros(rgb.shape[:2], dtype=bool)
    expected[8:11, 12:16] = True
    assert result.bbox == BoundingBox2D(12., 8., 16., 11.)
    assert np.array_equal(result.segmentation_mask, expected)
    assert not result.segmentation_mask.flags.writeable
    assert result.confidence == .61


def test_preserves_original_detections_without_extra_inference():
    detection = Detection2D('cup', .3, BoundingBox2D(1, 1, 3, 3))
    class Detector:
        calls = 0
        def detect(self, image, query):
            self.calls += 1
            return (detection,)
    detector = Detector()
    assert HeadDetector(detector).detect(np.zeros((8, 10, 3)), '') == (detection,)
    assert detector.calls == 1


@pytest.mark.parametrize('touch_edge', [False, True])
def test_deduplicates_repeated_rotation_masks_and_rejects_clipped_instances(touch_edge):
    class Detector:
        calls = 0
        def detect(self, image, query):
            self.calls += 1
            if self.calls == 1:
                return ()
            turns = (self.calls-2) % 4
            mask = np.zeros((8, 8), dtype=bool)
            mask[0 if touch_edge else 2:6, 2:6] = True
            return (Detection2D('cup', .5, BoundingBox2D(2, 2, 6, 6),
                segmentation_mask=np.rot90(mask, turns)),)
    detector = Detector()
    results = HeadDetector(detector).detect(np.zeros((16, 16, 3)), '')
    assert len(results) == (0 if touch_edge else 4)
    assert detector.calls == 17
    assert all(item.confidence == .5 for item in results)
