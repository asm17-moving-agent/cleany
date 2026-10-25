import sys
from pathlib import Path

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools/validation/isaac'))
from wrist_detector import WristDetector
from cleany_perception.core.models import Detection2D, BoundingBox2D


def test_half_turn_restores_bbox_and_mask_to_original_sensor_pixels():
    rgb = np.arange(6*10*3,dtype=np.uint8).reshape(6,10,3)
    mask = np.zeros((6,10),dtype=bool)
    mask[1:3,2:5] = True
    detection = Detection2D('cup',.56,BoundingBox2D(2,1,5,3),segmentation_mask=mask)

    class Detector:
        calls = 0
        def detect(self, image, query):
            self.calls += 1
            assert query == 'target'
            if self.calls == 1:
                assert np.array_equal(image,rgb)
                return ()
            assert np.array_equal(image,rgb[::-1,::-1])
            return (detection,)

    detector = Detector()
    result, = WristDetector(detector).detect(rgb,'target')
    assert detector.calls == 2
    assert result.bbox == BoundingBox2D(5,3,8,5)
    assert np.array_equal(result.segmentation_mask,mask[::-1,::-1])
    assert result.confidence == .56 and result.label == 'cup'


def test_original_detection_is_not_replaced_or_given_extra_confidence():
    detection = Detection2D('cup',.3,BoundingBox2D(1,1,3,3))
    class Detector:
        calls = 0
        def detect(self, image, query):
            self.calls += 1
            return (detection,)
    detector = Detector()
    assert WristDetector(detector).detect(np.zeros((4,4,3)),'') == (detection,)
    assert detector.calls == 1
