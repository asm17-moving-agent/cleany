from types import SimpleNamespace

import numpy as np
import pytest

from cleany_perception.adapters.yoloe_reference_tracker import YoloeReferenceTracker
from cleany_perception.core.models import BoundingBox2D, Detection2D, InspectionFailure


def test_corrected_semantic_label_reassociates_only_with_original_detector_class():
    rgb = np.zeros((20, 20, 3), dtype=np.uint8)
    mask = np.ones((20, 20), dtype=bool)
    box = BoundingBox2D(1, 1, 19, 19)
    source = Detection2D('remote control', .8, box, detector_label='power bank')
    fresh = Detection2D('power bank', .9, box, segmentation_mask=mask)
    tracker = YoloeReferenceTracker(SimpleNamespace(detect=lambda *_: (fresh,)))
    result = tracker.track(rgb, source, rgb)
    np.testing.assert_array_equal(result, mask)
    assert not np.shares_memory(result, mask)
    tracker.detector.detect = lambda *_: (
        Detection2D('remote control', .9, box, segmentation_mask=mask),)
    with pytest.raises(InspectionFailure, match='found 0'):
        tracker.track(rgb, source, rgb)
