from dataclasses import replace

import numpy as np
import pytest

from cleany_perception.core.models import BoundingBox2D, Detection2D, InspectionFailure
from cleany_perception.core.target_revalidation import match_approved_target


def test_approved_semantics_survive_while_current_detection_evidence_is_used():
    box = BoundingBox2D(1, 1, 19, 19)
    source = Detection2D('eye drops', .8, box, detector_label='bottle', sorting_category='lost_item')
    mask = np.ones((20, 20), dtype=bool)
    fresh = Detection2D('bottle', .6, box, segmentation_mask=mask)
    matched = match_approved_target(source, [fresh], (20, 20), .2)
    assert (matched.label, matched.sorting_category, matched.detector_label) == ('eye drops', 'lost_item', 'bottle')
    assert matched.confidence == .6 and matched.segmentation_mask is mask
    assert fresh.label == 'bottle' and source.confidence == .8


@pytest.mark.parametrize('case', ['missing', 'duplicate', 'class_change', 'moved', 'no_mask', 'empty_mask', 'no_raw_class'])
def test_uncertain_identity_never_falls_back_to_a_different_target(case):
    box = BoundingBox2D(1, 1, 19, 19)
    source = Detection2D('eye drops', .8, box, detector_label='bottle')
    fresh = Detection2D('bottle', .8, box, segmentation_mask=np.ones((20, 20), dtype=bool))
    candidates = [fresh]
    if case == 'missing': candidates = []
    if case == 'duplicate': candidates.append(fresh)
    if case == 'class_change': candidates = [replace(fresh, label='cup')]
    if case == 'moved': candidates = [replace(fresh, bbox=BoundingBox2D(30, 30, 40, 40))]
    if case == 'no_mask': candidates = [replace(fresh, segmentation_mask=None)]
    if case == 'empty_mask': candidates = [replace(fresh, segmentation_mask=np.zeros((20, 20), dtype=bool))]
    if case == 'no_raw_class': source = replace(source, detector_label='')
    with pytest.raises(InspectionFailure):
        match_approved_target(source, candidates, (20, 20), .2)
