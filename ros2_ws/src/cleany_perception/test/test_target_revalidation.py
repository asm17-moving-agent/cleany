from dataclasses import replace

import numpy as np
import pytest

from cleany_perception.core.models import BoundingBox2D, Detection2D, InspectionFailure
from cleany_perception.core.target_revalidation import match_approved_target


def test_approved_semantics_survive_while_current_detection_evidence_is_used():
    box = BoundingBox2D(1, 1, 19, 19)
    source = Detection2D('eye drops', .8, box, detector_label='bottle', sorting_category='lost_item', sorting_reason='personal item')
    mask = np.ones((20, 20), dtype=bool)
    fresh = Detection2D('bottle', .6, box, segmentation_mask=mask)
    matched = match_approved_target(source, [fresh], (20, 20), .2)
    assert (matched.label, matched.sorting_category, matched.detector_label) == ('eye drops', 'lost_item', 'bottle')
    assert matched.sorting_reason == 'personal item'
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


@pytest.mark.parametrize('case', ['reordered', 'same_track_twice', 'depth_missing', 'wrong_mask',
                                  'unknown_reference', 'two_nearby', 'competitor_only'])
def test_overlapping_raw_boxes_require_a_distinct_known_track_for_each_region(case):
    from cleany_perception.core.object_ranking import RankedDetection
    from cleany_perception.core.target_revalidation import identify_tracked_candidate
    from cleany_perception.core.tracking import TrackedDetection, TrackingState

    # Proposal #2 lies inside a much larger detection of the same raw class.
    large = BoundingBox2D(0, 0, 80, 50)
    small = BoundingBox2D(15, 7, 42, 42)
    outer_mask = np.ones((50, 80), dtype=bool)
    inner_mask = np.zeros_like(outer_mask)
    inner_mask[7:42, 15:42] = True
    outer_mask[inner_mask] = False
    refs = [Detection2D('plastic box', .8, large, detector_label='box', segmentation_mask=outer_mask),
            Detection2D('cardboard box', .4, small, detector_label='box', segmentation_mask=inner_mask)]
    tracks = [TrackedDetection('outer', TrackingState.TRACKED, (.35, .03, .36)),
              TrackedDetection('approved', TrackingState.TRACKED, (.34, .07, .44))]
    fresh = [RankedDetection(replace(refs[i], label='box'), .55, i, tracks[i].position) for i in (1, 0)]
    if case == 'same_track_twice': fresh[1] = fresh[0]
    if case == 'depth_missing': fresh[1] = replace(fresh[1], position=None)
    if case == 'wrong_mask': fresh[0] = replace(fresh[0], detection=replace(fresh[0].detection, segmentation_mask=outer_mask))
    if case == 'unknown_reference': tracks[0] = replace(tracks[0], state=TrackingState.AMBIGUOUS)
    if case == 'two_nearby': tracks[0] = replace(tracks[0], position=tracks[1].position)
    if case == 'competitor_only': fresh = [fresh[1]]
    chosen = identify_tracked_candidate(fresh, refs, tracks, 'approved', outer_mask.shape, .2, .03)
    if case == 'reordered':
        assert chosen is fresh[0].detection
    else:
        assert chosen is None
