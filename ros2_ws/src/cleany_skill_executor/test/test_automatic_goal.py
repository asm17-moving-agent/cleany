from types import SimpleNamespace as NS

import pytest

from cleany_skill_executor.core.automatic_goal import select_automatic_target


def detection(identifier, depth, **fields):
    return NS(**dict(dict(object_id=identifier, label='cup', confidence=.9,
        sorting_category='trash', sorting_reason='disposable', distance_valid=True,
        distance_m=depth, median_depth_m=depth, estimated_width_m=.07,
        estimated_height_m=.1), **fields))


def test_auto_selection_preserves_real_snapshot_and_chooses_eligible_target():
    snapshot = NS(snapshot_id='camera-capture-423', detections=[
        detection(1, .1, label='battery'), detection(2, .2, sorting_reason=''),
        detection(3, .3, distance_valid=False), detection(4, .5),
        detection(7, .4, sorting_category='lost_item')])
    target = select_automatic_target(snapshot, {'label_destinations': {'*': 'return_to_pick'}})
    assert (target.snapshot_id, target.object_id, target.skill_name, target.destination_id) == (
        'camera-capture-423', 7, 'collect_lost_item', 'return_to_pick')


def test_40mm_footprint_is_selectable_even_with_a_large_diagonal_image_region():
    target = detection(1, .5, label='box', sorting_category='lost_item',
        estimated_width_m=.040, estimated_height_m=.160,
        x_min=381.5, y_min=191.5, x_max=609.5, y_max=452.)
    selected = select_automatic_target(NS(snapshot_id='real', detections=[target]),
        {'label_destinations': {'*': 'return_to_pick'}}, maximum_smaller_extent_m=.11)
    assert selected.object_id == 1 and selected.skill_name == 'collect_lost_item'


@pytest.mark.parametrize('fields', [dict(confidence=float('nan')), dict(confidence=.1),
    dict(sorting_category='review'), dict(median_depth_m=0.), dict(estimated_width_m=0.),
    dict(distance_m=float('nan')), dict(object_id=0)])
def test_uncertain_detections_never_create_a_goal(fields):
    with pytest.raises(ValueError, match='No classified'):
        select_automatic_target(NS(snapshot_id='real', detections=[detection(1, .4, **fields)]),
                                {'label_destinations': {'*': 'return_to_pick'}})


def test_missing_destination_or_snapshot_does_not_invent_identifiers():
    with pytest.raises(ValueError):
        select_automatic_target(NS(snapshot_id='', detections=[detection(1, .4)]), {})
    with pytest.raises(ValueError):
        select_automatic_target(NS(snapshot_id='real', detections=[detection(1, .4)]), {})
