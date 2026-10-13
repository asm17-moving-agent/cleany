import pytest
from cleany_perception.core.tracking import ObjectTracker, TrackingConfig, TrackingState as S


def observe(tracker, snapshot, points, labels=None, session='desk'):
    return tracker.update(session, snapshot, labels or ['cup'] * len(points), points)


def test_two_cups_keep_identity_despite_order_number_and_position_changes():
    tracker = ObjectTracker()
    first = observe(tracker, '1', [(.5, .1, .7), (.6, -.1, .7)])
    second = observe(tracker, '2', [(.601, -.1, .7), (.502, .1, .7)])
    assert second.epoch == first.epoch
    assert [d.track_id for d in second.detections] == [d.track_id for d in first.detections[::-1]]
    assert not second.missing
    assert observe(tracker, '2', []).detections == second.detections


def test_close_candidates_and_large_motion_do_not_mint_fresh_ids():
    tracker = ObjectTracker()
    first = observe(tracker, '1', [(0, 0, .7), (.1, 0, .7)])
    ambiguous = observe(tracker, '2', [(.015, 0, .7), (.02, 0, .7)])
    assert all(d.state == S.AMBIGUOUS and not d.track_id for d in ambiguous.detections)
    moved = observe(tracker, '3', [(.3, 0, .7)])
    assert moved.detections[0].state == S.AMBIGUOUS
    assert {r.track_id for r in moved.missing} == {d.track_id for d in first.detections}


def test_occlusion_near_return_and_missing_references():
    tracker = ObjectTracker()
    first = observe(tracker, '1', [(0, 0, .7)])
    missing = observe(tracker, '2', [])
    assert missing.missing[0].snapshot_id == '1'
    assert observe(tracker, '3', [(.005, 0, .7)]).detections[0].track_id == first.detections[0].track_id


def test_invalid_depth_retains_old_identity_and_unresolved_new_object():
    tracker = ObjectTracker()
    first = observe(tracker, '1', [(0, 0, .7)])
    invalid = observe(tracker, '2', [None])
    assert invalid.detections[0].state == S.INVALID
    assert invalid.missing[0].track_id == first.detections[0].track_id
    assert observe(tracker, '3', [(.3, 0, .7)]).detections[0].state == S.AMBIGUOUS
    tracker = ObjectTracker()
    observe(tracker, '1', [None])
    assert observe(tracker, '2', [(0, 0, .7)]).detections[0].state == S.AMBIGUOUS


def test_epochs_change_after_expiry_restart_and_session_change():
    now = [0.]
    tracker = ObjectTracker(TrackingConfig(session_ttl_seconds=1.), lambda: now[0])
    first = observe(tracker, '1', [(0, 0, .7)])
    now[0] = 2.
    assert observe(tracker, '2', [(0, 0, .7)]).epoch != first.epoch
    assert observe(ObjectTracker(), '1', [(0, 0, .7)]).epoch != first.epoch
    assert observe(tracker, '3', [(0, 0, .7)], session='new').epoch != first.epoch
    assert observe(tracker, '4', [(0, 0, .7)], session='').detections[0].state == S.UNTRACKED


def test_types_and_explicit_aliases_are_separate_from_category():
    tracker = ObjectTracker()
    first = observe(tracker, '1', [(0, 0, .7)], ['paper cup'])
    second = observe(tracker, '2', [(0, 0, .7)], ['cup'])
    assert second.detections[0].track_id == first.detections[0].track_id
    assert observe(tracker, '3', [(0, 0, .7)], ['mouse']).detections[0].track_id != first.detections[0].track_id


def test_disappeared_initial_invalid_detection_remains_unresolved():
    tracker = ObjectTracker()
    observe(tracker, 'invalid', [None])
    missing = observe(tracker, 'empty', []).missing
    assert len(missing) == 1 and missing[0].snapshot_id == 'invalid'
    assert missing[0].track_id == ''


def test_unmatched_candidate_near_an_already_matched_track_does_not_get_new_id():
    tracker = ObjectTracker()
    first = observe(tracker, '1', [(0., 0., .7)])
    second = observe(tracker, '2', [(0., 0., .7), (.02, 0., .7)])
    assert second.detections[0].track_id == first.detections[0].track_id
    assert second.detections[1].state == S.AMBIGUOUS
@pytest.mark.parametrize('alias', ['mouse', 'computer mouse', 'wireless mouse'])
def test_mouse_aliases_keep_track_across_renumbering(alias):
    tracker = ObjectTracker()
    first = tracker.update('desk', 'first', ['computer mouse', 'lego brick'], [(0.3, 0., 0.), (0.5, 0., 0.)])
    second = tracker.update('desk', 'second', ['lego brick', alias], [(0.5, 0., 0.), (0.3, 0., 0.)])
    assert second.detections[1].track_id == first.detections[0].track_id
    assert second.detections[0].track_id == first.detections[1].track_id
