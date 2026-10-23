"""Replay semantic corrections across perception, tracking and approval gates."""
from dataclasses import replace
import json

import pytest
from rclpy.serialization import deserialize_message, serialize_message

from cleany_dev_monitor.proposals import DEFAULTS, fingerprint, validate_fresh_target
from cleany_perception.adapters.gemini_detector import parse_gemini_classifications
from cleany_perception.core.models import BoundingBox2D, Detection2D
from cleany_perception.core.tracking import ObjectTracker
from cleany_perception.inspection_node import InspectionNode
from cleany_skill_executor.core.automatic_goal import select_automatic_target


CONFIG = {'label_destinations': {'remote control': 'return_to_pick', 'power bank': 'wrong_bin'}}


def scene(tracker, snapshot_id, semantic_label, *, count=1, category='lost_item'):
    raw = Detection2D('power bank', .9, BoundingBox2D(10., 10., 70., 90.))
    payload = json.dumps({'objects': [dict(id=i+1, label=semantic_label,
        sorting_category=category) for i in range(count)]})
    detections = parse_gemini_classifications(payload, (raw,) * count)
    detections = [replace(d, median_depth_m=.4, estimated_width_m=.05,
                           estimated_height_m=.07) for d in detections]
    tracking = tracker.update('desk', snapshot_id, [d.label for d in detections],
        [(.4, 0., .2)] * count, detector_labels=[d.detector_label for d in detections])
    message = InspectionNode._detections_message(detections, 10**9, 'camera', snapshot_id,
                                                  [.4] * count, tracking)
    return deserialize_message(serialize_message(message), type(message))


def approved(snapshot):
    target = snapshot.detections[0]
    selected = select_automatic_target(snapshot, CONFIG)
    assert selected.destination_id == 'return_to_pick'
    return dict(created_at=99., expires_at=200., config_fingerprint=fingerprint(CONFIG),
        tracking_session_id=snapshot.tracking_session_id, tracking_epoch=snapshot.tracking_epoch,
        representative_frame='base_link', skill_name=selected.skill_name,
        destination_id=selected.destination_id,
        target=dict(track_id=target.track_id, label=target.label, category=target.sorting_category,
                    position=[.4, 0., .2], bbox=[10., 10., 70., 90.]))


def test_corrected_type_and_alias_keep_identity_and_approved_semantic_destination():
    tracker = ObjectTracker()
    first = scene(tracker, 'first', 'remote control')
    proposal = approved(first)
    fresh = scene(tracker, 'second', 'Remote')
    assert fresh.detections[0].label == 'remote control'
    assert fresh.detections[0].detector_label == 'power bank'
    assert fresh.detections[0].track_id == first.detections[0].track_id
    validate_fresh_target(proposal, fresh, CONFIG, DEFAULTS, 100.)


@pytest.mark.parametrize('label,category,count', [
    ('bottle', 'lost_item', 1), ('remote control', 'trash', 1),
    ('remote control', 'lost_item', 2),
])
def test_approval_still_rejects_changed_semantics_and_ambiguous_identity(label, category, count):
    tracker = ObjectTracker()
    proposal = approved(scene(tracker, 'first', 'remote control'))
    fresh = scene(tracker, 'second', label, category=category, count=count)
    with pytest.raises(ValueError):
        validate_fresh_target(proposal, fresh, CONFIG, DEFAULTS, 100.)


@pytest.mark.parametrize('label', ['battery', 'knife', 'broken glass', 'unknown'])
def test_corrected_excluded_or_unknown_type_cannot_use_original_detector_permission(label):
    observed = scene(ObjectTracker(), 'first', label)
    with pytest.raises(ValueError, match='No classified'):
        select_automatic_target(observed, {'label_destinations': {'*': 'return_to_pick'}})
