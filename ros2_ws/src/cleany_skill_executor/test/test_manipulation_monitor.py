"""Execution evidence, replay ordering and the viewer's binary wire format."""

from dataclasses import replace
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import pytest

from cleany_skill_executor.manipulation.monitor import (
    FAILURE, IDLE, RUNNING, SUCCESS, MonitorProjection, Progress,
)


XML = Path(__file__).parents[1] / 'docs/groot2_table_cleanup.xml'


def state(view, tag):
    return view.statuses[view.by_tag[tag]]


def group_state(view, name):
    node = next(node for node in view.groups if node.get('name') == name)
    return view.statuses[int(node.get('_uid'))]


def test_observation_does_not_claim_stage_completion():
    view = MonitorProjection(XML)
    event = Progress('a', 10, 1, 'GRASPING', 'APPROACHING')
    view.update(event)
    assert state(view, 'ApproachObject') == SUCCESS
    assert state(view, 'GraspObject') == RUNNING
    assert state(view, 'LiftObject') == IDLE
    assert group_state(view, '준비') == SUCCESS
    assert group_state(view, '물체 잡기') == RUNNING
    assert group_state(view, '물체 놓기') == IDLE
    assert group_state(view, '확인과 종료') == IDLE
    view.update(replace(event, revision=2, last_completed_stage='GRASPING'))
    assert state(view, 'GraspObject') == SUCCESS
    assert group_state(view, '물체 잡기') != SUCCESS


@pytest.mark.parametrize('status', ['FAILED', 'BLOCKED', 'CANCELED', 'FATAL'])
def test_final_failure_preserves_progress_and_unexecuted_nodes(status):
    view = MonitorProjection(XML)
    view.update(Progress('a', 10, 20, 'FINALIZING', 'LIFTING', True,
                         status, 'TRANSPORTING', stop_confirmed=status != 'FATAL'))
    assert state(view, 'LiftObject') == SUCCESS
    assert state(view, 'CarryObject') == FAILURE
    assert group_state(view, '물체 잡기') == SUCCESS
    assert group_state(view, '물체 놓기') == FAILURE
    assert group_state(view, '확인과 종료') == IDLE
    assert state(view, 'OpenGripperAtDestination') == IDLE
    assert state(view, 'FinalizeSuccess') == IDLE
    assert state(view, 'StopAndAssess') == (FAILURE if status == 'FATAL' else SUCCESS)
    assert state(view, 'FinalizeFailure') == SUCCESS
    assert view.statuses[view.controls[0]] == FAILURE


def test_success_and_next_execution_reset():
    view = MonitorProjection(XML)
    view.update(Progress('a', 10, 30, 'FINALIZING', 'VERIFYING_PLACEMENT', True, 'SUCCESS'))
    assert state(view, 'FinalizeSuccess') == SUCCESS
    assert state(view, 'StopAndAssess') == IDLE
    assert all(view.statuses[int(group.get('_uid'))] == SUCCESS for group in view.groups)
    view.update(Progress('b', 20, 1, 'VALIDATING'))
    assert state(view, 'ValidateGoal') == RUNNING
    assert state(view, 'FinalizeSuccess') == IDLE
    assert group_state(view, '준비') == RUNNING
    assert group_state(view, '물체 잡기') == IDLE
    assert not view.update(Progress('a', 10, 31, 'FINALIZING', has_result=True, status='SUCCESS'))
    assert not view.update(Progress('b', 20, 1, 'GRASPING'))
    assert not view.update(Progress('c', 30, 1, 'GRASPING', execution_profile='real'))
    assert state(view, 'ValidateGoal') == RUNNING


def test_stopping_and_crash_do_not_invent_stop_evidence():
    view = MonitorProjection(XML)
    view.update(Progress('a', 10, 4, 'STOPPING', 'GRASPING'))
    assert state(view, 'StopAndAssess') == RUNNING
    view.update(Progress('a', 10, 5, 'STOPPING', 'GRASPING', record_state='INTERRUPTED'))
    assert state(view, 'StopAndAssess') == IDLE
    assert state(view, 'FinalizeFailure') == IDLE
    assert view.statuses[view.controls[0]] == FAILURE


def test_finalizing_failure_does_not_flash_success_path():
    view = MonitorProjection(XML)
    view.update(Progress('a', 10, 4, 'FINALIZING', 'APPROACHING'))
    assert state(view, 'FinalizeFailure') == RUNNING
    assert state(view, 'FinalizeSuccess') == IDLE


def test_return_cancel_projection_shows_recovery_and_never_flashes_success():
    view = MonitorProjection(XML)
    view.update(Progress('a', 10, 4, 'RECOVERING_ARM', 'LIFTING',
                         substage='ReturnArmAfterCancel', completed_substages=('ReleaseInPlace',),
                         cancel_mode='RETURN_ARM'))
    assert state(view, 'ReleaseInPlace') == SUCCESS
    assert state(view, 'ReturnArmAfterCancel') == RUNNING
    assert view.statuses[view.controls[1]] == FAILURE
    view.update(Progress('a', 10, 5, 'FINALIZING', 'VERIFYING_PLACEMENT', cancel_mode='CHECKPOINT'))
    assert state(view, 'FinalizeFailure') == RUNNING
    assert state(view, 'FinalizeSuccess') == IDLE


def test_cancel_after_atomic_stage_preserves_completed_stage():
    view = MonitorProjection(XML)
    view.update(Progress('a', 10, 6, 'FINALIZING', 'GRASPING', True,
                         'CANCELED', 'GRASPING', stop_confirmed=True))
    assert state(view, 'GraspObject') == SUCCESS
    assert state(view, 'LiftObject') == IDLE
    assert view.statuses[view.controls[0]] == FAILURE


def test_full_tree_and_status_uids_match_viewer_protocol():
    view = MonitorProjection(XML)
    header = struct.pack('<BBI', 2, ord('T'), 123)
    reply, xml = view.reply([header])
    assert reply == header + view.tree_id
    tree = ET.fromstring(xml).find('BehaviorTree')
    uids = {int(node.get('_uid')) for node in list(tree.iter())[1:]}
    view.update(Progress('a', 10, 1, 'APPROACHING', 'PREPARING_TARGET'))
    header = struct.pack('<BBI', 2, ord('S'), 124)
    reply, payload = view.reply([header])
    statuses = dict(struct.iter_unpack('<HB', payload))
    assert set(statuses) == uids
    assert statuses[view.by_tag['ApproachObject']] == RUNNING
    assert reply[:6] == header
    for frames in ([], [b'bad'], [b'\x02T1234', b'extra'], [b'\x02B1234']):
        assert view.reply(frames)[1] == b''


def test_detailed_feedback_marks_only_observed_substages():
    view = MonitorProjection(XML)
    event = Progress('a', 10, 5, 'GRASPING', 'APPROACHING',
                     substage='ConfirmGrasp', completed_substages=(
                         'MoveToPregrasp', 'ApproachObject', 'GraspObject'))
    view.update(event)
    assert state(view, 'GraspObject') == SUCCESS
    assert state(view, 'ConfirmGrasp') == RUNNING
    assert state(view, 'LiftObject') == IDLE
    assert group_state(view, '물체 잡기') == RUNNING
    view.update(replace(event, revision=6, stage='FINALIZING', substage='',
                        has_result=True, status='FAILED', failed_stage='GRASPING',
                        failed_substage='ConfirmGrasp', stop_confirmed=True))
    assert state(view, 'GraspObject') == SUCCESS
    assert state(view, 'ConfirmGrasp') == FAILURE
    assert state(view, 'LiftObject') == IDLE


def test_final_success_does_not_invent_missing_release_evidence():
    view = MonitorProjection(XML)
    view.update(Progress('a', 10, 40, 'FINALIZING', 'VERIFYING_PLACEMENT',
                         has_result=True, status='SUCCESS', completed_substages=(
                             'OpenGripperAtDestination', 'ReturnArm', 'VerifyPlacedObject')))
    assert state(view, 'ConfirmRelease') == IDLE
    assert state(view, 'VerifyPlacedObject') == SUCCESS
    assert state(view, 'FinalizeSuccess') == SUCCESS
