"""Physical progress, failure, cancellation and crash recovery with a fake clock."""

from dataclasses import replace
from pathlib import Path
import sqlite3
from concurrent.futures import ThreadPoolExecutor

import pytest

from cleany_skill_executor.manipulation.core import ATOMIC_STAGES, NORMAL_STAGES, ExecutionCore
from cleany_skill_executor.manipulation.mock import MockAdapter, MockConfig
from cleany_skill_executor.manipulation.models import (
    CancelMode, Error, Goal, ObjectState, Placement, Record, RecordState, Stage, Status,
)
from cleany_skill_executor.manipulation.store import (
    DuplicateExecution, ExecutionStore, StoreError, default_database_path,
)
from cleany_skill_executor.manipulation.steps import STAGE_STEPS


CONFIG_PATH = Path(__file__).parents[1] / 'config/manipulation_mock.yaml'


class FakeClock:
    now = 0.0

    def __call__(self):
        return self.now

    def wall_ns(self):
        return 1_000_000_000 + int(self.now * 1_000_000_000)


def goal(execution_id='execution-1', **kwargs):
    return replace(Goal('mission', 'task', execution_id, 'collect_trash',
                        'mock-snapshot-001', 1, 'mock_trash_bin'), **kwargs)


@pytest.mark.parametrize('destination,expected', [
    ('mock_lost_item_bin', Error.NONE), ('mock_trash_bin', Error.DESTINATION_UNAVAILABLE),
])
def test_lost_item_goal_and_mock_destination_contract(destination, expected):
    approved = goal(skill_name='collect_lost_item', destination_id=destination)
    assert approved.valid()
    port = MockAdapter(MockConfig())
    port.begin(Stage.PREPARING_TARGET, approved, 0.)
    assert port._stage_evidence(1.).error == expected


@pytest.fixture
def harness(tmp_path):
    stores = []

    def create(scenario='success', *, config=None, path=None):
        clock = FakeClock()
        config = config or MockConfig.load(str(CONFIG_PATH))
        store = ExecutionStore(str(path or tmp_path / f'{len(stores)}.sqlite3'))
        stores.append(store)
        port = MockAdapter(config, scenario)
        core = ExecutionCore(port, store, config.timeout, clock=clock, wall_clock_ns=clock.wall_ns)
        return core, port, store, clock

    yield create
    for store in stores:
        store.close()


def step(core, clock, duration=0.05):
    clock.now += duration
    core.tick()


def until(core, clock, predicate):
    for _ in range(1000):
        if predicate():
            return
        step(core, clock)
    raise AssertionError(f'Execution did not reach condition: {core.record}')


def finish(core, clock):
    until(core, clock, lambda: core.record.result is not None)
    return core.record.result


def reach(core, port, clock, stage):
    until(core, clock, lambda: port.stage == stage)


def test_success_requires_bin_recovery_stop_and_complete_feedback_sequence(harness):
    core, port, store, clock = harness()
    assert core.accept(goal())[0]
    result = finish(core, clock)
    assert result.status == Status.SUCCESS
    assert result.execution_profile == 'mock'
    assert result.error_code == Error.NONE and result.failed_stage == ''
    assert result.last_completed_stage == Stage.VERIFYING_PLACEMENT.value
    assert result.object_state == ObjectState.LEFT_GRIPPER
    assert result.placement_state == Placement.CONFIRMED
    assert result.selected_arm == 'left'
    assert result.stop_confirmed and result.arm_recovered and not result.retryable
    assert port.commands == list(NORMAL_STAGES)
    events = core.drain_events()
    stages = list(dict.fromkeys(event.stage for event in events))
    assert stages == list(NORMAL_STAGES) + [Stage.FINALIZING]
    grasp_messages = [event.message for event in events if event.stage == Stage.GRASPING]
    assert grasp_messages == [
        'Starting GRASPING',
        'Substage completed: GraspObject; starting ConfirmGrasp',
        'Observation received: Mock contact observation',
        'Stage completed: GRASPING',
    ]
    final_messages = [event.message for event in events if event.stage == Stage.FINALIZING]
    assert final_messages == [
        'Finalizing result: Mock collection verified',
        'Execution finished: SUCCESS; Mock collection verified',
    ]
    assert result.message == 'Mock collection verified'
    assert core.record.completed_substages == tuple(
        step.node for stage in NORMAL_STAGES for step in STAGE_STEPS[stage])
    assert store.get(goal().execution_id) == core.record
    assert core.get('missing') is None
    assert not core.request_cancel(goal().execution_id)
    assert core.accept(goal('execution-2'))[0]


def test_gripper_command_completion_does_not_claim_held_state(harness):
    core, port, store, clock = harness()
    core.accept(goal())
    reach(core, port, clock, Stage.GRASPING)
    until(core, clock, lambda: core.record.substage == 'ConfirmGrasp')
    record = store.get(goal().execution_id)
    assert 'GraspObject' in record.completed_substages
    assert 'ConfirmGrasp' not in record.completed_substages
    assert record.object_state == ObjectState.NOT_TOUCHED
    assert record.last_completed_stage == Stage.APPROACHING.value
    until(core, clock, lambda: core.record.last_completed_stage == Stage.GRASPING.value)
    assert core.record.object_state == ObjectState.HELD


def test_substage_failure_stops_remaining_operations_and_survives_lookup(harness):
    config = replace(MockConfig(), scenarios={
        'success': {'substage_errors': {'GraspObject': 'GRASP_FAILED'}}})
    core, port, store, clock = harness(config=config)
    core.accept(goal())
    result = finish(core, clock)
    assert result.status == Status.FAILED and result.failed_substage == 'GraspObject'
    assert 'GraspObject' not in core.record.completed_substages
    assert 'ConfirmGrasp' not in core.record.completed_substages
    assert Stage.LIFTING not in port.commands
    assert store.get(goal().execution_id).result.failed_substage == 'GraspObject'


def test_legacy_record_without_substages_can_still_be_loaded():
    record = Record(goal())
    data = record.to_dict()
    for name in ('substage', 'completed_substages', 'failed_substage'):
        data.pop(name)
    assert Record.from_dict(data) == record


@pytest.mark.parametrize('changes', [
    {'mission_id': ''}, {'task_id': ' '}, {'execution_id': ''}, {'snapshot_id': ''},
    {'destination_id': ''}, {'object_id': 0}, {'skill_name': 'pick'},
])
def test_invalid_request_never_creates_record_or_motion(harness, changes):
    core, port, store, _ = harness()
    assert not core.accept(goal(**changes))[0]
    assert store.all_records() == [] and port.commands == []


@pytest.mark.parametrize('input_goal,error', [
    (goal(snapshot_id='absent'), Error.TARGET_UNAVAILABLE),
    (goal(object_id=99), Error.TARGET_UNAVAILABLE),
    (goal(snapshot_id='mock-stale-snapshot'), Error.STALE_TARGET),
    (goal(destination_id='absent'), Error.DESTINATION_UNAVAILABLE),
])
def test_target_preparation_blocks_without_motion(harness, input_goal, error):
    core, port, _, clock = harness()
    assert core.accept(input_goal)[0]
    result = finish(core, clock)
    assert result.status == Status.BLOCKED and result.error_code == error
    assert result.object_state == ObjectState.NOT_TOUCHED
    assert result.last_completed_stage == Stage.VALIDATING.value
    assert result.placement_state == Placement.NOT_CHECKED
    assert port.commands == [Stage.VALIDATING, Stage.PREPARING_TARGET]
    assert core.record.failed_substage == 'PrepareTarget'
    assert 'ReconstructTarget' not in core.record.completed_substages


@pytest.mark.parametrize('scenario,error,stop', [
    ('backend_not_ready', Error.BACKEND_NOT_READY, False),
    ('verification_unavailable', Error.VERIFICATION_UNAVAILABLE, True),
])
def test_readiness_failure_has_no_fabricated_stop_evidence(harness, scenario, error, stop):
    core, port, _, clock = harness(scenario)
    core.accept(goal())
    result = finish(core, clock)
    assert result.status == Status.BLOCKED and result.error_code == error
    assert result.stop_confirmed == stop
    assert result.retryable == stop
    assert port.commands == [Stage.VALIDATING]


@pytest.mark.parametrize('scenario,error,state,placement,last,forbidden', [
    ('grasp_failure', Error.GRASP_FAILED, ObjectState.UNKNOWN, Placement.NOT_CHECKED,
     Stage.APPROACHING, Stage.LIFTING),
    ('grasp_lost', Error.GRASP_LOST, ObjectState.UNKNOWN, Placement.NOT_CHECKED,
     Stage.LIFTING, Stage.PLACING),
    ('placement_failure', Error.PLACEMENT_NOT_CONFIRMED, ObjectState.LEFT_GRIPPER,
     Placement.NOT_CONFIRMED, Stage.RETURNING_ARM, None),
    ('verification_timeout', Error.VERIFICATION_TIMEOUT, ObjectState.LEFT_GRIPPER,
     Placement.UNKNOWN, Stage.RETURNING_ARM, None),
    ('return_failure', Error.MOTION_FAILED, ObjectState.LEFT_GRIPPER, Placement.CONFIRMED,
     Stage.PLACING, Stage.VERIFYING_PLACEMENT),
])
def test_failures_preserve_physical_progress(harness, scenario, error, state, placement, last, forbidden):
    core, port, store, clock = harness(scenario)
    core.accept(goal())
    result = finish(core, clock)
    assert result.status == Status.FAILED and result.error_code == error
    assert result.object_state == state and result.placement_state == placement
    assert result.last_completed_stage == last.value
    assert result.stop_confirmed and not result.retryable
    assert forbidden not in port.commands
    assert store.get(goal().execution_id).result == result
    if state == ObjectState.UNKNOWN:
        assert not core.accept(goal('new'))[0]


def test_release_command_alone_is_unknown_until_independent_observation(harness):
    core, port, store, clock = harness('release_unobserved')
    core.accept(goal())
    reach(core, port, clock, Stage.RETURNING_ARM)
    assert store.get(goal().execution_id).object_state == ObjectState.UNKNOWN
    assert store.get(goal().execution_id).placement_state == Placement.NOT_CHECKED
    assert 'OpenGripperAtDestination' in core.record.completed_substages
    assert 'ConfirmRelease' not in core.record.completed_substages
    assert finish(core, clock).status == Status.SUCCESS


@pytest.mark.parametrize('stage', NORMAL_STAGES)
def test_cancel_every_stage_prevents_next_motion_and_preserves_checkpoint(harness, stage):
    core, port, store, clock = harness()
    core.accept(goal())
    reach(core, port, clock, stage)
    count = len(port.commands)
    assert core.request_cancel(goal().execution_id)
    step(core, clock)
    if stage in ATOMIC_STAGES:
        assert core.record.result is None and port.stage == stage
    result = finish(core, clock)
    assert result.status == Status.CANCELED and result.error_code == Error.CANCELED
    assert result.failed_stage == stage.value and result.stop_confirmed
    assert not result.retryable
    assert port.commands[count:] == [Stage.STOPPING]
    expected = (ObjectState.HELD if stage in (Stage.GRASPING, Stage.LIFTING, Stage.TRANSPORTING)
                else ObjectState.LEFT_GRIPPER if stage in (
                    Stage.PLACING, Stage.RETURNING_ARM, Stage.VERIFYING_PLACEMENT)
                else ObjectState.NOT_TOUCHED)
    assert result.object_state == expected
    assert store.get(goal().execution_id).result == result
    if expected == ObjectState.HELD:
        assert not core.accept(goal('new'))[0]


@pytest.mark.parametrize('stage', [Stage.GRASPING, Stage.TRANSPORTING, Stage.APPROACHING])
def test_return_cancel_releases_in_place_then_recovers_without_bin_verification(harness, stage):
    core, port, store, clock = harness()
    core.accept(goal())
    reach(core, port, clock, stage)
    count = len(port.commands)
    assert core.request_cancel(goal().execution_id, CancelMode.RETURN_ARM)
    result = finish(core, clock)
    release = [] if stage == Stage.APPROACHING else [Stage.RELEASING_IN_PLACE]
    assert port.commands[count:] == [Stage.STOPPING, *release, Stage.RECOVERING_ARM, Stage.STOPPING]
    assert result.status == Status.CANCELED and result.cancel_mode == 'RETURN_ARM'
    assert result.arm_recovered and result.stop_confirmed
    assert result.object_state == (ObjectState.NOT_TOUCHED if not release else ObjectState.LEFT_GRIPPER)
    assert result.placement_state == Placement.NOT_CHECKED
    assert store.get(goal().execution_id).result == result
    assert core.accept(goal('new'))[0]


def test_immediate_cancel_does_not_wait_for_atomic_completion(harness):
    core, port, _, clock = harness()
    core.accept(goal())
    reach(core, port, clock, Stage.GRASPING)
    assert core.request_cancel(goal().execution_id, CancelMode.IMMEDIATE)
    core.tick()
    assert port.stage == Stage.STOPPING
    result = finish(core, clock)
    assert result.cancel_mode == 'IMMEDIATE' and result.status == Status.CANCELED
    assert result.object_state == ObjectState.UNKNOWN and not result.arm_recovered
    assert Stage.LIFTING not in port.commands


def test_checkpoint_cancel_completes_current_transport_before_stopping(harness):
    core, port, _, clock = harness()
    core.accept(goal())
    reach(core, port, clock, Stage.TRANSPORTING)
    assert core.request_cancel(goal().execution_id, CancelMode.CHECKPOINT)
    core.tick()
    assert port.stage == Stage.TRANSPORTING
    result = finish(core, clock)
    assert result.last_completed_stage == Stage.TRANSPORTING.value
    assert result.object_state == ObjectState.HELD and result.cancel_mode == 'CHECKPOINT'
    assert Stage.PLACING not in port.commands


def test_immediate_cancel_preempts_return_without_downgrade(harness):
    core, port, _, clock = harness()
    core.accept(goal())
    reach(core, port, clock, Stage.TRANSPORTING)
    core.request_cancel(goal().execution_id, CancelMode.RETURN_ARM)
    reach(core, port, clock, Stage.RECOVERING_ARM)
    assert core.request_cancel(goal().execution_id, CancelMode.IMMEDIATE)
    assert core.request_cancel(goal().execution_id, CancelMode.RETURN_ARM)
    result = finish(core, clock)
    assert result.cancel_mode == 'IMMEDIATE' and not result.arm_recovered
    assert result.object_state == ObjectState.LEFT_GRIPPER and result.stop_confirmed


def test_failed_release_does_not_report_successful_return_cancel(harness):
    core, port, _, clock = harness()
    port.scenario = {'stage_errors': {'RELEASING_IN_PLACE': 'MOTION_FAILED'}}
    core.accept(goal())
    reach(core, port, clock, Stage.TRANSPORTING)
    core.request_cancel(goal().execution_id, CancelMode.RETURN_ARM)
    result = finish(core, clock)
    assert result.status == Status.FAILED and result.error_code == Error.MOTION_FAILED
    assert result.cancel_mode == 'RETURN_ARM' and not result.arm_recovered
    assert Stage.RECOVERING_ARM not in port.commands


def test_invalid_cancel_mode_is_rejected_without_canceling(harness):
    core, _, _, clock = harness()
    core.accept(goal())
    assert not core.request_cancel(goal().execution_id, 'unsupported')
    assert finish(core, clock).status == Status.SUCCESS


@pytest.mark.parametrize('scenario', ['stop_failure', 'stop_timeout'])
def test_stop_failure_is_fatal_and_remains_inhibited_after_restart(harness, scenario):
    core, port, store, clock = harness(scenario)
    core.accept(goal())
    reach(core, port, clock, Stage.TRANSPORTING)
    core.request_cancel(goal().execution_id)
    result = finish(core, clock)
    assert result.status == Status.FATAL and result.error_code == Error.STOP_UNCONFIRMED
    assert not result.stop_confirmed and result.object_state == ObjectState.HELD
    path = store.path
    store.close()
    restarted, _, _, _ = harness(path=path)
    assert not restarted.accept(goal('new'))[0]


@pytest.mark.parametrize('scenario,stage,error', [
    ('hardware_fault', Stage.TRANSPORTING, Error.HARDWARE_ERROR),
    ('e_stop', Stage.LIFTING, Error.E_STOP),
])
def test_fault_wins_over_cancel_and_bypasses_atomic_wait(harness, scenario, stage, error):
    core, port, _, clock = harness(scenario)
    core.accept(goal())
    reach(core, port, clock, stage)
    clock.now += 0.3
    core.request_cancel(goal().execution_id)
    core.tick()
    assert port.stage == Stage.STOPPING
    result = finish(core, clock)
    assert result.status == Status.FATAL and result.error_code == error
    assert result.failed_stage == stage.value
    assert not core.accept(goal('new'))[0]


def test_parallel_requests_and_duplicate_arguments_execute_once(harness):
    core, port, store, clock = harness()
    with ThreadPoolExecutor(max_workers=2) as executor:
        decisions = list(executor.map(core.accept, [goal(), goal('other')]))
    assert sum(accepted for accepted, _ in decisions) == 1
    finish(core, clock)
    accepted_goal = store.all_records()[0].goal
    assert not core.accept(accepted_goal)[0]
    assert not core.accept(replace(accepted_goal, object_id=2))[0]
    assert port.commands.count(Stage.GRASPING) == 1


def test_cancel_and_success_race_has_exactly_one_terminal_result(harness):
    for _ in range(10):
        core, port, store, clock = harness()
        core.accept(goal())
        reach(core, port, clock, Stage.VERIFYING_PLACEMENT)
        clock.now += 0.6
        with ThreadPoolExecutor(max_workers=2) as executor:
            tick = executor.submit(core.tick)
            cancel = executor.submit(core.request_cancel, goal().execution_id)
            tick.result()
            cancel_accepted = cancel.result()
        result = finish(core, clock)
        assert result.status == (Status.CANCELED if cancel_accepted else Status.SUCCESS)
        assert store.get(goal().execution_id).result == result
        assert sum(event.result is not None for event in core.drain_events()) == 1


def test_crash_after_physical_evidence_keeps_that_evidence_before_step_completion(harness):
    core, port, store, clock = harness()
    core.accept(goal())
    reach(core, port, clock, Stage.GRASPING)
    # Emulate interruption immediately after contact evidence reaches the journal.
    from cleany_skill_executor.manipulation.ports import Evidence
    core._apply(Evidence(object_state=ObjectState.HELD, message='Fresh contact evidence'))
    before = store.get(goal().execution_id)
    assert before.object_state == ObjectState.HELD
    assert before.last_completed_stage == Stage.APPROACHING.value
    path = store.path
    store.close()
    restarted, _, _, _ = harness(path=path)
    after = restarted.get(goal().execution_id)
    assert after.object_state == ObjectState.HELD
    assert after.evidence_at_ns == before.evidence_at_ns
    assert after.record_state == RecordState.INTERRUPTED and after.result is None


def test_recovery_has_no_action_result_and_keeps_last_confirmed_evidence(harness):
    core, port, store, clock = harness()
    core.accept(goal())
    reach(core, port, clock, Stage.LIFTING)
    before = store.get(goal().execution_id)
    assert before.object_state == ObjectState.HELD
    path = store.path
    store.close()
    restarted, new_port, recovered_store, _ = harness(path=path)
    record = restarted.get(goal().execution_id)
    assert record.record_state == RecordState.INTERRUPTED
    assert record.human_confirmation_required and record.result is None
    assert not record.stop_confirmed
    assert record.object_state == before.object_state
    assert record.stage == before.stage and record.last_completed_stage == before.last_completed_stage
    assert record.substage == before.substage
    assert record.completed_substages == before.completed_substages
    assert record.evidence_at_ns == before.evidence_at_ns
    assert restarted.drain_events() == [record]
    assert not restarted.accept(goal('new'))[0]
    assert new_port.commands == []
    assert recovered_store.get(goal().execution_id) == record


@pytest.mark.parametrize('failure_point', ['stage_start', 'contact', 'final', 'all', 'serialization'])
def test_record_failure_preserves_success_motion_and_runtime_lookup(harness, monkeypatch, failure_point):
    core, port, store, clock = harness()
    save = store.save

    def fail(record, **kwargs):
        selected = (
            failure_point in ('all', 'serialization')
            or (failure_point == 'stage_start' and record.stage == Stage.APPROACHING)
            or (failure_point == 'contact' and record.object_state == ObjectState.HELD)
            or (failure_point == 'final' and record.result is not None)
        )
        if selected:
            if failure_point == 'serialization':
                raise TypeError('injected serialization failure')
            raise StoreError('injected disk failure')
        save(record, **kwargs)

    monkeypatch.setattr(store, 'save', fail)
    assert core.accept(goal())[0]
    result = finish(core, clock)
    assert result.status == Status.SUCCESS and result.error_code == Error.NONE
    assert result.stop_confirmed and result.arm_recovered
    assert result.object_state == ObjectState.LEFT_GRIPPER
    assert result.placement_state == Placement.CONFIRMED
    assert core.record.record_state == RecordState.FINISHED
    assert not core.record.human_confirmation_required and not core.inhibited
    assert port.commands == list(NORMAL_STAGES)
    assert core.get(goal().execution_id).result == result
    assert not core.accept(goal())[0]
    if failure_point == 'final':
        assert store.get(goal().execution_id).result is None
    assert core.accept(goal('new'))[0]
    assert finish(core, clock).status == Status.SUCCESS
    # The previous result stays queryable after another Goal replaces core.record.
    assert core.get(goal().execution_id).result == result


def test_stop_failure_remains_explicit_when_recording_also_fails(harness, monkeypatch):
    core, port, store, clock = harness('stop_failure')
    save = store.save

    def fail_contact(record, **kwargs):
        if record.object_state == ObjectState.HELD:
            raise StoreError('injected contact transaction failure')
        save(record, **kwargs)

    monkeypatch.setattr(store, 'save', fail_contact)
    core.accept(goal())
    reach(core, port, clock, Stage.TRANSPORTING)
    assert core.request_cancel(goal().execution_id)
    result = finish(core, clock)
    assert result.status == Status.FATAL and result.error_code == Error.STOP_UNCONFIRMED
    assert not result.stop_confirmed and result.object_state == ObjectState.HELD
    assert core.record.record_state == RecordState.FINISHED
    assert Stage.LIFTING in port.commands
    assert not core.accept(goal('new'))[0]


@pytest.mark.parametrize('scenario,status,error', [
    ('backend_not_ready', Status.BLOCKED, Error.BACKEND_NOT_READY),
    ('grasp_failure', Status.FAILED, Error.GRASP_FAILED),
])
def test_record_failure_does_not_replace_physical_failure(harness, monkeypatch, scenario, status, error):
    core, _, store, clock = harness(scenario)
    def fail(*args, **kwargs):
        raise StoreError('injected persistent write outage')
    monkeypatch.setattr(store, 'save', fail)
    assert core.accept(goal())[0]
    result = finish(core, clock)
    assert (result.status, result.error_code) == (status, error)
    assert core.get(goal().execution_id).result == result


def test_record_failure_preserves_cancel_and_stop_confirmation(harness, monkeypatch):
    core, port, store, clock = harness()
    def fail(*args, **kwargs):
        raise StoreError('injected persistent write outage')
    monkeypatch.setattr(store, 'save', fail)
    assert core.accept(goal())[0]
    reach(core, port, clock, Stage.APPROACHING)
    assert core.request_cancel(goal().execution_id)
    result = finish(core, clock)
    assert result.status == Status.CANCELED and result.error_code == Error.CANCELED
    assert result.stop_confirmed


def test_initial_record_write_is_retried_without_reexecuting_motion(harness, monkeypatch):
    core, port, store, clock = harness()
    save = store.save
    def fail_initial(record, **kwargs):
        if record.revision == 0:
            raise StoreError('injected initial write failure')
        save(record, **kwargs)
    monkeypatch.setattr(store, 'save', fail_initial)
    assert core.accept(goal())[0]
    assert store.get(goal().execution_id) is None
    result = finish(core, clock)
    assert result.status == Status.SUCCESS
    assert store.get(goal().execution_id).result == result
    assert port.commands == list(NORMAL_STAGES)


def test_success_cannot_be_created_without_stop_evidence(harness, monkeypatch):
    core, port, _, clock = harness()
    poll = port.poll

    def missing_stop(now):
        evidence = poll(now)
        if evidence is not None and port.stage == Stage.RETURNING_ARM:
            return replace(evidence, stop_confirmed=False)
        return evidence

    monkeypatch.setattr(port, 'poll', missing_stop)
    core.accept(goal())
    assert finish(core, clock).status == Status.FATAL


def test_atomic_timeout_is_bounded_and_late_evidence_cannot_succeed(harness):
    config = replace(MockConfig(), motion_timeout_sec=0.6,
                     scenarios={'slow': {'stage_durations_sec': {'GRASPING': 2.0}}})
    core, port, _, clock = harness('slow', config=config)
    core.accept(goal())
    reach(core, port, clock, Stage.GRASPING)
    core.request_cancel(goal().execution_id)
    result = finish(core, clock)
    assert result.status == Status.FAILED and result.error_code == Error.TIMEOUT
    assert result.object_state == ObjectState.UNKNOWN
    assert Stage.LIFTING not in port.commands


def test_stop_command_exception_cannot_leave_execution_running(harness, monkeypatch):
    core, port, _, clock = harness()
    core.accept(goal())
    reach(core, port, clock, Stage.APPROACHING)

    def broken_stop(now):
        raise RuntimeError('stop transport unavailable')

    monkeypatch.setattr(port, 'stop', broken_stop)
    core.request_cancel(goal().execution_id)
    result = finish(core, clock)
    assert result.status == Status.FATAL and result.error_code == Error.STOP_UNCONFIRMED


def test_database_ownership_uniqueness_and_atomic_journal(harness):
    _, _, store, _ = harness()
    with pytest.raises(StoreError, match='already owned'):
        ExecutionStore(str(store.path))
    record = Record(goal())
    store.save(record, create=True)
    with pytest.raises(DuplicateExecution):
        store.save(record, create=True)
    # Journal conflict must roll back the corresponding snapshot update.
    with pytest.raises(StoreError):
        store.save(replace(record, object_state=ObjectState.HELD))
    assert store.get(goal().execution_id).object_state == ObjectState.NOT_TOUCHED
    with sqlite3.connect(str(store.path)) as db:
        assert db.execute('SELECT COUNT(*) FROM execution_events').fetchone()[0] == 1


def test_database_path_uses_xdg_state_home(monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_STATE_HOME', str(tmp_path))
    assert default_database_path() == str(tmp_path / 'cleany/manipulation_mock/executions.sqlite3')


@pytest.mark.parametrize('field,value', [('motion_timeout_sec', 0), ('stop_timeout_sec', float('nan'))])
def test_mock_config_rejects_invalid_deadlines(field, value):
    with pytest.raises(ValueError):
        replace(MockConfig(), **{field: value})
