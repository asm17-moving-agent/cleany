"""Revision-bound recovery acknowledgement preserves historical failure evidence."""
from dataclasses import replace
from types import SimpleNamespace
from concurrent.futures import ThreadPoolExecutor
import threading

import pytest
from cleany_manipulation_bt.core import ExecutionCore
from cleany_manipulation_bt.store import BTExecutionStore
from cleany_skill_executor.manipulation.models import (
    Error, Goal, ObjectState, Placement, Record, RecordState, Result, Stage, Status,
)
from cleany_skill_executor.manipulation.store import StoreError


def failed_record(execution_id='failed'):
    goal = Goal('mission', 'task', execution_id, 'collect_trash', 'old-snapshot', 1, 'return_to_pick')
    result = Result(execution_id, 'jetson', Status.FATAL, Error.STOP_UNCONFIRMED,
        'STOPPING', 'VALIDATING', ObjectState.NOT_TOUCHED, Placement.NOT_CHECKED,
        '', False, False, False, 'Physical stop unconfirmed')
    return Record(goal, execution_profile='jetson', record_state=RecordState.FINISHED,
        stage=Stage.FINALIZING, result=result, human_confirmation_required=True, revision=17)


@pytest.fixture
def recovery(tmp_path):
    store = BTExecutionStore(str(tmp_path / 'executions.sqlite3'))
    record = failed_record()
    store.save(record, create=True)
    backend = SimpleNamespace(ready=lambda: True)
    core = ExecutionCore(backend, store, '', lambda *args: object(), execution_profile='jetson')
    yield core, store, record
    store.close()


def confirm(core, record, **kwargs):
    options = dict(stopped=True, gripper_empty=True, note='Operator inspection', verify_stationary=lambda: True)
    options.update(kwargs)
    return core.confirm_recovery(record.goal.execution_id, record.revision, **options)


def test_acknowledgement_survives_restart_and_keeps_fatal_record(recovery):
    core, store, record = recovery
    assert core.inhibited
    assert confirm(core, record)[0]
    assert not core.inhibited and core.recovery_blocks() == []
    assert core.get(record.goal.execution_id) == store.get(record.goal.execution_id) == record
    rows = store._db.execute('SELECT execution_id, revision, operator_note FROM recovery_confirmations').fetchall()
    assert rows == [('failed', 17, 'Operator inspection')]
    restarted = ExecutionCore(core.backend, store, '', lambda *args: object(), execution_profile='jetson')
    assert not restarted.inhibited
    assert restarted.get('failed').result.status == Status.FATAL
    assert restarted.get('failed').human_confirmation_required


@pytest.mark.parametrize('kwargs', [
    {'stopped': False}, {'gripper_empty': False}, {'note': ' '},
    {'verify_stationary': lambda: False},
])
def test_missing_operator_or_stationary_evidence_keeps_inhibition(recovery, kwargs):
    core, store, record = recovery
    assert not confirm(core, record, **kwargs)[0]
    assert core.inhibited
    assert store._db.execute('SELECT COUNT(*) FROM recovery_confirmations').fetchone()[0] == 0


def test_active_execution_or_pending_operations_cannot_be_acknowledged(recovery):
    core, _, record = recovery
    core.record = replace(record, record_state=RecordState.ACTIVE, result=None)
    assert not confirm(core, record)[0]
    core.record = None
    core.backend.ready = lambda: False
    assert not confirm(core, record)[0]
    assert core.inhibited


def test_different_or_stale_revision_cannot_clear_the_gate(recovery):
    core, store, record = recovery
    assert not confirm(core, replace(record, revision=record.revision-1))[0]
    store.save(replace(record, revision=record.revision+1))
    assert not confirm(core, record)[0]
    assert core.inhibited


def test_acknowledgement_write_failure_keeps_gate_and_original_record(recovery, monkeypatch):
    core, store, record = recovery

    def fail(*args):
        raise StoreError('disk unavailable')

    monkeypatch.setattr(store, 'confirm_recovery', fail)
    accepted, message = confirm(core, record)
    assert not accepted and 'disk unavailable' in message
    assert core.inhibited and store.get('failed') == record


def test_another_record_still_blocks_and_a_changed_record_invalidates_ack(recovery):
    core, store, first = recovery
    second = failed_record('another')
    store.save(second, create=True)
    core = ExecutionCore(core.backend, store, '', lambda *args: object(), execution_profile='jetson')
    assert confirm(core, first)[0] and core.inhibited
    assert [r.goal.execution_id for r in core.recovery_blocks()] == ['another']
    assert confirm(core, second)[0] and not core.inhibited
    changed = replace(first, revision=18, message='New failure evidence')
    store.save(changed)
    restarted = ExecutionCore(core.backend, store, '', lambda *args: object(), execution_profile='jetson')
    assert restarted.inhibited
    assert [r.goal.execution_id for r in restarted.recovery_blocks()] == ['failed']


def test_interrupted_record_can_be_acknowledged_without_resuming_old_goal(recovery):
    core, store, record = recovery
    active = replace(record, record_state=RecordState.ACTIVE, result=None, revision=18)
    store.save(active)
    restarted = ExecutionCore(core.backend, store, '', lambda *args: object(), execution_profile='jetson')
    interrupted = restarted.get('failed')
    assert interrupted.record_state == RecordState.INTERRUPTED and restarted.tree is None
    assert confirm(restarted, interrupted)[0]
    assert not restarted.inhibited and restarted.tree is None
    assert store.get('failed').result is None


def test_confirmation_is_idempotent_and_does_not_create_new_events(recovery):
    core, store, record = recovery
    assert confirm(core, record)[0]
    assert confirm(core, record)[0]
    assert store._db.execute('SELECT COUNT(*) FROM recovery_confirmations').fetchone()[0] == 1
    assert store._db.execute('SELECT COUNT(*) FROM execution_events').fetchone()[0] == 1


def test_stop_verification_leaves_callbacks_free_and_blocks_other_recoveries(recovery):
    core, _, record = recovery
    entered, finish = threading.Event(), threading.Event()

    def verify():
        entered.set()
        assert finish.wait(3.)
        return True

    with ThreadPoolExecutor(max_workers=1) as executor:
        task = executor.submit(confirm, core, record, verify_stationary=verify)
        try:
            assert entered.wait(2.)
            assert core.lock.acquire(timeout=.5)
            core.lock.release()
            assert not core.accept(Goal('m', 't', 'new', 'collect_trash', 'new-snapshot', 2, 'return_to_pick'))[0]
            assert not confirm(core, record)[0]
        finally:
            finish.set()
        assert task.result(timeout=2.)[0]
    assert not core.inhibited
