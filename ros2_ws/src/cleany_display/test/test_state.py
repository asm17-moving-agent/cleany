import json
from pathlib import Path

import pytest
from cleany_display.state import DisplayState


@pytest.fixture
def state():
    return DisplayState(json.loads((Path(__file__).parents[1]/'config/display.json').read_text()))


def snapshot(state, now=1, sources=None, wall=100):
    return state.snapshot(now, set(state.samples) if sources is None else sources, wall)


def test_initial_idle_then_disconnect_recovery(state):
    assert snapshot(state)['mode'] == 'idle'
    state.ingest('mission', {'state': 'NAVIGATE_TO_TARGET'}, 1)
    assert snapshot(state)['mode'] == 'moving'
    assert snapshot(state, 2, set())['mode'] == 'idle'
    assert snapshot(state, 5, set())['mode'] == 'help'
    state.ingest('mission', {'state': 'IDLE'}, 6)
    assert snapshot(state, 6)['mode'] == 'idle'


def test_event_driven_state_does_not_expire_while_publisher_alive(state):
    state.ingest('mission', {'state': 'WORKING', 'bt_stage': 'ExecuteOne'}, 1)
    assert snapshot(state, 500)['mode'] == 'collecting'
    state.ingest('mission', {'state': 'WORKING', 'bt_stage': 'Reobserve'}, 501)
    assert snapshot(state, 501)['mode'] == 'checking'


def test_real_battery_nan_and_stale_charge(state):
    state.ingest('battery', {'power_supply_status': 1, 'percentage': float('nan')}, 1)
    assert snapshot(state)['mode'] == 'charging'
    assert snapshot(state)['battery_percent'] is None
    state.ingest('battery', {'power_supply_status': 1, 'percentage': .72}, 2)
    assert snapshot(state, 2)['battery_percent'] == 72
    assert snapshot(state, 20)['mode'] != 'charging'


def test_error_priority_over_override_charge_and_collection(state):
    state.ingest('battery', {'power_supply_status': 1}, 1)
    state.override({'mode': 'complete'}, 1)
    state.ingest('manipulation', {'human_confirmation_required': True}, 1)
    assert snapshot(state)['mode'] == 'help'


def test_velocity_direction_and_expiry(state):
    state.ingest('odometry', {'linear': 0., 'angular': -.2}, 1)
    assert snapshot(state)['direction'] == 'right'
    assert snapshot(state)['mode'] == 'moving'
    assert snapshot(state, 3)['mode'] != 'moving'


def test_duplicate_success_does_not_extend_celebration(state):
    event = {'has_result': True, 'status': 'SUCCESS', 'updated_at_ns': 100_000_000_000}
    state.ingest('manipulation', event, 1)
    assert snapshot(state)['mode'] == 'complete'
    state.ingest('manipulation', event, 5)
    assert snapshot(state, 8, wall=107)['mode'] == 'idle'


def test_old_success_replay_and_out_of_order_events(state):
    state.ingest('manipulation', {'record_state': 'ACTIVE', 'updated_at_ns': 100_000_000_000}, 1)
    state.ingest('manipulation', {'has_result': True, 'status': 'SUCCESS', 'updated_at_ns': 90_000_000_000}, 2)
    assert snapshot(state, 2)['mode'] == 'collecting'
    state.ingest('manipulation', {'has_result': True, 'status': 'SUCCESS', 'updated_at_ns': 110_000_000_000}, 3)
    assert snapshot(state, 3, wall=200)['mode'] == 'idle'


def test_override_expires_and_invalid_input_is_ignored(state):
    assert state.override({'mode': 'checking', 'duration_seconds': 2}, 1)
    assert snapshot(state)['mode'] == 'checking'
    assert not state.override({'mode': 'invalid'}, 2)
    assert not state.override({'mode': []}, 2)
    assert not state.override({'mode': {}}, 2)
    assert not state.override({'mode': 'charging', 'duration_seconds': float('inf')}, 2)
    assert not state.override({'mode': 'help', 'duration_seconds': True}, 2)
    assert snapshot(state, 4)['mode'] == 'idle'


def test_old_mission_result_not_replayed(state):
    report = {'mission_id': 'first', 'outcome': 'SUCCESS', 'completed_tasks': ['a']}
    state.ingest('result', report, 1)
    assert snapshot(state)['mode'] == 'idle'
    state.ingest('mission', {'mission_id': 'first', 'state': 'WORKING'}, 1)
    state.ingest('mission', {'state': 'IDLE'}, 2)
    state.ingest('result', report, 2)
    result = snapshot(state, 2)
    assert result['mode'] == 'complete'
    assert result['collected_count'] == 1
    state.ingest('result', report, 5)
    assert snapshot(state, 9)['mode'] == 'idle'
    state.ingest('mission', {'mission_id': 'second', 'state': 'WORKING'}, 10)
    state.ingest('mission', {'state': 'IDLE'}, 11)
    state.ingest('result', report, 11)
    assert snapshot(state, 11)['mode'] == 'idle'


def test_display_override_alone_does_not_claim_lost_robot_connection(state):
    state.override({'mode': 'charging', 'duration_seconds': 1}, 1)
    assert snapshot(state, 1)['mode'] == 'charging'
    assert snapshot(state, 20)['mode'] == 'idle'
