from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from cleany_skill_executor.core.gripper_contact import (
    ContactCriteria, ContactSample, ContactState, LiveContactHistory, evaluate_contact,
)


def live_interfaces(sample):
    values = dict(sample.__dict__)
    values.pop('motor_id')
    values['feedback_stamp_ns'] = values.pop('stamp_ns')
    values['temperature_raw'] = values.pop('temperature_c')
    values['motor_status_raw'] = values.pop('motor_status')
    values['current_limit_raw'] = 250
    return values


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_live_contact_uses_source_stamp_and_requires_raw_registers(arm):
    c = criteria(arm)
    history = LiveContactHistory(c)
    for sample in samples(c):
        history.observe(live_interfaces(sample))
    now = history.samples[-1].stamp_ns
    assert history.evaluate(now_ns=now).state == ContactState.CONTACT
    # Repeated broadcaster publication cannot renew a stale source read.
    history.observe(live_interfaces(history.samples[-1]))
    assert history.evaluate(now_ns=now+110_000_000).state == ContactState.UNKNOWN
    broken = live_interfaces(replace(samples(c)[-1], stamp_ns=now+20_000_000))
    broken.pop('protocol_error_code')
    history.observe(broken)
    assert history.evaluate(now_ns=now+20_000_000).state == ContactState.UNKNOWN
    for sample in samples(c):
        history.observe(live_interfaces(sample))
    assert history.evaluate(now_ns=now).state == ContactState.UNKNOWN


@pytest.mark.parametrize('field,value', [
    ('temperature_raw', 150), ('motor_status_raw', 32),
    ('torque_limit_readback_raw', 800), ('current_limit_raw', 400),
    ('current_raw', 250), ('goal_position_raw', 1345),
])
def test_live_raw_faults_cannot_be_masked_by_normalized_interfaces(field, value):
    c = criteria()
    history = LiveContactHistory(c)
    for sample in samples(c):
        interfaces = live_interfaces(sample)
        interfaces.update(temperature=35., motor_status=0.)
        interfaces[field] = value
        history.observe(interfaces)
    assert history.evaluate(now_ns=samples(c)[-1].stamp_ns).state == ContactState.UNKNOWN


def criteria(arm='left'):
    root = Path(__file__).resolve().parents[4]
    data = yaml.safe_load((root/'configs/robot/gripper_contact_decision.yaml').read_text())
    return ContactCriteria(**data['criteria'], **data['arms'][arm])


def samples(c, **changes):
    sample = ContactSample(1_000_000_000, c.motor_id, c.closed_goal_raw+340,
                           c.closed_goal_raw, 18, 200, 1, 0, 0, 0,
                           c.output_limit_raw, 120, 40)
    return [replace(sample, stamp_ns=sample.stamp_ns+n*10_000_000, **changes)
            for n in range(51)]


def decide(w, c, **changes):
    options = dict(now_ns=w[-1].stamp_ns, after_ns=999_000_000)
    options.update(changes)
    return evaluate_contact(w, c, **options)


@pytest.mark.parametrize('arm,goal,empty_residual,current,load', [
    ('left', 1358, 6, 1, 36), ('right', 1372, 12, 2, 60),
])
def test_bilateral_zero_offsets_and_observed_empty_signatures(arm, goal, empty_residual, current, load):
    c = criteria(arm)
    assert c.closed_goal_raw == goal
    result = decide(samples(c), c)
    assert result.state == ContactState.CONTACT and result.window_sec >= .4
    w = samples(c, position_raw=goal+empty_residual,
                current_raw=current, load_magnitude_raw=load)
    assert decide(w, c).state == ContactState.EMPTY


@pytest.mark.parametrize('field,value,expected', [
    ('position_raw', 1386, ContactState.UNKNOWN),  # left goal +28: strict >
    ('position_raw', 1387, ContactState.CONTACT),
    ('current_raw', 8, ContactState.UNKNOWN),
    ('current_raw', 9, ContactState.CONTACT),
    ('load_magnitude_raw', 129, ContactState.UNKNOWN),
    ('load_magnitude_raw', 130, ContactState.CONTACT),
])
def test_contact_threshold_boundaries(field, value, expected):
    c = criteria()
    assert decide(samples(c, **{field: value}), c).state == expected


@pytest.mark.parametrize('field,value', [
    ('protocol_error_code', None), ('current_raw', None),
    ('motor_id', 12), ('goal_position_raw', 1400), ('torque_limit_readback_raw', 400),
    ('torque_enabled', 0), ('motor_status', 32), ('protocol_error_code', 32),
    ('temperature_c', 80), ('temperature_c', 150), ('temperature_c', -1),
    ('voltage_raw', 89), ('voltage_raw', 141), ('moving_flag', 1),
    ('position_raw', float('nan')), ('current_raw', float('inf')),
    ('current_raw', -18), ('current_raw', 32768), ('load_magnitude_raw', 1124),
    ('current_raw', 250), ('current_raw', 32767),
    ('load_magnitude_raw', 1001), ('position_raw', 1.5),
])
def test_missing_faulted_wrong_arm_and_encoded_words_cannot_confirm(field, value):
    c = criteria()
    w = samples(c)
    w[-1] = replace(w[-1], **{field: value})
    assert decide(w, c).state == ContactState.UNKNOWN


@pytest.mark.parametrize('arm', ['left', 'right'])
@pytest.mark.parametrize('temperature,expected', [
    (60, ContactState.CONTACT), (61, ContactState.CONTACT),
    (73, ContactState.CONTACT), (79, ContactState.CONTACT),
    (80, ContactState.UNKNOWN), (81, ContactState.UNKNOWN),
])
def test_operational_temperature_limit_is_strictly_below_80(arm, temperature, expected):
    c = criteria(arm)
    result = decide(samples(c, temperature_c=temperature), c)
    assert result.state == expected
    if expected == ContactState.UNKNOWN:
        assert result.reason == 'voltage_or_temperature_invalid'


def test_explicit_lower_temperature_limit_remains_enforced():
    c = replace(criteria(), temperature_limit_c=60)
    assert decide(samples(c, temperature_c=59), c).state == ContactState.CONTACT
    assert decide(samples(c, temperature_c=60), c).state == ContactState.UNKNOWN


@pytest.mark.parametrize('arm', ['left', 'right'])
@pytest.mark.parametrize('span', [4, 17, 100])
@pytest.mark.parametrize('direction', [-1, 1])
def test_loaded_position_changes_are_diagnostic_only(arm, span, direction):
    c = criteria(arm)
    w = samples(c)
    w = [replace(s, position_raw=c.closed_goal_raw+340+
                 direction*round(span*max(0, i-10)/40)) for i, s in enumerate(w)]
    result = decide(w, c)
    assert result.state == ContactState.CONTACT
    assert result.position_span_ticks == span


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_empty_decision_also_ignores_position_span(arm):
    c = criteria(arm)
    w = samples(c, current_raw=2, load_magnitude_raw=36)
    w = [replace(s, position_raw=c.closed_goal_raw+round(28*max(0, i-10)/40))
         for i, s in enumerate(w)]
    result = decide(w, c)
    assert result.state == ContactState.EMPTY and result.position_span_ticks == 28


def test_current_spike_or_one_low_load_sample_cannot_create_contact():
    c = criteria()
    w = samples(c, current_raw=2)
    w[-1] = replace(w[-1], current_raw=200)
    assert decide(w, c).state == ContactState.UNKNOWN
    w = samples(c)
    w[-1] = replace(w[-1], load_magnitude_raw=60)
    assert decide(w, c).state == ContactState.UNKNOWN


def test_empty_requires_all_three_low_signals():
    c = criteria()
    w = samples(c, position_raw=c.closed_goal_raw+28, current_raw=8, load_magnitude_raw=129)
    assert decide(w, c).state == ContactState.EMPTY
    w[-1] = replace(w[-1], load_magnitude_raw=130)
    assert decide(w, c).state == ContactState.UNKNOWN


def test_window_duration_gap_freshness_request_epoch_and_replay_are_explicit():
    c = criteria()
    w = samples(c)
    assert decide(w[-40:], c).state == ContactState.UNKNOWN  # only .39 seconds
    gap = w[:21]+w[30:]
    assert decide(gap, c).reason == 'feedback_gap'  # exactly .1 seconds
    assert decide(w, c, now_ns=w[-1].stamp_ns+100_000_001).reason == 'stale_or_future_feedback'
    assert decide(w, c, now_ns=w[-1].stamp_ns-1).reason == 'stale_or_future_feedback'
    assert decide(w, c, after_ns=w[10].stamp_ns).state == ContactState.UNKNOWN
    assert decide(w+[w[-1]], c).reason == 'replayed_or_out_of_order_feedback'
    assert evaluate_contact([], c, now_ns=2_000_000_000, after_ns=0).state == ContactState.UNKNOWN


def test_closing_direction_is_explicit():
    c = replace(criteria(), opening_direction=-1)
    assert decide(samples(c, position_raw=c.closed_goal_raw-340), c).state == ContactState.CONTACT


@pytest.mark.parametrize('changes', [
    dict(output_limit_raw=0), dict(current_threshold_raw=0), dict(load_threshold_raw=0),
    dict(current_stop_raw=9),
    dict(opening_direction=0), dict(window_sec=0), dict(maximum_age_sec=float('nan')),
])
def test_invalid_criteria_rejected(changes):
    with pytest.raises(ValueError):
        replace(criteria(), **changes)
