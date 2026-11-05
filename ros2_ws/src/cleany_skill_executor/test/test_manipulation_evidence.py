from dataclasses import replace
import pytest
from cleany_skill_executor.core.manipulation_evidence import (
    RawContactHistory, TrackingConfig, TrackingSample,
)
from test_gripper_contact import criteria, live_interfaces, samples


def history():
    c = criteria()
    result = RawContactHistory(c, TrackingConfig())
    for sample in samples(c):
        result.observe(live_interfaces(sample))
    return result


def tracking(**changes):
    return replace(TrackingSample(1_490_000_000, 'left', 'wrist', 'snapshot',
                                  1, True, True, 400, 10000), **changes)


def evaluate(h, observed=None, **changes):
    args = dict(arm='left', wrist_reference_id='wrist', snapshot_id='snapshot',
                object_id=1, after_ns=1_000_000_000, now_ns=1_510_000_000)
    args.update(changes)
    return h.evaluate(observed or tracking(), **args)


def test_sustained_motor_contact_and_exact_tracking_identify_contact_proxy():
    result = evaluate(history())
    assert result.success and result.contact and result.tracked
    assert result.stamp_ns == 1_490_000_000
    assert not evaluate(history(), tracking(visible=False)).tracked


@pytest.mark.parametrize('field,value', [
    ('reference_id', 'other'), ('snapshot_id', 'other'), ('object_id', 2),
    ('arm', 'right'), ('valid', False), ('mask_pixels', 0), ('image_pixels', 1),
    ('stamp_ns', 900_000_000), ('stamp_ns', 2_000_000_000),
    ('stamp_ns', 1_200_000_000),
])
def test_wrong_identity_invalid_mask_and_invalid_time_cannot_confirm(field, value):
    assert not evaluate(history(), tracking(**{field: value})).success


def test_after_stamp_is_strict_and_old_motor_evidence_cannot_confirm():
    assert not evaluate(history(), after_ns=1_490_000_000).success
    assert not evaluate(history(), now_ns=2_100_000_000).success


@pytest.mark.parametrize('field', ['maximum_age_sec', 'maximum_skew_sec'])
@pytest.mark.parametrize('value', [0., -1., float('nan'), float('inf')])
def test_tracking_limits_reject_invalid_values(field, value):
    with pytest.raises(ValueError):
        TrackingConfig(**{field: value})
