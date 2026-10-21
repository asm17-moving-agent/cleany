from dataclasses import replace
import math
import pytest
from cleany_skill_executor.core.manipulation_evidence import ContactConfig, ContactHistory, MotorSample, TrackingSample


def history(**changes):
    config=ContactConfig(load_threshold_raw=50.,closed_position_rad=-.3745)
    h=ContactHistory(replace(config,**changes))
    for n in range(11):
        h.add(MotorSample(10_000_000_000+n*20_000_000,.1,100.,1.,0.,35.))
    return h


def tracking(**changes):
    return replace(TrackingSample(10_190_000_000,'left','wrist','snapshot',1,True,True,400,10000),**changes)


def evaluate(h,t=None,**changes):
    args=dict(arm='left',wrist_reference_id='wrist',snapshot_id='snapshot',object_id=1,
              after_ns=10_000_000_000,now_ns=10_210_000_000)
    args.update(changes)
    return h.evaluate(t or tracking(),**args)


def test_sustained_load_stall_and_exact_tracking_identify_contact_proxy():
    r=evaluate(history())
    assert r.success and r.contact and r.tracked
    assert r.stamp_ns==10_190_000_000  # Oldest constituent, never receipt time.


@pytest.mark.parametrize('field,value',[('load_raw',0.),('position',-.3745)])
def test_empty_closed_jaw_or_no_load_is_not_contact(field,value):
    h=history();h.samples=type(h.samples)(replace(s,**{field:value}) for s in h.samples)
    r=evaluate(h);assert r.success and not r.contact


def test_single_load_dip_rejects_contact_but_position_variation_does_not():
    h=history();h.samples[0]=replace(h.samples[0],load_raw=0.)
    h.samples[-3]=replace(h.samples[-3],load_raw=0.)
    assert not evaluate(h).contact
    h=history();h.samples[-1]=replace(h.samples[-1],position=.12)
    assert evaluate(h).contact


@pytest.mark.parametrize('field,value',[('load_raw',float('nan')),('load_raw',2048.),('load_raw',1.5),('torque_enabled',0.),('status',4.),('temperature',60.),('position',float('inf'))])
def test_faulted_or_malformed_telemetry_cannot_confirm(field,value):
    h=history();h.samples[-1]=replace(h.samples[-1],**{field:value})
    assert not evaluate(h).success


@pytest.mark.parametrize('field,value',[('reference_id','other'),('snapshot_id','other'),('object_id',2),('arm','right'),('valid',False),('mask_pixels',0),('image_pixels',1),('stamp_ns',9_000_000_000),('stamp_ns',11_000_000_000)])
def test_wrong_object_invalid_mask_future_and_stale_frames_rejected(field,value):
    assert not evaluate(history(),tracking(**{field:value})).success


def test_unconfigured_threshold_and_closed_reference_are_explicitly_blocked():
    assert 'unconfigured' in evaluate(history(load_threshold_raw=0.)).reason
    assert 'unconfigured' in evaluate(history(closed_position_rad=math.nan)).reason


def test_replay_does_not_refresh_history_and_after_stamp_is_strict():
    h=history();h.add(MotorSample(10_100_000_000,.1,100.,1.,0.,35.))
    assert len(h.samples)==11
    assert not evaluate(h,after_ns=10_190_000_000).success
    assert not evaluate(h,now_ns=11_000_000_000).success


def test_load_sign_bit_is_not_force_and_discontinuous_history_is_invalid():
    h=history();h.samples=type(h.samples)(replace(s,load_raw=1124.) for s in h.samples)
    assert evaluate(h).contact  # bit10 direction, magnitude100
    h.samples=type(h.samples)([h.samples[0],h.samples[-2],h.samples[-1]])
    assert not evaluate(h).success


def test_visible_false_is_not_tracked_and_reverse_joint_direction_supported():
    assert not evaluate(history(),tracking(visible=False)).tracked
    h=history(closed_position_rad=.3745,opening_direction=-1)
    assert evaluate(h).contact
