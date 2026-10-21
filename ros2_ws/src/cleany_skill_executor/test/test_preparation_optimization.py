"""Preparation reuse must preserve changing state, bounds and fresh evidence."""
from types import SimpleNamespace as NS

import pytest

from cleany_skill_executor.core.joint_stability import JointStabilityHistory
from cleany_skill_executor.core.hardware_execution import PhysicalJointLimit, bounded_path


def stable_history():
    history = JointStabilityHistory()
    for i in range(1, 8):
        history.observe(i*100_000_000, {'a': .1, 'b': .2}, 0)
    return history


def settled(history, now=700_000_000, duration=.5, q=None):
    return history.stable(('a', 'b'), q or {'a': .1, 'b': .2}, now_ns=now,
        duration_sec=duration, maximum_age_sec=.25, velocity_rad_s=.02)


def test_stability_reuses_measured_window_but_rejects_current_drift():
    history = stable_history()
    assert settled(history)
    assert settled(history, duration=.3)
    assert not settled(history, q={'a': .2, 'b': .2})
    assert not settled(history, now=960_000_000)
    assert not settled(history, now=600_000_000)


@pytest.mark.parametrize('kind', ['motion', 'recovery', 'clock_reset', 'nan', 'gap', 'clear'])
def test_stability_never_reuses_invalidated_evidence(kind):
    history = stable_history()
    if kind == 'motion': history.observe(800_000_000, {'a': .2, 'b': .2}, 0)
    if kind == 'recovery': history.observe(800_000_000, {'a': .1, 'b': .2}, 1)
    if kind == 'clock_reset': history.observe(100_000_000, {'a': .1, 'b': .2}, 0)
    if kind == 'nan': history.observe(800_000_000, {'a': float('nan'), 'b': .2}, 0)
    if kind == 'gap': history.observe(1_000_000_000, {'a': .1, 'b': .2}, 0)
    if kind == 'clear': history.clear()
    assert not settled(history, now=1_000_000_000 if kind == 'gap' else 800_000_000)


def test_republished_feedback_cannot_manufacture_a_stable_window():
    history = JointStabilityHistory()
    for _ in range(100): history.observe(700_000_000, {'a': .1, 'b': .2}, 0)
    assert not settled(history)


def test_stable_preparation_has_no_fixed_sleep_but_still_monitors(monkeypatch):
    from cleany_skill_executor import hardware_execution as module
    history = stable_history()
    values = {'hardware_endpoint_settle_timeout_sec': 2., 'hardware_settle_time_sec': .5,
              'hardware_endpoint_settle_velocity_rad_s': .02, 'hardware_guard_max_age_sec': .25}
    calls = []
    fake = NS(_stability_history=history, value=values.__getitem__,
        monitor=lambda: (calls.append('monitor') or NS(name=['a', 'b'], position=[.1, .2])),
        node=NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=700_000_000))))
    fake.wait_for_settle = lambda names, **kw: module.HardwareExecution.wait_for_settle(fake, names, **kw)
    monkeypatch.setattr(module.time, 'sleep', lambda _: pytest.fail('Stable history must be reusable'))
    assert module.HardwareExecution._wait_before_execution(fake, ('a', 'b')).position == [.1, .2]
    assert calls == ['monitor']


def test_real_feedback_adapter_uses_acquisition_stamp_and_resets_on_recovery():
    import threading
    from cleany_skill_executor.hardware_execution import HardwareExecution
    from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES
    names = [name for arm in ('left', 'right') for name in (*ARM_JOINT_NAMES[arm], f'{arm}_gripper_joint')]
    fake = NS(_stability_history=JointStabilityHistory(), contact_lock=threading.Lock(),
              contact_histories={}, fault='')
    def message(stamp, generation=0):
        return NS(joint_names=names, interface_values=[NS(
            interface_names=['position','feedback_stamp_ns','communication_recovery_count'],
            values=[.1,float(stamp),float(generation)]) for _ in names])
    for i in range(1,8):HardwareExecution.on_contact_feedback(fake,message(i*100_000_000))
    options = dict(now_ns=700_000_000,duration_sec=.5,maximum_age_sec=.25,velocity_rad_s=.02)
    assert fake._stability_history.stable(names, dict.fromkeys(names,.1),**options)
    HardwareExecution.on_contact_feedback(fake,message(800_000_000,1))
    options['now_ns'] = 800_000_000
    assert not fake._stability_history.stable(names, dict.fromkeys(names,.1),**options)
    assert not fake.fault and fake.communication_recovery_count == 1


@pytest.mark.parametrize('stale', [False, True])
def test_moving_or_stale_feedback_cannot_bypass_settle_wait(monkeypatch, stale):
    from cleany_skill_executor import hardware_execution as module
    history, clock = stable_history(), [.7]
    if not stale:history.observe(700_000_001, {'a': .3, 'b': .2}, 0)
    values = {'hardware_endpoint_settle_duration_sec': .3,
              'hardware_endpoint_settle_timeout_sec': .4,
              'hardware_endpoint_settle_velocity_rad_s': .02,'hardware_guard_max_age_sec': .25}
    def monitor():
        if not stale:history.observe(round(clock[0]*1e9), {'a':clock[0], 'b':.2}, 0)
        return NS(name=['a','b'],position=[clock[0] if not stale else .1,.2])
    fake = NS(_stability_history=history,value=values.__getitem__,monitor=monitor,
              node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=round((clock[0]+.3 if stale else clock[0])*1e9)))))
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(module.time,'sleep',lambda seconds:clock.__setitem__(0,clock[0]+seconds))
    with pytest.raises(RuntimeError,match='did not settle'):
        module.HardwareExecution.wait_for_settle(fake,('a','b'))


@pytest.mark.parametrize('distance', [.005, .02, .3])
def test_short_correction_retiming_keeps_geometry_and_driver_limits(distance):
    from cleany_skill_executor.core.joint_path import executable_joint_path
    limits = {'a': PhysicalJointLimit(-2., 2., .8, 1.2, 2048., 1., 0., 0., 4095.)}
    points = [(0.,), (distance,)]
    old, old_duration = bounded_path(('a',), points, limits, velocity=.8, acceleration=1.2)
    new, duration = bounded_path(('a',), points, limits, velocity=.8, acceleration=1.2,
                                 minimum_duration_sec=.3)
    assert new == old and .3 <= duration <= old_duration
    assert duration >= executable_joint_path(points).minimum_duration([.8*.8], [1.2*.8])*1.05
    assert max(abs(p.derivatives[0]/duration) for p in new) <= .8*.8+1e-9
    assert new[0].derivatives == new[-1].derivatives == (0.,)
    if distance == .005:
        assert old_duration == 1. and duration == .3


@pytest.mark.parametrize('floor', [0., -1., float('nan'), float('inf'), True])
def test_invalid_correction_floor_is_rejected(floor):
    limits = {'a': PhysicalJointLimit(-2., 2., .8, 1.2, 2048., 1., 0., 0., 4095.)}
    with pytest.raises(ValueError):
        bounded_path(('a',), [(0.,), (.01,)], limits, velocity=.8, acceleration=1.,
                     minimum_duration_sec=floor)


def test_hardware_model_cache_preserves_live_checks_and_invalidates_changes(monkeypatch):
    from cleany_skill_executor import hardware_execution as module
    parse_calls, limit_calls, live_calls = [], [], []
    parse = module.ET.fromstring
    monkeypatch.setattr(module.ET, 'fromstring', lambda xml: (parse_calls.append(xml) or parse(xml)))
    monkeypatch.setattr(module, 'physical_limits', lambda xml, arm: (limit_calls.append((xml, arm)) or {'arm': arm}))
    physical, planning = ['<robot name="a"/>'], ['<robot name="a"/>']
    enabled = [False]
    def call(client, request):
        live_calls.append(client)
        if client == 'hardware':
            return NS(component=[NS(class_type='cleany_feetech_hardware/FeetechSystem',state=NS(label='active'))])
        if client == 'controllers':
            return NS(controller=[NS(name='left_'+part+'_controller',state='active') for part in ('arm','gripper')])
        if client == 'physical':return NS(values=[NS(string_value=physical[0])])
        return NS(values=[NS(string_value=planning[0]),NS(bool_value=enabled[0])])
    fake = NS(node=NS(get_parameter=lambda _: NS(value=False), call=call,
                     motion=NS(set_robot_description=lambda _: None)), monitor=lambda: None,
              hardware='hardware', controllers='controllers',parameters='physical',moveit_parameters='planning')
    for _ in range(2):assert module.HardwareExecution.verify(fake,'left') == {'arm':'left'}
    assert len(live_calls) == 8 and len(parse_calls) == 2 and len(limit_calls) == 1
    physical[0] = planning[0] = '<robot name="b"/>'
    module.HardwareExecution.verify(fake,'left')
    assert len(parse_calls) == 4 and len(limit_calls) == 2
    planning[0] = '<robot name="different"/>'
    with pytest.raises(RuntimeError,match='models differ'):module.HardwareExecution.verify(fake,'left')
    planning[0] = physical[0]
    enabled[0] = True
    with pytest.raises(RuntimeError,match='planning only'):module.HardwareExecution.verify(fake,'left')


def test_identical_urdf_preserves_fk_structure_and_verification(monkeypatch):
    from cleany_skill_executor import physical_moveit_adapter as module
    from cleany_skill_executor.core.physical_grasp_selection import REQUIRED_JOINT_NAMES
    calls = []
    monkeypatch.setattr(module, 'UrdfChain', lambda *args: (calls.append(args) or object()))
    xml = '<robot>'+''.join(f'<joint name="{n}" type="revolute"><limit lower="-2" upper="2" velocity="1"/></joint>'
                          for n in REQUIRED_JOINT_NAMES)+'</robot>'
    adapter = module.MoveItGraspAdapter(object(), ik_client=object(),validity_client=object(),plan_client=object())
    adapter.set_robot_description(xml)
    chains = adapter._local_fk
    adapter._verified_fk_arms.add('left')
    adapter.set_robot_description(xml)
    assert adapter._local_fk is chains and 'left' in adapter._verified_fk_arms and len(calls) == 2
    adapter.set_robot_description(xml.replace('upper="2"','upper="3"'))
    assert not adapter._verified_fk_arms and len(calls) == 4
