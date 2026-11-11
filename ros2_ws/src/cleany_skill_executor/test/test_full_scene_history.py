"""Exercise the real feedback adapter and complete scene-barrier contract."""
from types import SimpleNamespace as NS
import threading

import pytest

from cleany_skill_executor.core.joint_stability import JointStabilityHistory
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES
from cleany_skill_executor.hardware_execution import HardwareExecution


ARMS = [name for arm in ('left', 'right')
        for name in (*ARM_JOINT_NAMES[arm], f'{arm}_gripper_joint')]
HEAD = ['head_pan_joint', 'head_tilt_joint']


def frame(stamp, positions, generation=0):
    return NS(joint_names=list(positions), interface_values=[NS(
        interface_names=(['position', 'feedback_stamp_ns', 'communication_recovery_count']
                         if name in ARMS else ['position']),
        values=([value, float(stamp), float(generation)] if name in ARMS else [value]))
        for name, value in positions.items()])


@pytest.mark.parametrize('change', ['none', 'head', 'opposite_arm', 'missing_head',
    'nan_head', 'duplicate_head_interface', 'malformed_head', 'replayed', 'stale', 'recovery'])
def test_callback_preserves_all_fourteen_joints_and_real_acquisition(change):
    positions = dict.fromkeys(ARMS + HEAD, .1)
    hw = NS(_stability_history=JointStabilityHistory(), contact_lock=threading.Lock(),
        contact_histories={}, fault='', reuse_scene_barrier=True, motion_submissions=0,
        communication_recovery_count=0, value=lambda _: .3)
    for stamp in range(100_000_000, 700_000_001, 100_000_000):
        HardwareExecution.on_contact_feedback(hw, frame(stamp, positions))
    now = 800_000_000
    changed = dict(positions)
    if change == 'head': changed['head_pan_joint'] += .001
    if change == 'opposite_arm': changed['right_shoulder_yaw_joint'] += .001
    if change == 'missing_head': changed.pop('head_pan_joint')
    if change == 'nan_head': changed['head_pan_joint'] = float('nan')
    message = frame(now, changed, generation=1 if change == 'recovery' else 0)
    if change == 'duplicate_head_interface':
        message.interface_values[-1] = NS(interface_names=['position', 'position'], values=[.1, .1])
    if change == 'malformed_head': message.interface_values[-1].values = []
    if change in ('replayed', 'stale'):
        message = frame(700_000_000, positions)
        now = 1_100_000_000
    HardwareExecution.on_contact_feedback(hw, message)
    # The independent measured snapshot still contains all 14 joints.
    measured = {**positions, **{k: v for k, v in changed.items() if v == v}}
    hw.wait_for_settle = lambda _: NS(name=list(measured), position=list(measured.values()))
    hw.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=now)))
    barrier = HardwareExecution.capture_scene_barrier(hw, 'left')
    assert barrier['stamp_ns'] == (100_000_000 if change == 'none' else now)
    assert len(barrier['positions']) == 14
    assert not hw.fault


def test_republished_complete_frame_does_not_create_a_stationary_window():
    hw = NS(_stability_history=JointStabilityHistory(), contact_lock=threading.Lock(),
        contact_histories={}, fault='')
    positions = dict.fromkeys(ARMS + HEAD, .1)
    for _ in range(40):
        HardwareExecution.on_contact_feedback(hw, frame(700_000_000, positions))
    assert hw._stability_history.unchanged_since(positions, now_ns=800_000_000,
        duration_sec=.3, maximum_age_sec=.25) is None
