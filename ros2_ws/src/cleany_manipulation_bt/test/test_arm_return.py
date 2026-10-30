"""Opening, cancellation and stop gates with no motor or ROS transport access."""
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS
import pytest
from cleany_manipulation_bt.arm_return import load_home, run_return, selected_arms
from cleany_manipulation_bt.backend import Observation
from cleany_skill_executor.core.physical_grasp_selection import ARM_JOINT_NAMES

ROOT = Path(__file__).resolve().parents[4]
HOME_FILE = ROOT / 'configs/robot/arm_return_home.json'


class FakeOperations:
    def __init__(self):
        self.config = json.loads((ROOT / 'ros2_ws/src/cleany_skill_executor/test/data/supervised_collection_fixture.json').read_text())
        self.calls = []
        self.jaw_calls = []
        self.carrying = None
        self.releasing = False
        self.stop_confirmed = self.initial_stop = True
        self.failure = ''
        self.started = threading.Event()
        self.block = False
        self.at_home = False
        self.real = NS(monitor=self.monitor, verify=self.verify,
                       value=lambda name: .25 if name == 'hardware_guard_max_age_sec' else .005,
                       fail_stop=lambda: self.calls.append('latched_stop'))
        self.scene_monitor = NS(wait=lambda: None)
        self.scene = NS(restore=lambda: self.calls.append('restore_scene'))
        self.motion = NS(set_gripper_position=lambda *_: None)

    def value(self, _):
        return 180.

    def event(self, *args, **kwargs):
        pass

    def quiescent(self):
        return True

    def destroy_node(self):
        pass

    def check_execution(self):
        if self.execution_context.abort.is_set():
            raise RuntimeError('Canceled')

    def monitor(self):
        # Lost-contact evidence must never prevent this explicit opening.
        assert self.releasing
        if self.failure == 'guard':
            raise RuntimeError('Hardware fault')
        home = load_home(str(HOME_FILE))
        names = [name for arm in ARM_JOINT_NAMES.values() for name in arm]
        values = [q for arm in home.values() for q in arm] if self.at_home else [0.] * len(names)
        return NS(name=names, position=values)

    def verify(self, arm):
        def check(_):
            if self.failure == 'limit':
                raise ValueError('Outside physical joint range')
        return {name: NS(check=check) for name in (*ARM_JOINT_NAMES[arm], f'{arm}_gripper_joint')}

    def _assess_stationary(self, *args, **kwargs):
        return self.initial_stop

    def jaw(self, arm, position, stage, *, check_collisions=True):
        self.jaw_calls.append((stage, arm, check_collisions))
        self.calls.append(('open' if stage.startswith('manual_open') else 'close', arm))
        self.started.set()
        if self.block:
            self.execution_context.abort.wait(5.)
            self.check_execution()
        if self.failure == 'open' and stage.startswith('manual_open'):
            raise RuntimeError('Opening failed')
        if (self.failure == 'opening_collision' and stage.startswith('manual_open')
                and check_collisions):
            raise RuntimeError('Opening trajectory failed collision validation')

    def detach(self, *, assume_released):
        assert assume_released and self.carrying is not None
        self.calls.append('detach_assumed')
        self.carrying = None

    def release_grasp_scene_freeze(self):
        self.calls.append('unfreeze')

    def move_arm_return(self, arm, home, gripper_closed):
        assert gripper_closed == self.config['arms'][arm]['gripper']['closed_joint_rad']
        self.calls.append(('return', arm, home))
        if self.failure == 'return':
            raise RuntimeError('Planning failed')

    def stop(self, context):
        self.calls.append('stop')
        context.abort.set()
        if self.failure == 'stop':
            raise RuntimeError('Stop assessment failed')
        return Observation(stop_confirmed=self.stop_confirmed)


def test_open_both_before_returning_to_saved_pose_and_ignore_lost_contact():
    ops = FakeOperations()
    ops.carrying = {'arm': 'right', 'release_confirmed': False}
    home = load_home(str(HOME_FILE))
    result = run_return(ops, selected_arms('both'), home, threading.Event(), lambda _: None)
    assert result.success and result.stop_confirmed
    assert ops.calls == [('open', 'left'), ('open', 'right'), 'detach_assumed',
        'restore_scene', 'unfreeze', ('return', 'left', home['left']),
        ('return', 'right', home['right']), 'stop']
    assert not ops.releasing and ops.carrying is None


def test_single_arm_does_not_open_or_command_the_other_arm():
    ops = FakeOperations()
    home = load_home(str(HOME_FILE))
    assert run_return(ops, ('right',), home, threading.Event(), lambda _: None).success
    assert [c for c in ops.calls if isinstance(c, tuple)] == [
        ('open', 'right'), ('return', 'right', home['right'])]


@pytest.mark.parametrize('arm', ['left', 'right', 'both'])
@pytest.mark.parametrize('collision_checks', [None, True, False])
def test_manual_opening_uses_shared_collision_policy_and_preserves_close_checks(
        arm, collision_checks):
    ops = FakeOperations()
    ops.at_home = True
    ops.failure = 'opening_collision'
    ops.config.pop('gripper_opening_collision_check_enabled', None)
    if collision_checks is not None:
        ops.config['gripper_opening_collision_check_enabled'] = collision_checks
    arms = selected_arms(arm)
    result = run_return(ops, arms, load_home(str(HOME_FILE)), threading.Event(), lambda _: None)
    assert result.stop_confirmed and ops.calls[-1] == 'stop'
    if collision_checks is not False:
        assert not result.success and 'collision validation' in result.message
        assert ops.jaw_calls == [('manual_open_' + arms[0], arms[0], True)]
    else:
        assert result.success
        assert ops.jaw_calls == [
            *(('manual_open_' + chosen, chosen, False) for chosen in arms),
            *(('manual_close_' + chosen, chosen, True) for chosen in arms),
        ]


def test_already_home_still_opens_grippers_and_confirms_stop_without_a_zero_path():
    ops = FakeOperations()
    ops.at_home = True
    result = run_return(ops, ('left', 'right'), load_home(str(HOME_FILE)),
                        threading.Event(), lambda _: None)
    assert result.success and result.stop_confirmed
    assert [c for c in ops.calls if isinstance(c, tuple)] == [
        ('open', 'left'), ('open', 'right'), ('close', 'left'), ('close', 'right')]


@pytest.mark.parametrize('failure', ['guard', 'limit', 'open', 'return', 'stop', 'initial_stop'])
def test_failures_stop_and_never_report_success(failure):
    ops = FakeOperations()
    ops.failure = failure
    ops.initial_stop = failure != 'initial_stop'
    result = run_return(ops, ('right',), load_home(str(HOME_FILE)), threading.Event(), lambda _: None)
    assert not result.success
    assert 'stop' in ops.calls
    if failure in ('guard', 'limit', 'initial_stop'):
        assert ('open', 'right') not in ops.calls
    if failure == 'open':
        assert not any(c[0] == 'return' for c in ops.calls if isinstance(c, tuple))
    if failure == 'stop':
        assert not result.stop_confirmed and 'latched_stop' in ops.calls


def test_unconfirmed_final_stop_cannot_succeed():
    ops = FakeOperations()
    ops.stop_confirmed = False
    result = run_return(ops, ('left',), load_home(str(HOME_FILE)), threading.Event(), lambda _: None)
    assert not result.success and not result.stop_confirmed


def test_cancel_before_opening_sends_no_motion_and_still_assesses_stop():
    ops = FakeOperations()
    abort = threading.Event()
    abort.set()
    assert not run_return(ops, ('left',), load_home(str(HOME_FILE)), abort, lambda _: None).success
    assert ops.calls == ['stop']


def test_payload_on_unselected_arm_prevents_scene_cleanup_or_motion():
    ops = FakeOperations()
    ops.carrying = {'arm': 'left'}
    result = run_return(ops, ('right',), load_home(str(HOME_FILE)), threading.Event(), lambda _: None)
    assert not result.success and ops.calls == ['stop']


@pytest.mark.parametrize('value', [None, [], [0.] * 4, [0.] * 6, [float('nan')] * 5,
                                  [float('inf')] * 5, [True] * 5, ['0'] * 5])
def test_corrupt_home_is_rejected(tmp_path, value):
    path = tmp_path / 'home.json'
    path.write_text(json.dumps({'frame': 'base_link', 'arms': {'left': value, 'right': [0.] * 5}}))
    with pytest.raises(ValueError):
        load_home(str(path))


def test_unknown_arm_rejected():
    with pytest.raises(ValueError):
        selected_arms('head')


@pytest.mark.parametrize('guard_state,fresh', [('fault', True), ('ready', True), ('ready', False)])
def test_latched_fault_gives_recovery_steps_without_another_stop_or_motion(guard_state, fresh):
    ops = FakeOperations()
    ops.real.fault = 'Operator software stop'
    ops.real.guard = {'state': guard_state}
    ops.real.guard_time = time.monotonic() if fresh else 0.
    stages = []
    result = run_return(ops, ('right',), load_home(str(HOME_FILE)), threading.Event(), stages.append)
    assert not result.success and not result.stop_confirmed and ops.calls == []
    assert stages == ['blocked_before_motion']
    assert 'make manipulation-restart' in result.message
    assert ('make collection-recover-control' in result.message) == (guard_state != 'ready' or not fresh)
    assert ops.real.fault == 'Operator software stop'
