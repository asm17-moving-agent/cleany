"""Exercise stop evidence through the real JointState callback, without DDS."""
from concurrent.futures import Future
from types import SimpleNamespace as NS

import pytest
from sensor_msgs.msg import JointState

from cleany_manipulation_bt.backend import ExecutionContext, OperationError
from cleany_manipulation_bt.joint_feedback import JointFeedback, StationaryWindow
from cleany_manipulation_bt.ros_operations import MujocoOperations
from cleany_skill_executor.core.grasp_selection import REQUIRED_JOINT_NAMES
from cleany_skill_executor.manipulation.models import Error, Goal


@pytest.fixture
def feedback_node(monkeypatch):
    import cleany_manipulation_bt.ros_operations as module

    wall = [10.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: wall[0])
    values = {'bt_stop_timeout_sec': .8, 'arm_stationary_samples': 3,
              'arm_stationary_velocity_rad_s': .02,
              'bt_stationary_duration_sec': .2, 'bt_stop_feedback_max_age_sec': .15,
              'bt_feedback_max_age_sec': 2.0}
    node = object.__new__(MujocoOperations)
    node._joint_positions, node._joint_velocities = {}, {}
    node._joint_feedback = JointFeedback(REQUIRED_JOINT_NAMES)
    node.submissions, node.handles = [], []
    node.get_parameter = lambda name: NS(value=values[name])
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(wall[0] * 10**9)))
    context = ExecutionContext(Goal('m', 't', 'e', 'collect_trash', 's', 1, 'trash_right'))

    def publish(names=REQUIRED_JOINT_NAMES, *, velocities=None, stamp=None):
        message = JointState(name=list(names), position=[0.0] * len(names),
                             velocity=[0.0] * len(names) if velocities is None else velocities)
        stamp = int(wall[0] * 10**9) if stamp is None else stamp
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
        node._on_joints(message)

    publish()
    return node, context, wall, publish, values


@pytest.mark.parametrize('names', [
    ('left_gripper_joint', 'right_gripper_joint'),
    tuple(name for name in REQUIRED_JOINT_NAMES if name.startswith('left_')),
])
def test_partial_messages_cannot_refresh_old_joint_stop_evidence(feedback_node, monkeypatch, names):
    import cleany_manipulation_bt.ros_operations as module

    node, context, wall, publish, _ = feedback_node
    wall[0] += 10.0  # The last full arm feedback is now ten seconds old.

    def spin(*_, **__):
        wall[0] += .05
        publish(names)

    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    assert not node._assess_stationary(context, cancel=True)
    assert not context.stop_confirmed


@pytest.mark.parametrize('split', [False, True])
@pytest.mark.parametrize('cancel', [False, True])
def test_stop_accepts_continuous_new_feedback_in_full_or_split_messages(feedback_node, monkeypatch, split, cancel):
    import cleany_manipulation_bt.ros_operations as module

    node, context, wall, publish, values = feedback_node
    steps = [0]

    def spin(*_, **__):
        wall[0] += .05
        steps[0] += 1
        if split:
            side = 'left_' if steps[0] % 2 else 'right_'
            publish(tuple(name for name in REQUIRED_JOINT_NAMES if name.startswith(side)))
        else:
            publish()

    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    assert node._assess_stationary(context, cancel=cancel)
    assert context.stop_confirmed
    assert wall[0] >= 10.05 + values['bt_stationary_duration_sec']


@pytest.mark.parametrize('moving_joint', REQUIRED_JOINT_NAMES)
def test_any_moving_arm_or_gripper_joint_prevents_confirmation(feedback_node, monkeypatch, moving_joint):
    import cleany_manipulation_bt.ros_operations as module

    node, context, wall, publish, _ = feedback_node
    velocities = [.1 if name == moving_joint else 0.0 for name in REQUIRED_JOINT_NAMES]

    def spin(*_, **__):
        wall[0] += .05
        publish(velocities=velocities)

    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    assert not node._assess_stationary(context, cancel=True)


@pytest.mark.parametrize('problem', ['missing', 'short', 'nan', 'inf', 'duplicate',
                                    'frozen', 'backward', 'old', 'future'])
def test_invalid_joint_feedback_cannot_reuse_cached_zero_velocity(feedback_node, monkeypatch, problem):
    import cleany_manipulation_bt.ros_operations as module

    node, context, wall, publish, _ = feedback_node
    initial_stamp = int(wall[0] * 10**9)

    def spin(*_, **__):
        wall[0] += .05
        names = list(REQUIRED_JOINT_NAMES)
        velocities = [0.0] * len(names)
        stamp = None
        if problem == 'missing':
            velocities = []
        elif problem == 'short':
            velocities.pop()
        elif problem in ('nan', 'inf'):
            velocities[0] = float(problem)
        elif problem == 'duplicate':
            names.append(names[0])
            velocities.append(0.0)
        elif problem == 'frozen':
            stamp = initial_stamp
        elif problem == 'backward':
            stamp = initial_stamp - 1
        elif problem == 'old':
            stamp = int((wall[0] - 1.0) * 10**9)
        elif problem == 'future':
            stamp = int((wall[0] + 1.0) * 10**9)
        publish(names, velocities=velocities, stamp=stamp)

    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    assert not node._assess_stationary(context, cancel=True)


def test_each_counted_round_requires_new_samples_from_every_joint(feedback_node, monkeypatch):
    import cleany_manipulation_bt.ros_operations as module

    node, context, wall, publish, values = feedback_node
    values['bt_stop_feedback_max_age_sec'] = 2.0  # Isolate sample reuse from expiry.
    steps = [0]

    def spin(*_, **__):
        wall[0] += .05
        steps[0] += 1
        publish() if steps[0] == 1 else publish(('left_gripper_joint', 'right_gripper_joint'))

    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    assert not node._assess_stationary(context, cancel=True)


def test_feedback_received_before_stop_does_not_count_despite_newer_sensor_stamp(feedback_node, monkeypatch):
    import cleany_manipulation_bt.ros_operations as module

    node, context, wall, publish, values = feedback_node
    values['bt_stop_feedback_max_age_sec'] = 2.0
    publish(stamp=int((wall[0] + .1) * 10**9))

    def spin(*_, **__):
        wall[0] += .05
        publish(('unrelated_joint',))

    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    assert not node._assess_stationary(context, cancel=True)


def test_fast_sample_burst_does_not_replace_stationary_duration(feedback_node, monkeypatch):
    import cleany_manipulation_bt.ros_operations as module

    node, context, wall, publish, values = feedback_node

    def spin(*_, **__):
        wall[0] += .005
        publish()

    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    assert node._assess_stationary(context, cancel=True)
    assert wall[0] >= 10.005 + values['bt_stationary_duration_sec']


def test_delayed_poll_cannot_bridge_a_feedback_gap_with_one_new_message(feedback_node, monkeypatch):
    import cleany_manipulation_bt.ros_operations as module

    node, context, wall, publish, values = feedback_node
    steps = [0]

    def spin(*_, **__):
        steps[0] += 1
        wall[0] += .3 if steps[0] == 3 else .05
        publish()

    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    assert node._assess_stationary(context, cancel=True)
    assert wall[0] >= 10.4 + values['bt_stationary_duration_sec'] - 1e-9


@pytest.mark.parametrize('interruption', ['moving', 'stale', 'active_action'])
def test_interrupted_stationary_window_restarts_before_confirmation(feedback_node, monkeypatch, interruption):
    import cleany_manipulation_bt.ros_operations as module

    node, context, wall, publish, values = feedback_node
    steps = [0]
    if interruption == 'active_action':
        node.quiescent = lambda: steps[0] != 3

    def spin(*_, **__):
        wall[0] += .05
        steps[0] += 1
        if interruption == 'stale' and 4 <= steps[0] <= 8:
            return
        velocities = [0.0] * len(REQUIRED_JOINT_NAMES)
        if interruption == 'moving' and steps[0] == 3:
            velocities[0] = .1
        publish(velocities=velocities)

    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    assert node._assess_stationary(context, cancel=True)
    restart = .45 if interruption == 'stale' else .2
    assert wall[0] >= 10.0 + restart + values['bt_stationary_duration_sec'] - 1e-9


def test_cancel_is_sent_once_and_stop_waits_for_terminal_action_and_feedback(feedback_node, monkeypatch):
    import cleany_manipulation_bt.ros_operations as module
    from action_msgs.msg import GoalStatus

    node, context, wall, publish, values = feedback_node
    terminal, canceled = Future(), []
    node.handles.append((NS(cancel_goal_async=lambda: canceled.append(wall[0])), terminal))
    steps = [0]

    def spin(*_, **__):
        wall[0] += .05
        steps[0] += 1
        publish()
        if steps[0] == 5:
            terminal.set_result(NS(status=GoalStatus.STATUS_CANCELED))

    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    assert node._assess_stationary(context, cancel=True)
    assert len(canceled) == 1
    assert wall[0] >= 10.25 + values['bt_stationary_duration_sec'] - 1e-9


def test_feedback_ready_at_stop_deadline_cannot_confirm_or_keep_previous_confirmation(feedback_node, monkeypatch):
    import cleany_manipulation_bt.ros_operations as module

    node, context, wall, publish, values = feedback_node
    values['bt_stop_timeout_sec'] = .25
    context.stop_confirmed = True

    def spin(*_, **__):
        wall[0] += .05
        publish()

    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    assert not node._assess_stationary(context, cancel=True)
    assert not context.stop_confirmed


@pytest.mark.parametrize('names', [
    ('left_gripper_joint', 'right_gripper_joint'),
    tuple(name for name in REQUIRED_JOINT_NAMES if name.startswith('left_')),
])
def test_motion_guard_detects_stale_joints_despite_partial_new_messages(feedback_node, names):
    node, context, wall, publish, _ = feedback_node
    wall[0] += 3.0
    publish(names)
    node.execution_context, node.current_node, node.deadline = context, 'MoveToPregrasp', 20.0
    node._clock_value, node._clock_progress_wall = int(wall[0] * 10**9), wall[0]
    with pytest.raises(OperationError) as raised:
        node._guard()
    assert raised.value.error == Error.HARDWARE_ERROR
    assert 'right_shoulder_yaw_joint' in str(raised.value)


@pytest.mark.parametrize('setting,value', [
    ('max_age_sec', 0.0), ('max_age_sec', float('nan')),
    ('minimum_duration_sec', 0.0), ('minimum_duration_sec', float('inf')),
    ('maximum_velocity', -1.0), ('maximum_velocity', float('nan')),
    ('required_samples', 0),
])
def test_stationary_window_rejects_invalid_limits(setting, value):
    limits = dict(barrier_ns=1, started_at=1.0, max_age_sec=.5,
                  maximum_velocity=.02, required_samples=5, minimum_duration_sec=.25)
    limits[setting] = value
    with pytest.raises(ValueError):
        StationaryWindow(**limits)
