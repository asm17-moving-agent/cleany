from types import SimpleNamespace as NS
import pytest
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from cleany_skill_executor.collection_pipeline import CollectionPipeline
from cleany_skill_executor.core.approach_path import CartesianPathUnavailable
from cleany_skill_executor.core.hardware_execution import CollisionValidationError, ExecutionCanceled

@pytest.fixture
def pipeline():
    node = object.__new__(CollectionPipeline)
    node.config = {'approach_collision_replanning_enabled': True,
                   'approach_collision_scene_refresh_attempts': 1,
                   'approach_collision_detour_attempts': 4,
                   'approach_collision_detour_offset_m': .02,
                   'approach_collision_final_straight_m': .04,
                   'approach_collision_max_extra_length_m': .04}
    target = Pose()
    target.orientation.w = 1.
    start = Pose()
    start.orientation.w, start.position.y = 1., .08
    selected = NS(selected_arm='right', fk_grasp_pose=target,
                  grasp_joint_state=JointState(name=['right_wrist_roll_joint'], position=[.2]))
    trace, report = [], {}
    node.pose = lambda _: start
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=100))
    node.real = NS(monitor=lambda: 'fresh feedback')
    node.scene_monitor = NS(wait=lambda **kwargs: trace.append(('scene', kwargs)))
    node.event = lambda name, **kwargs: trace.append((name, kwargs))
    return node, selected, trace, report


def collision(phase='command_path', index=2):
    return CollisionValidationError('predicted collision', phase=phase, sample_index=index)


def test_replanning_retains_the_goal_and_executes_only_the_first_valid_detour(pipeline):
    node, selected, trace, report = pipeline
    calls, sent = [], []
    def linear(arm, target, stage, roll, **kwargs):
        assert target is selected.fk_grasp_pose and roll == .2
        calls.append(kwargs.get('route'))
        if len(calls) <= 3:
            raise collision()
        sent.append(kwargs['route'].name)
        return 'arrived'
    node.linear = linear
    assert node.approach_target(selected, report) == 'arrived'
    assert len(calls) == 4 and sent == ['closing_minus']
    assert calls[2].waypoints[-1] == node.cartesian_pose(selected.fk_grasp_pose)
    state = report['approach_collision_replanning']
    assert state['scene_refreshes'] == 1 and state['detour_attempts'] == 2
    assert state['selected_route'] == 'closing_minus' and state['outcome'] == 'executed'
    assert len([entry for entry in trace if entry[0] == 'scene']) == 1


def test_unavailable_straight_cartesian_path_can_use_a_bounded_detour(pipeline):
    node, selected, _, report = pipeline
    def linear(*args, **kwargs):
        if 'route' not in kwargs:
            raise CartesianPathUnavailable('partial path', error_code=1, fraction=.4)
        return 'arrived'
    node.linear = linear
    assert node.approach_target(selected, report) == 'arrived'
    assert report['approach_collision_replanning']['detour_attempts'] == 1


@pytest.mark.parametrize('phase,index', [('hold_bridge', 2), ('command_path', 0), ('cartesian_path', 0)])
def test_blocked_entry_is_refreshed_once_without_a_futile_later_detour(pipeline, phase, index):
    node, selected, trace, report = pipeline
    calls = []
    def linear(*args, **kwargs):
        calls.append(kwargs)
        raise collision(phase, index)
    node.linear = linear
    with pytest.raises(CollisionValidationError):
        node.approach_target(selected, report)
    assert len(calls) == 2 and all('route' not in call for call in calls)
    assert report['approach_collision_replanning']['outcome'] == 'entry_blocked'


def test_a_fresh_map_can_clear_a_transient_entry_rejection(pipeline):
    node, selected, _, report = pipeline
    calls = []
    def linear(*args, **kwargs):
        calls.append(kwargs)
        if len(calls) == 1:
            raise collision('hold_bridge', 2)
        return 'arrived'
    node.linear = linear
    assert node.approach_target(selected, report) == 'arrived'
    assert report['approach_collision_replanning']['selected_route'] == 'straight_after_scene_refresh'
    assert report['approach_collision_replanning']['detour_attempts'] == 0


@pytest.mark.parametrize('error', [
    RuntimeError('Physical TCP endpoint outside tolerance'),
    RuntimeError('Stale feedback'), ExecutionCanceled('canceled'),
])
def test_execution_faults_and_endpoint_failures_never_enter_replanning(pipeline, error):
    node, selected, trace, report = pipeline
    node.linear = lambda *args, **kwargs: (_ for _ in ()).throw(error)
    with pytest.raises(type(error), match=str(error)):
        node.approach_target(selected, report)
    assert trace == []


def test_collision_after_communication_resume_cannot_restart_an_approach(pipeline):
    node, selected, trace, report = pipeline
    error = collision()
    error.motion_started = True
    node.linear = lambda *args, **kwargs: (_ for _ in ()).throw(error)
    with pytest.raises(CollisionValidationError):
        node.approach_target(selected, report)
    assert trace == []


def test_waiting_for_a_fresh_scene_checks_cancellation_without_sending_a_path(pipeline):
    node, selected, _, report = pipeline
    calls = []
    def linear(*args, **kwargs):
        calls.append(kwargs)
        raise collision()
    node.linear = linear
    node.real.monitor = lambda: (_ for _ in ()).throw(ExecutionCanceled('canceled during scene wait'))
    node.scene_monitor.wait = lambda **kwargs: kwargs['canceled']()
    with pytest.raises(ExecutionCanceled, match='during scene wait'):
        node.approach_target(selected, report)
    assert len(calls) == 1 and report['approach_collision_replanning']['detour_attempts'] == 0


def test_rejected_detours_stop_at_the_configured_attempt_limit(pipeline):
    node, selected, _, report = pipeline
    calls = []
    def linear(*args, **kwargs):
        calls.append(kwargs)
        raise collision()
    node.linear = linear
    with pytest.raises(RuntimeError, match='detours_exhausted'):
        node.approach_target(selected, report)
    assert len(calls) == 6
    assert report['approach_collision_replanning']['detour_attempts'] == 4


def test_final_grasp_entry_does_not_get_a_sideways_detour(pipeline):
    node, selected, _, report = pipeline
    node.pose('right').position.y = .03
    node.linear = lambda *args, **kwargs: (_ for _ in ()).throw(collision())
    with pytest.raises(RuntimeError, match='final_entry_or_length_limit'):
        node.approach_target(selected, report)
    assert report['approach_collision_replanning']['detour_attempts'] == 0


def test_disabled_replanning_preserves_the_single_original_approach(pipeline):
    node, selected, trace, report = pipeline
    node.config = {'approach_collision_replanning_enabled': False}
    node.linear = lambda *args, **kwargs: 'arrived'
    assert node.approach_target(selected, report) == 'arrived'
    assert trace == [] and report == {}


def test_collision_opt_out_only_applies_to_the_selected_straight_approach(pipeline):
    node, selected, trace, report = pipeline
    node.config = {'approach_collision_check_enabled': False}
    calls = []
    node.linear = lambda *args, **kwargs: calls.append((args, kwargs)) or 'arrived'
    assert node.approach_target(selected, report) == 'arrived'
    args, options = calls[0]
    assert args == ('right', selected.fk_grasp_pose, 'approach', .2)
    assert options == {'check_orientation': False, 'check_collisions': False}
    assert report == {'approach_collision_checks_enabled': False}
    assert trace == [('approach_collision_checks_disabled', {'arm': 'right'})]
    # The opt-out is an argument for one operation, not mutable execution state.
    node.config = {'approach_collision_replanning_enabled': False}
    node.approach_target(selected, {})
    assert 'check_collisions' not in calls[1][1]
