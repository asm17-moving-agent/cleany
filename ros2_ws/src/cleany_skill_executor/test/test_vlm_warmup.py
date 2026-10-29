import threading
import time
from types import SimpleNamespace

import pytest
from std_srvs.srv import SetBool

from cleany_skill_executor.physical_grasp_selection_node import GraspSelectionNode


def node_fixture():
    calls = []
    params = {'curobo_ik_vlm_warmup_window_sec': 30.,
              'curobo_ik_vlm_warmup_arms': ['left', 'right']}
    node = SimpleNamespace(_vlm_warmup_enabled=True, _vlm_warmup_until=0.,
        _vlm_warmup_lock=threading.Lock(), _vlm_warmup_completed=set(),
        _vlm_warmup_failed=set(), _description_ready=True, _goal_active=False,
        _adapter_lock=threading.RLock(), _current_joint_state=lambda: 'fresh',
        get_parameter=lambda key: SimpleNamespace(value=params[key]),
        get_logger=lambda: SimpleNamespace(info=lambda _: None, warning=lambda _: None))
    node._vlm_warmup_active = lambda: GraspSelectionNode._vlm_warmup_active(node)
    node._adapter = SimpleNamespace(set_current_state=lambda _: None,
        set_deadline=lambda _: None, set_progress_callback=lambda _: None,
        warmup=lambda arms, **kw: calls.append((arms, kw['include_aim'], kw['continue_work']())))
    return node, calls


def request(node, active):
    return GraspSelectionNode._vlm_warmup_request(node, SetBool.Request(data=active), SetBool.Response())


def test_vlm_warmup_is_demand_driven_grasp_only_and_reuses_completed_arms():
    node, calls = node_fixture()
    GraspSelectionNode._try_vlm_warmup(node)
    assert not calls
    assert request(node, True).success
    GraspSelectionNode._try_vlm_warmup(node)
    assert calls == [(['left'], False, True)]
    GraspSelectionNode._try_vlm_warmup(node)
    assert calls[-1] == (['right'], False, True)
    request(node, False)
    request(node, True)
    GraspSelectionNode._try_vlm_warmup(node)
    assert len(calls) == 2


@pytest.mark.parametrize('stop', ['vlm_finished', 'expired', 'goal_active', 'missing_description'])
def test_no_second_arm_after_window_ends_or_real_goal_arrives(stop):
    node, calls = node_fixture()
    request(node, True)
    GraspSelectionNode._try_vlm_warmup(node)
    if stop == 'vlm_finished':
        request(node, False)
    elif stop == 'expired':
        node._vlm_warmup_until = time.monotonic() - 1.
    elif stop == 'goal_active':
        node._goal_active = True
    else:
        node._description_ready = False
    GraspSelectionNode._try_vlm_warmup(node)
    assert calls == [(['left'], False, True)]


def test_real_goal_can_interrupt_between_warmup_phases_without_losing_gpu_lock():
    node, calls = node_fixture()
    def warmup(arms, **kw):
        node._goal_active = True
        assert not kw['continue_work']()
        raise InterruptedError('real goal arrived')
    node._adapter.warmup = warmup
    request(node, True)
    GraspSelectionNode._try_vlm_warmup(node)
    assert not node._vlm_warmup_completed and not node._vlm_warmup_failed
    assert node._adapter_lock.acquire(blocking=False)
    node._adapter_lock.release()


def test_optional_warmup_failure_does_not_poison_selection_or_spin_retries():
    node, calls = node_fixture()
    def fail(arms, **kw):
        calls.append(arms)
        raise RuntimeError('GPU unavailable')
    node._adapter.warmup = fail
    request(node, True)
    for _ in range(3):
        GraspSelectionNode._try_vlm_warmup(node)
    assert calls == [['left'], ['right']]
    assert not node._goal_active


@pytest.mark.parametrize('preferred,allowed,expected', [
    ('', ['right', 'left'], 'right'),
    ('left', ['right', 'left'], 'left'),
    ('right', ['left', 'right'], 'right'),
    ('left', ['right'], 'right'),
    ('invalid', ['right', 'left'], 'right'),
])
def test_vlm_warmup_prioritizes_known_arm_within_configured_subset(preferred, allowed, expected):
    node, calls = node_fixture()
    original = node.get_parameter
    node.get_parameter = lambda key: (SimpleNamespace(value=allowed)
        if key == 'curobo_ik_vlm_warmup_arms' else original(key))
    GraspSelectionNode._remember_warmup_arm(node, preferred)
    request(node, True)
    GraspSelectionNode._try_vlm_warmup(node)
    assert calls == [([expected], False, True)]
    GraspSelectionNode._remember_warmup_arm(node, expected)
    GraspSelectionNode._try_vlm_warmup(node)
    assert sum(arms == [expected] for arms, *_ in calls) == 1


def test_vlm_warmup_rejected_goal_does_not_change_preference():
    from rclpy.action import GoalResponse
    node, _ = node_fixture()
    node._goal_lock = threading.Lock()
    node._validate_candidates = lambda *_: None
    node.get_parameter = lambda _: SimpleNamespace(value='base_link')
    node._vlm_warmup_preferred_arm = 'right'
    goal = SimpleNamespace(required_arm='left', candidates=[])
    node._goal_active = True
    assert GraspSelectionNode._on_goal(node, goal) == GoalResponse.REJECT
    assert node._vlm_warmup_preferred_arm == 'right'
    node._goal_active = False
    assert GraspSelectionNode._on_goal(node, goal) == GoalResponse.ACCEPT
    assert node._vlm_warmup_preferred_arm == 'left'
