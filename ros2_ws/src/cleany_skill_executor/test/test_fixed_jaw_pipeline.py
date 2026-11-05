from types import SimpleNamespace as NS
import numpy as np
import pytest
from cleany_interfaces.msg import GraspCandidate
from cleany_skill_executor.collection_pipeline import CollectionPipeline
from cleany_skill_executor.core.fixed_jaw_clearance import Bounds
from cleany_skill_executor.core.hardware_execution import CollisionValidationError, ExecutionCanceled

JAW = Bounds((.008, -.006, -.005), (.016, .06, .005))


def candidate_message():
    message = GraspCandidate()
    message.target_object.obb_pose.orientation.w = 1.
    message.target_object.obb_size.x = .04
    message.target_object.obb_size.y = .03
    message.target_object.obb_size.z = .04
    return message


def guarded_pipeline(monkeypatch):
    node = object.__new__(CollectionPipeline)
    node.config = {'fixed_jaw_clearance_enabled': True,
                   'fixed_jaw_collision_meshes': ['finger.stl']}
    node.description = 'verified runtime URDF'
    node.real = NS()
    events = []
    node.event = lambda name, **values: events.append((name, values.copy()))
    monkeypatch.setattr('cleany_skill_executor.collection_pipeline.load_fixed_jaw_bounds',
                        lambda *args: JAW)
    class Chain:
        def __init__(self, *args):
            pass
        def pose(self, q):
            return np.asarray((q['command_x'], q.get('command_y', 0.), 0.)), np.eye(3)
    monkeypatch.setattr('cleany_skill_executor.collection_pipeline.UrdfChain', Chain)
    selected = NS(selected_arm='right', selected_candidate=candidate_message())
    return node, selected, events


def test_actual_biased_command_contact_rejects_even_when_the_nominal_goal_is_clear(monkeypatch):
    node, selected, events = guarded_pipeline(monkeypatch)
    def approach(*args):
        node.real._approach_clearance_guard({'command_x': .03}, 0, 'hold_bridge')
        node.real._approach_clearance_guard({'command_x': .01}, 4, 'command_path')
        pytest.fail('Collision must prevent the trajectory goal')
    node._approach_target = approach
    report = {}
    with pytest.raises(CollisionValidationError, match='sample=4'):
        node.approach_target(selected, report)
    assert node.real._approach_clearance_guard is None
    assert report['fixed_jaw_approach_clearance']['outcome'] == 'collision'
    assert report['fixed_jaw_approach_clearance']['checked_samples'] == 2
    assert events[-1][0] == 'fixed_jaw_approach_clearance'


def test_clear_path_is_executed_and_guard_is_removed_before_other_stages(monkeypatch):
    node, selected, _ = guarded_pipeline(monkeypatch)
    def approach(*args):
        for i in range(5):
            node.real._approach_clearance_guard({'command_x': .03}, i, 'command_path')
        return 'terminal feedback'
    node._approach_target = approach
    report = {}
    assert node.approach_target(selected, report) == 'terminal feedback'
    assert node.real._approach_clearance_guard is None
    assert report['fixed_jaw_approach_clearance']['outcome'] == 'executed'
    assert report['fixed_jaw_approach_clearance']['minimum_gap_m'] == pytest.approx(.018)


def test_cancellation_always_removes_the_scoped_guard(monkeypatch):
    node, selected, _ = guarded_pipeline(monkeypatch)
    node._approach_target = lambda *args: (_ for _ in ()).throw(ExecutionCanceled('canceled'))
    with pytest.raises(ExecutionCanceled):
        node.approach_target(selected, {})
    assert node.real._approach_clearance_guard is None
