from types import SimpleNamespace

import pytest
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from cleany_interfaces.msg import GraspCandidate

from cleany_skill_executor.core.grasp_selection import InfrastructureError
from cleany_skill_executor.planning_scene import TargetSceneTransaction, support_patch_from_obb


class _Future:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _ApplyClient:
    def __init__(self, responses):
        self._responses = iter(responses)
        self.requests = []

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return _Future(next(self._responses))


def test_only_evaluated_gripper_links_are_allowed_to_touch_target():
    original = AllowedCollisionMatrix()
    original.entry_names = ['base_link', 'right_gripper_frame']
    for _ in original.entry_names:
        entry = AllowedCollisionEntry()
        entry.enabled = [False, False]
        original.entry_values.append(entry)

    updated = TargetSceneTransaction._with_target_permissions(
        original, 'target', 'left'
    )
    matrix = {
        row_name: dict(zip(updated.entry_names, row.enabled))
        for row_name, row in zip(updated.entry_names, updated.entry_values)
    }
    assert matrix['target']['left_gripper_frame'] is True
    assert matrix['target']['left_moving_jaw_link'] is True
    assert matrix['target']['base_link'] is False
    assert matrix['target']['right_gripper_frame'] is False
    assert original.entry_names == ['base_link', 'right_gripper_frame']


def test_failed_restore_keeps_transaction_state_for_retry():
    client = _ApplyClient(
        [SimpleNamespace(success=False), SimpleNamespace(success=True)]
    )
    existing = PlanningScene()
    existing.world.collision_objects = [CollisionObject(id='target')]
    fetched = _ApplyClient([SimpleNamespace(scene=existing)])
    transaction = TargetSceneTransaction(
        object(),
        apply_client=client,
        get_client=fetched,
    )
    transaction._object_id = 'target'
    transaction._saved_acm = AllowedCollisionMatrix()

    with pytest.raises(InfrastructureError, match='restore planning scene'):
        transaction.restore()

    assert transaction._object_id == 'target'
    assert transaction._saved_acm is not None
    transaction.restore()
    assert transaction._object_id == ''
    assert transaction._saved_acm is None


def test_failed_restore_response_recovers_when_scene_is_already_clean():
    saved = AllowedCollisionMatrix()
    cleaned = PlanningScene()
    cleaned.allowed_collision_matrix = saved
    applied = _ApplyClient([SimpleNamespace(success=False)])
    fetched = _ApplyClient([SimpleNamespace(scene=cleaned)])
    transaction = TargetSceneTransaction(
        object(), apply_client=applied, get_client=fetched,
    )
    transaction._object_id = 'target'
    transaction._support_id = 'target_support'
    transaction._saved_acm = saved

    transaction.restore()

    assert not transaction.active
    assert transaction._support_id == ''
    assert transaction._saved_acm is None
    assert fetched.requests[0].components.components == (
        PlanningSceneComponents.WORLD_OBJECT_NAMES
        | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    )


@pytest.mark.parametrize('unconfirmed', ['support', 'attached', 'acm', 'query_failure'])
def test_failed_restore_requires_absent_owned_objects_and_original_permissions(unconfirmed):
    from moveit_msgs.msg import AttachedCollisionObject

    saved = AllowedCollisionMatrix()
    observed = PlanningScene()
    if unconfirmed == 'support':
        observed.world.collision_objects = [CollisionObject(id='target/perceived_support')]
    elif unconfirmed == 'attached':
        attached = AttachedCollisionObject()
        attached.object.id = 'target'
        observed.robot_state.attached_collision_objects = [attached]
    elif unconfirmed == 'acm':
        observed.allowed_collision_matrix.entry_names = ['base_link']
        observed.allowed_collision_matrix.entry_values = [AllowedCollisionEntry(enabled=[True])]
    applied = _ApplyClient([SimpleNamespace(success=False)])
    fetched = _ApplyClient([None if unconfirmed == 'query_failure'
                           else SimpleNamespace(scene=observed)])
    transaction = TargetSceneTransaction(object(), apply_client=applied, get_client=fetched)
    transaction._object_id = 'target'
    transaction._support_id = 'target/perceived_support'
    transaction._saved_acm = saved
    transaction._attached = unconfirmed == 'attached'

    with pytest.raises(InfrastructureError, match='restore planning scene'):
        transaction.restore()

    assert transaction.active
    assert transaction._support_id == 'target/perceived_support'
    assert transaction._saved_acm is saved
    if transaction._attached:
        assert fetched.requests[0].components.components & PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS


def test_support_patch_touches_observed_bottom_and_restores_only_owned_objects():
    candidate = GraspCandidate()
    candidate.header.frame_id = 'base_link'
    candidate.target_object.obb_pose.orientation.w = 1.
    candidate.target_object.obb_pose.position.z = .5
    candidate.target_object.obb_size.x = .06
    candidate.target_object.obb_size.y = .08
    candidate.target_object.obb_size.z = .1
    patch = support_patch_from_obb(candidate, 'target_support', .08)
    assert patch.primitives[0].dimensions == pytest.approx([.22, .24, .01])
    assert patch.primitive_poses[0].position.z + .005 == pytest.approx(.45)
    applied = _ApplyClient([SimpleNamespace(success=True), SimpleNamespace(success=True)])
    original = PlanningScene()
    fetched = _ApplyClient([SimpleNamespace(scene=original)])
    transaction = TargetSceneTransaction(object(), apply_client=applied, get_client=fetched,
                                         support_patch_margin_m=.08)
    transaction.begin(candidate, 'target')
    support_id = applied.requests[0].scene.world.collision_objects[1].id
    transaction.restore()
    restored = applied.requests[-1].scene
    assert restored.is_diff and restored.robot_state.is_diff
    assert [(o.id, o.operation) for o in restored.world.collision_objects] == [
        ('target', CollisionObject.REMOVE), (support_id, CollisionObject.REMOVE)]
    assert not restored.world.octomap.octomap.data
    assert restored.allowed_collision_matrix == original.allowed_collision_matrix
    candidate.target_object.obb_pose.orientation.w = 0.
    candidate.target_object.obb_pose.orientation.x = 1.
    candidate.target_object.obb_pose.orientation.w = 0.
    with pytest.raises(ValueError, match='upward'):
        support_patch_from_obb(candidate, 'target_support', .08)


def test_invalid_support_does_not_open_transaction_or_poison_next_request():
    candidate = GraspCandidate()
    candidate.header.frame_id = 'base_link'
    candidate.target_object.obb_pose.orientation.x = 1.
    candidate.target_object.obb_pose.orientation.w = 0.
    candidate.target_object.obb_size.x = .06
    candidate.target_object.obb_size.y = .08
    candidate.target_object.obb_size.z = .1
    applied = _ApplyClient([SimpleNamespace(success=True), SimpleNamespace(success=True)])
    fetched = _ApplyClient([SimpleNamespace(scene=PlanningScene())])
    transaction = TargetSceneTransaction(object(), apply_client=applied, get_client=fetched,
                                         support_patch_margin_m=.08)
    with pytest.raises(ValueError, match='upward'):
        transaction.begin(candidate, 'invalid')
    transaction.restore()
    assert not transaction.active
    assert not applied.requests and not fetched.requests
    candidate.target_object.obb_pose.orientation.x = 0.
    candidate.target_object.obb_pose.orientation.w = 1.
    transaction.begin(candidate, 'valid')
    transaction.restore()
    assert not transaction.active
