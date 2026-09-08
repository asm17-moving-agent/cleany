from copy import deepcopy
from types import SimpleNamespace

import pytest
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    CollisionObject,
)

from cleany_skill_executor.core.grasp_selection import InfrastructureError
from cleany_skill_executor.planning_scene import TargetSceneTransaction
from cleany_skill_executor.planning_scene import SceneAwarePort
from cleany_skill_executor.planning_scene import support_patch_from_obb
from cleany_interfaces.msg import GraspCandidate


def support_candidate():
    candidate = GraspCandidate()
    candidate.header.frame_id = 'base_link'
    candidate.target_object.obb_pose.orientation.w = 1.
    candidate.target_object.obb_pose.position.z = .35
    candidate.target_object.obb_size.x = .10
    candidate.target_object.obb_size.y = .05
    candidate.target_object.obb_size.z = .02
    return candidate


def test_perceived_support_patch_top_matches_obb_bottom_without_gt():
    candidate = support_candidate()
    patch = support_patch_from_obb(candidate, 'target/support', .08)
    assert patch.header.frame_id == 'base_link'
    assert patch.primitives[0].dimensions == pytest.approx([.26, .21, .01])
    assert patch.primitive_poses[0].position.z + .005 == pytest.approx(.34)
    assert candidate.target_object.obb_pose.position.z == .35


@pytest.mark.parametrize('margin', [-1., 0., .31, float('nan')])
def test_invalid_perceived_support_patch_rejected(margin):
    with pytest.raises(ValueError, match='support patch'):
        support_patch_from_obb(support_candidate(), 'support', margin)


def test_support_patch_rejects_non_plane_aligned_obb():
    candidate = support_candidate()
    candidate.target_object.obb_pose.orientation.x = 1.
    candidate.target_object.obb_pose.orientation.w = 0.
    with pytest.raises(ValueError, match='upward'):
        support_patch_from_obb(candidate, 'support', .08)


def test_support_permission_never_allows_robot_contact():
    base = TargetSceneTransaction._with_allowed_links(
        AllowedCollisionMatrix(), 'target', {'target/support'})
    acm = TargetSceneTransaction._with_target_permissions(base, 'target', 'left')
    matrix = {name: dict(zip(acm.entry_names, row.enabled))
              for name, row in zip(acm.entry_names, acm.entry_values)}
    assert matrix['target']['target/support']
    assert matrix['target']['left_gripper_frame']
    assert not matrix['target/support']['left_gripper_frame']
    assert not matrix['target/support']['left_moving_jaw_link']


@pytest.mark.parametrize('attached', [False, True])
def test_support_patch_is_registered_and_removed_with_target(attached):
    get = object()
    transaction = TargetSceneTransaction(object(), apply_client=object(), get_client=get,
                                          support_patch_margin_m=.08)
    transaction._call = lambda client, request: SimpleNamespace(
        scene=SimpleNamespace(allowed_collision_matrix=AllowedCollisionMatrix()))
    scenes = []
    def apply(scene):
        scenes.append(deepcopy(scene))
        return SimpleNamespace(success=True)
    transaction._apply = apply
    transaction.begin(support_candidate(), 'target')
    assert [o.id for o in scenes[0].world.collision_objects] == ['target', 'target/perceived_support']
    if attached:
        transaction.attach_to('left')
    transaction.restore()
    removed = [o.id for scene in scenes for o in scene.world.collision_objects
               if o.operation == CollisionObject.REMOVE]
    assert 'target' in removed and 'target/perceived_support' in removed
    assert not transaction.active and not transaction._support_id


def test_matching_observed_mesh_replaces_only_target_primitive():
    from test_collision_geometry_cache import geometry_pair
    geometry, candidate = geometry_pair()
    candidate.target_object.obb_size.x = .08
    candidate.target_object.obb_size.y = .04
    candidate.target_object.obb_size.z = .02
    transaction = TargetSceneTransaction(object(), apply_client=object(), get_client=object(),
        support_patch_margin_m=.02, geometry_lookup=lambda _: geometry)
    transaction._call = lambda *args: SimpleNamespace(
        scene=SimpleNamespace(allowed_collision_matrix=AllowedCollisionMatrix()))
    scenes = []
    transaction._apply = lambda scene: scenes.append(deepcopy(scene)) or SimpleNamespace(success=True)
    transaction.begin(candidate, 'target')
    target, support = scenes[0].world.collision_objects
    assert target.primitives == [] and target.meshes == [geometry.mesh]
    assert target.mesh_poses == [geometry.mesh_pose]
    assert support.meshes == [] and len(support.primitives) == 1
    transaction.attach_to('left')
    assert scenes[-1].robot_state.attached_collision_objects[0].object.meshes == [geometry.mesh]


def test_missing_required_geometry_does_not_fall_back_to_box():
    transaction = TargetSceneTransaction(object(), apply_client=object(), get_client=object(),
        timeout_sec=.001, geometry_lookup=lambda _: None, spin_once=lambda _: None)
    transaction._call = lambda *args: SimpleNamespace(
        scene=SimpleNamespace(allowed_collision_matrix=AllowedCollisionMatrix()))
    transaction._apply = lambda _: pytest.fail('Missing required mesh must not apply a box')
    with pytest.raises(InfrastructureError, match='geometry unavailable'):
        transaction.begin(support_candidate(), 'target')


@pytest.mark.parametrize('raises', [False, True])
def test_open_clearance_restores_target_permissions_even_on_service_failure(raises):
    calls = []
    def check(*args):
        if raises:
            raise RuntimeError('transport failure')
        return False
    scene = SimpleNamespace(disallow_target_contacts=lambda: calls.append(None),
                            allow_contacts_for=lambda arm: calls.append(arm))
    port = SceneAwarePort(SimpleNamespace(open_grasp_is_valid=check), scene)
    port.set_target_contacts('left')
    if raises:
        with pytest.raises(RuntimeError):
            port.open_grasp_is_valid('left', object(), 1.4)
    else:
        assert not port.open_grasp_is_valid('left', object(), 1.4)
    assert calls == ['left', None, 'left']


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

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
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
    transaction = TargetSceneTransaction(
        object(),
        apply_client=client,
        get_client=object(),
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


def test_attach_moves_world_obb_to_selected_gripper():
    requests = []

    class Client(_ApplyClient):
        def call_async(self, request):
            requests.append(request)
            return super().call_async(request)

    transaction = TargetSceneTransaction(
        object(),
        apply_client=Client([SimpleNamespace(success=True)]),
        get_client=object(),
    )
    collision = CollisionObject()
    collision.header.frame_id = 'base_link'
    collision.id = 'target'
    transaction._object_id = 'target'
    transaction._collision_object = collision

    transaction.attach_to('left')

    scene = requests[0].scene
    assert scene.world.collision_objects == []
    attached = scene.robot_state.attached_collision_objects[0]
    assert attached.link_name == 'left_gripper_frame'
    assert attached.object.id == 'target'
    assert set(attached.touch_links) == {
        'left_gripper_frame', 'left_moving_jaw_link'
    }


def test_restore_detaches_then_removes_the_recreated_world_obb():
    requests = []

    class Client(_ApplyClient):
        def call_async(self, request):
            requests.append(deepcopy(request))
            return super().call_async(request)

    transaction = TargetSceneTransaction(
        object(),
        apply_client=Client([SimpleNamespace(success=True)] * 2),
        get_client=object(),
    )
    transaction._object_id = 'target'
    transaction._saved_acm = AllowedCollisionMatrix()
    transaction._attached = True

    transaction.restore()

    scene = requests[0].scene
    assert scene.world.collision_objects == []
    attached = scene.robot_state.attached_collision_objects[0]
    assert attached.object.id == 'target'
    assert attached.object.operation == CollisionObject.REMOVE
    assert requests[1].scene.robot_state.attached_collision_objects == []
    removed = requests[1].scene.world.collision_objects[0]
    assert removed.id == 'target'
    assert removed.operation == CollisionObject.REMOVE
    assert not transaction.active


def test_failed_world_cleanup_retries_without_detaching_twice():
    transaction = TargetSceneTransaction(
        object(), get_client=object(), apply_client=_ApplyClient([
            SimpleNamespace(success=True), SimpleNamespace(success=False),
            SimpleNamespace(success=True),
        ]),
    )
    transaction._object_id = 'target'
    transaction._attached = True
    with pytest.raises(InfrastructureError, match='detached target'):
        transaction.restore()
    assert transaction.active and not transaction._attached
    transaction.restore()
    assert not transaction.active
