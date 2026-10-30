from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
from geometry_msgs.msg import Pose, Vector3
from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix, PlanningScene
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from cleany_skill_executor.collection_pipeline import CollectionPipeline
from cleany_skill_executor.core.collection import configuration_issues
from cleany_skill_executor.planning_scene import collision_pair_is_allowed

@pytest.fixture(params=['left', 'right'])
def payload_scene(request):
    arm, object_id = request.param, 'physical_payload_current_execution'
    names = ['base_link', 'top_base_link', 'left_moving_jaw_link', 'right_moving_jaw_link',
             '<octomap>', 'named_obstacle', 'other_payload', object_id]
    original = AllowedCollisionMatrix(entry_names=names,
        entry_values=[AllowedCollisionEntry(enabled=[False]*len(names)) for _ in names])
    original.entry_values[0].enabled[1] = original.entry_values[1].enabled[0] = True
    scene = PlanningScene()
    scene.allowed_collision_matrix = deepcopy(original)
    applied, events = [], []
    node = NS(config={}, carrying=None, payload_future=None, releasing=False,
              get_scene=object(), payload_selected=None, payload_evidence_time=0.,
              get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: Header().stamp)),
              pose=lambda _: Pose(), measured=lambda: JointState(
                  name=[f'{arm}_wrist_roll_joint'], position=[.3]),
              event=lambda stage, **fields: events.append((stage, fields)))
    def apply(diff):
        applied.append(deepcopy(diff))
        if diff.allowed_collision_matrix.entry_names:
            scene.allowed_collision_matrix = deepcopy(diff.allowed_collision_matrix)
        for attached in diff.robot_state.attached_collision_objects:
            scene.robot_state.attached_collision_objects = (
                [] if attached.object.operation == attached.object.REMOVE else [deepcopy(attached)])
    node.apply = apply
    node.call = lambda *_: NS(scene=deepcopy(scene))
    selected = NS(selected_arm=arm, selected_candidate=NS(header=Header(frame_id='base_link')),
                  selected_object=NS(obb_size=Vector3(x=.1, y=.05, z=.03), obb_pose=Pose()))
    return node, selected, object_id, original, scene, applied, events


@pytest.mark.parametrize('enabled', [None, True, False])
def test_only_current_payload_map_pair_is_exempt_until_confirmed_release(payload_scene, enabled):
    node, selected, object_id, original, scene, applied, events = payload_scene
    if enabled is not None:
        node.config['held_object_octomap_collision_check_enabled'] = enabled
    CollectionPipeline.attach(node, selected, object_id)
    assert len(applied) == 1 and applied[0].world.collision_objects == []
    current = scene.allowed_collision_matrix
    assert collision_pair_is_allowed(current, object_id, '<octomap>') is (enabled is False)
    for i, first in enumerate(original.entry_names):
        for j, second in enumerate(original.entry_names):
            if {first, second} == {object_id, '<octomap>'}:
                continue
            assert current.entry_values[i].enabled[j] == original.entry_values[i].enabled[j]
    assert not collision_pair_is_allowed(current, 'right_moving_jaw_link', '<octomap>')
    assert not collision_pair_is_allowed(current, 'other_payload', '<octomap>')
    assert not collision_pair_is_allowed(current, object_id, 'named_obstacle')
    assert not collision_pair_is_allowed(current, object_id, 'base_link')
    with pytest.raises(RuntimeError, match='before physical release'):
        CollectionPipeline.detach(node)
    assert len(applied) == 1  # A failure with a held object retains ownership/permission.
    node.carrying['release_confirmed'] = True
    CollectionPipeline.detach(node)
    assert node.carrying is None and scene.allowed_collision_matrix == original
    stages = [stage for stage, _ in events]
    if enabled is False:
        assert stages == ['attach', 'held_object_octomap_collision_check_disabled',
                          'held_object_octomap_collision_permission_restored', 'detach']
    else:
        assert stages == ['attach', 'detach']


def test_permission_must_be_confirmed_in_the_authoritative_scene(payload_scene):
    node, selected, object_id, original, scene, _, _ = payload_scene
    node.config['held_object_octomap_collision_check_enabled'] = False
    def stale_reply(*_):
        result = deepcopy(scene)
        result.allowed_collision_matrix = deepcopy(original)
        return NS(scene=result)
    node.call = stale_reply
    with pytest.raises(RuntimeError, match='did not confirm held-object OctoMap'):
        CollectionPipeline.attach(node, selected, object_id)
    assert node.carrying is not None and node.carrying['payload_octomap_saved_acm'] == original


def test_empty_acm_is_rejected_before_creating_an_unrestorable_exemption(payload_scene):
    node, selected, object_id, _, scene, applied, _ = payload_scene
    node.config['held_object_octomap_collision_check_enabled'] = False
    scene.allowed_collision_matrix = AllowedCollisionMatrix()
    with pytest.raises(RuntimeError, match='active target collision matrix'):
        CollectionPipeline.attach(node, selected, object_id)
    assert not applied and node.carrying is None


def test_failed_detach_keeps_permission_snapshot_for_retry(payload_scene):
    node, selected, object_id, original, _, _, _ = payload_scene
    node.config['held_object_octomap_collision_check_enabled'] = False
    CollectionPipeline.attach(node, selected, object_id)
    node.carrying['release_confirmed'] = True
    def failed(_):
        raise RuntimeError('scene update failed')
    node.apply = failed
    with pytest.raises(RuntimeError, match='scene update failed'):
        CollectionPipeline.detach(node)
    assert node.carrying is not None and node.carrying['payload_octomap_saved_acm'] == original


def test_unconfirmed_restoration_does_not_clear_payload_ownership(payload_scene):
    node, selected, object_id, _, scene, _, _ = payload_scene
    node.config['held_object_octomap_collision_check_enabled'] = False
    CollectionPipeline.attach(node, selected, object_id)
    allowed = deepcopy(scene.allowed_collision_matrix)
    node.carrying['release_confirmed'] = True
    node.call = lambda *_: NS(scene=NS(allowed_collision_matrix=allowed))
    with pytest.raises(RuntimeError, match='permission restoration'):
        CollectionPipeline.detach(node)
    assert node.carrying is not None


@pytest.mark.parametrize('value', ['false', 0, None])
def test_permission_flag_requires_explicit_boolean(value):
    root = Path(__file__).resolve().parents[4]
    settings = json.loads((root/'ros2_ws/src/cleany_skill_executor/test/data/supervised_collection_fixture.json').read_text())
    settings['held_object_octomap_collision_check_enabled'] = value
    assert 'held_object_octomap_collision_check_enabled must be boolean' in configuration_issues(
        settings)


def test_readback_requires_both_sides_of_the_explicit_pair():
    matrix = AllowedCollisionMatrix(entry_names=['payload', '<octomap>'], entry_values=[
        AllowedCollisionEntry(enabled=[False, True]), AllowedCollisionEntry(enabled=[False, False])])
    assert not collision_pair_is_allowed(matrix, 'payload', '<octomap>')
    matrix.entry_values[1].enabled[0] = True
    assert collision_pair_is_allowed(matrix, 'payload', '<octomap>')
    assert not collision_pair_is_allowed(matrix, 'another_payload', '<octomap>')
