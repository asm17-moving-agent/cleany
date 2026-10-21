from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AllowedCollisionEntry, CollisionObject, PlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES,
    InfrastructureError,
    JointSolution,
    REQUIRED_JOINT_NAMES,
)
from cleany_skill_executor.curobo_adapter import CuRoboGraspAdapter
from cleany_skill_executor.curobo_model import CuRoboConfig, validation_states
from cleany_skill_executor.curobo_scene import world_from_scene
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig


def test_sphere_fitting_requires_valid_configuration():
    assert CuRoboConfig().sphere_budget_per_link == 8
    assert CuRoboConfig(sphere_budget_per_link=0).sphere_budget_per_link == 0
    for budget in (-1, None, True, 2.5):
        with pytest.raises(ValueError, match='Sphere budget'):
            CuRoboConfig(sphere_budget_per_link=budget)
    with pytest.raises(ValueError):
        CuRoboConfig(spheres_per_link=0)
    with pytest.raises(ValueError):
        CuRoboConfig(sphere_surface_radius_m=float("nan"))


def test_collision_validation_includes_segment_interior_and_caps_work():
    states = validation_states([(0.0, 0.0), (0.1, -0.04)], 0.025, 10)
    assert len(states) == 5
    assert states[2] == pytest.approx((0.05, -0.02))
    with pytest.raises(ValueError, match="budget"):
        validation_states([(0.0,), (2.0,)], 0.01, 5)
    with pytest.raises(ValueError, match="invalid"):
        validation_states([(float("nan"),)], 0.01, 5)
    with pytest.raises(ValueError):
        CuRoboConfig(validation_step_rad=0.0)


def scene_with_box():
    scene = PlanningScene()
    obj = CollisionObject()
    obj.id = "target"
    obj.header.frame_id = "base_link"
    primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.1, 0.2, 0.3])
    pose = Pose()
    pose.orientation.w = 1.0
    pose.position.x = 0.4
    obj.primitives = [primitive]
    obj.primitive_poses = [pose]
    scene.world.collision_objects = [obj]
    return scene


def test_world_conversion_preserves_dimensions_and_quaternion_order():
    scene = scene_with_box()
    world = world_from_scene(scene)
    assert world["cuboid"]["target/primitive/0"] == {
        "dims": [0.1, 0.2, 0.3],
        "pose": [0.4, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    }
    scene.world.collision_objects[0].header.frame_id = "camera"
    with pytest.raises(ValueError, match="base_link"):
        world_from_scene(scene)


def test_world_contacts_are_kept_for_gpu_acceptance_and_only_omitted_from_cost():
    scene = scene_with_box()
    scene.allowed_collision_matrix.entry_names = ["target", "left_gripper_frame"]
    scene.allowed_collision_matrix.entry_values = [
        AllowedCollisionEntry(enabled=[False, True]),
        AllowedCollisionEntry(enabled=[True, False]),
    ]
    assert world_from_scene(scene, optimization=True)["cuboid"] == {}
    assert "target/primitive/0" in world_from_scene(scene)["cuboid"]
    # Conversion must not mutate the real scene/ACM used by MoveIt validation.
    assert len(scene.world.collision_objects) == 1
    assert scene.allowed_collision_matrix.entry_values[0].enabled == [False, True]


def test_octomap_is_rejected_instead_of_ignored():
    scene = PlanningScene()
    scene.world.octomap.octomap.data = [1]
    with pytest.raises(ValueError, match="OctoMap"):
        world_from_scene(scene)


def test_scene_key_ignores_only_timestamps_not_geometry():
    scene = scene_with_box()
    key = CuRoboGraspAdapter._scene_key(scene)
    scene.world.octomap.header.stamp.sec = 100
    scene.world.octomap.octomap.header.stamp.sec = 101
    scene.world.collision_objects[0].header.stamp.sec = 102
    assert CuRoboGraspAdapter._scene_key(scene) == key
    scene.world.collision_objects[0].primitive_poses[0].position.x += 0.01
    assert CuRoboGraspAdapter._scene_key(scene) != key


def adapter_with_path():
    names = ARM_JOINT_NAMES["left"]
    engine = SimpleNamespace(
        plan=lambda *args: (names, [(0.0,) * 5, (0.1, 0.0, 0.0, 0.0, 0.0)]),
        valid_states=lambda names, positions, *args: [True] * len(positions),
    )
    adapter = CuRoboGraspAdapter(
        object(),
        MoveItAdapterConfig(),
        engine=engine,
        scene_client=object(),
        ik_client=object(),
        fk_client=object(),
        validity_client=object(),
        plan_client=object(),
    )
    state = JointState(name=list(REQUIRED_JOINT_NAMES), position=[0.0] * 12)
    adapter.set_current_state(state)
    adapter._snapshot = lambda: PlanningScene()
    goal = JointSolution(names, (0.1, 0.0, 0.0, 0.0, 0.0))
    return adapter, goal


def test_gpu_plan_is_rejected_when_only_segment_interior_collides():
    adapter, goal = adapter_with_path()
    checked = []

    def valid(names, positions, *args):
        checked.extend(q[0] for q in positions)
        return [not 0.04 < q[0] < 0.06 for q in positions]

    adapter._engine.valid_states = valid
    assert adapter.plan("left", goal, None) is None
    assert 0.05 in checked


def test_gpu_plan_preserves_full_state_opening_and_checks_every_state():
    adapter, goal = adapter_with_path()
    adapter.set_gripper_position("right", 1.2)
    checked = []

    def valid(names, positions, state, *args):
        checked.append((state, positions))
        return [True] * len(positions)

    adapter._engine.valid_states = valid
    adapter._validity_client = SimpleNamespace(
        call_async=lambda *args: pytest.fail("CPU collision call")
    )
    path = adapter.plan("left", goal, None)
    assert path.joint_names == ARM_JOINT_NAMES["left"]
    assert len(checked) == 1  # all five states in ONE GPU call
    assert len(checked[0][1]) == 5
    assert checked[0][0]["right_gripper_joint"] == 1.2
    assert checked[0][0]["right_shoulder_yaw_joint"] == 0.0


def test_gpu_plan_rejects_scene_change_and_wrong_endpoint():
    adapter, goal = adapter_with_path()
    scenes = iter([PlanningScene(), scene_with_box()])
    adapter._snapshot = lambda: next(scenes)
    with pytest.raises(InfrastructureError, match="scene changed"):
        adapter.plan("left", goal, None)
    adapter._snapshot = lambda: PlanningScene()
    with pytest.raises(InfrastructureError, match="endpoint"):
        adapter.plan("left", JointSolution(goal.names, (0.2, 0.0, 0.0, 0.0, 0.0)), None)


def test_cancellation_discards_gpu_result():
    adapter, goal = adapter_with_path()
    original = adapter._engine.plan

    def cancel_during_call(*args):
        adapter.cancel_active()
        return original(*args)

    adapter._engine.plan = cancel_during_call
    with pytest.raises(InterruptedError):
        adapter.plan("left", goal, None)


def test_canceled_gpu_adapter_can_be_reused_for_a_new_goal():
    adapter, _ = adapter_with_path()
    adapter.cancel_active()
    with pytest.raises(InterruptedError):
        adapter._check_active()
    adapter.set_current_state(adapter._current_state)
    adapter.set_deadline(float('inf'))
    adapter._check_active()
    adapter._check_cancel()
    adapter.set_deadline(0.0)
    with pytest.raises(InterruptedError):
        adapter._check_active()


def test_contact_policy_is_pair_specific_and_explicit_false_overrides_defaults():
    from cleany_skill_executor.curobo_collision import ContactPolicy

    scene = scene_with_box()
    acm = scene.allowed_collision_matrix
    acm.entry_names = ["target", "finger", "arm"]
    acm.entry_values = [
        AllowedCollisionEntry(enabled=row)
        for row in ([False, True, False], [True, False, False], [False, False, False])
    ]
    acm.default_entry_names = ["target"]
    acm.default_entry_values = [True]
    policy = ContactPolicy(acm)
    assert policy.allows("finger", "target")
    assert not policy.allows("arm", "target")
    assert policy.allows("unlisted", "target")
    assert not policy.allows("base_link", "left_shoulder_yaw_link")
    acm.entry_values[0].enabled = [False]
    with pytest.raises(ValueError, match="square"):
        ContactPolicy(acm)


def test_contact_policy_combines_defaults_conservatively_and_rejects_asymmetry():
    from cleany_skill_executor.curobo_collision import ContactPolicy

    acm = PlanningScene().allowed_collision_matrix
    acm.default_entry_names = ["object", "arm"]
    acm.default_entry_values = [True, False]
    assert not ContactPolicy(acm).allows("object", "arm")
    assert ContactPolicy(acm).allows("object", "finger")
    acm.entry_names = ["object", "arm"]
    acm.entry_values = [
        AllowedCollisionEntry(enabled=[False, True]),
        AllowedCollisionEntry(enabled=[False, False]),
    ]
    with pytest.raises(ValueError, match="asymmetric"):
        ContactPolicy(acm)


def test_shadow_validation_never_silently_accepts_disagreement(monkeypatch):
    from dataclasses import replace
    from cleany_skill_executor.physical_moveit_adapter import MoveItGraspAdapter

    adapter, goal = adapter_with_path()
    adapter._gpu_config = replace(adapter._gpu_config, moveit_shadow_validation=True)
    monkeypatch.setattr(MoveItGraspAdapter, "state_is_valid", lambda *args: False)
    with pytest.raises(InfrastructureError, match="disagreement"):
        adapter.plan("left", goal, None)


def test_unsupported_scene_is_rejected_before_any_gpu_work():
    adapter, goal = adapter_with_path()
    scene = PlanningScene()
    scene.world.octomap.octomap.data = [1]
    adapter._snapshot = lambda: scene
    adapter._engine.plan = lambda *args: pytest.fail(
        "unsupported scene reached planner"
    )
    with pytest.raises(InfrastructureError, match="OctoMap"):
        adapter.plan("left", goal, None)


def test_expired_action_does_not_start_gpu_work():
    adapter, goal = adapter_with_path()
    adapter.set_deadline(0.0)
    adapter._engine.plan = lambda *args: pytest.fail("expired request reached GPU")
    with pytest.raises(InterruptedError):
        adapter.plan("left", goal, None)
