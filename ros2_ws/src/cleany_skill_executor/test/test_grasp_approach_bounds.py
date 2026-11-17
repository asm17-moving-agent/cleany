"""Approach-only shoulder policy with otherwise full calibrated travel."""
from types import SimpleNamespace as NS

import pytest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cleany_skill_executor.core.grasp_approach_bounds import (
    GraspApproachYawBounds, active_grasp_approach_yaw, grasp_approach_yaw_scope,
)
from cleany_skill_executor.core.hardware_execution import PhysicalJointLimit, JointRangeValidationError
from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, REQUIRED_JOINT_NAMES, JointSolution, PlannedPath,
)
from cleany_skill_executor.hardware_execution import HardwareExecution
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig, MoveItGraspAdapter

POLICY = GraspApproachYawBounds((-2.16, 0.), (0., 2.0033789089786476))


@pytest.fixture(params=[('left', 1.), ('right', -1.)])
def arm_sign(request):
    return request.param


def test_policy_scope_restored_on_cancellation_and_nested_calls():
    assert active_grasp_approach_yaw.get() is None
    with pytest.raises(InterruptedError), grasp_approach_yaw_scope(POLICY):
        with grasp_approach_yaw_scope(None):
            assert active_grasp_approach_yaw.get() is None
        assert active_grasp_approach_yaw.get() is POLICY
        raise InterruptedError('canceled')
    assert active_grasp_approach_yaw.get() is None


def test_rear_states_and_path_interiors_rejected_only_during_approach(arm_sign):
    arm, sign = arm_sign
    names = ARM_JOINT_NAMES[arm]
    adapter = MoveItGraspAdapter(object(), MoveItAdapterConfig(validate_motion_paths=True),
        ik_client=object(), fk_client=object(), validity_client=object(), plan_client=object())
    adapter._joint_limits = {n: (-3., 3., 1.) for n in REQUIRED_JOINT_NAMES}
    adapter.set_current_state(JointState(name=list(REQUIRED_JOINT_NAMES),
                                        position=[0.]*len(REQUIRED_JOINT_NAMES)))
    adapter._verify_local_fk = lambda *_: None
    adapter._batch_validity = NS(batch_size=32, check=lambda *_: NS(first_invalid_index=-1))
    adapter._call = lambda *_: NS(valid=True)
    rear = JointSolution(names, (sign*.2, 0., 0., 0., 0.))
    front = (-sign*.2, 0., 0., 0., 0.)
    path = PlannedPath(names, (rear.positions, front, rear.positions))
    assert adapter.state_is_valid(arm, rear)
    assert adapter._validated_cubic_path(arm, path) is not None
    with grasp_approach_yaw_scope(POLICY):
        assert not adapter.state_is_valid(arm, rear)
        assert adapter._validated_cubic_path(arm, path) is None
        assert not adapter.validate_approach_path(arm, path, None, None)
        constraint = adapter._grasp_approach_constraints(arm).joint_constraints[0]
        assert constraint.joint_name == names[0]
        assert constraint.position-constraint.tolerance_below == pytest.approx(
            POLICY.intersect(names[0], -3., 3.)[0])
    assert adapter._grasp_approach_constraints(arm) is None
    assert adapter.state_is_valid(arm, rear)
    assert adapter._validated_cubic_path(arm, path) is not None


def test_execution_rejects_rear_midpoint_even_without_collision_checks(monkeypatch, arm_sign):
    arm, sign = arm_sign
    names = ARM_JOINT_NAMES[arm]
    front = (-sign*.2, 0., 0., 0., 0.)
    rear = (sign*.2, 0., 0., 0., 0.)
    limits = {n: PhysicalJointLimit(-3., 3., 1., 2., 2048., -1., 0., 0., 4095.) for n in names}
    settings = {'hardware_velocity_rad_s': 1., 'hardware_acceleration_rad_s2': 2.,
        'hardware_path_step_rad': .01, 'hardware_path_simplification_tolerance_rad': .002,
        'hardware_start_tolerance_rad': .005, 'hardware_start_blend_max_rad': .03}
    fake = NS(verify=lambda _: limits, value=settings.__getitem__, arm_holds={})
    monkeypatch.setattr(HardwareExecution, '_wait_before_execution',
                        lambda *_: JointState(name=list(names), position=list(front)))
    trajectory = JointTrajectory(joint_names=list(names), points=[
        JointTrajectoryPoint(positions=list(q)) for q in (front, rear, front)])
    with grasp_approach_yaw_scope(POLICY), pytest.raises(JointRangeValidationError):
        HardwareExecution._execute(fake, arm, trajectory, 'arm', check_collisions=False)
    unchanged = HardwareExecution._with_grasp_approach_limits(arm, limits, None)
    assert unchanged is limits
    unchanged[names[0]].check(rear[0])


@pytest.mark.parametrize('entry', ['pregrasp', 'grasp', 'visual_grasp'])
@pytest.mark.parametrize('fail', [False, True])
def test_motion_entry_scopes_policy_and_restores_it(monkeypatch, entry, fail):
    from cleany_skill_executor.pregrasp_pipeline import PregraspPipeline
    from cleany_skill_executor.collection_pipeline import CollectionPipeline
    fake = NS(real=NS(grasp_approach_yaw=POLICY))
    def body(*_):
        assert active_grasp_approach_yaw.get() is POLICY
        if fail:
            raise InterruptedError('stop')
        return 'done'
    if entry == 'pregrasp':
        monkeypatch.setattr(PregraspPipeline, '_execute_pregrasp', body)
        call = lambda: PregraspPipeline.execute_pregrasp(fake, NS())
    elif entry == 'grasp':
        monkeypatch.setattr(CollectionPipeline, '_scoped_approach_target', body)
        call = lambda: CollectionPipeline.approach_target(fake, NS(), {})
    else:
        monkeypatch.setattr(CollectionPipeline, '_scoped_visual_approach', body)
        call = lambda: CollectionPipeline.visual_approach(fake, NS(), None, None, {})
    if fail:
        with pytest.raises(InterruptedError):
            call()
    else:
        assert call() == 'done'
    assert active_grasp_approach_yaw.get() is None


def test_gpu_validation_applies_front_policy_without_moveit_shadow(monkeypatch, arm_sign):
    import cleany_skill_executor.curobo_adapter as module
    arm, sign = arm_sign
    names = ARM_JOINT_NAMES[arm]
    solutions = [JointSolution(names, (v, 0., 0., 0., 0.)) for v in (-sign*.2, sign*.2)]
    monkeypatch.setattr(module, 'world_from_scene', lambda *_: object())
    fake = NS(_active_scene=NS(allowed_collision_matrix=object()),
        _engine=NS(valid_states=object()), _state=lambda: {},
        _compute=lambda *_: [True, True], _gpu_config=NS(moveit_shadow_validation=False),
        _config=MoveItAdapterConfig())
    with grasp_approach_yaw_scope(POLICY):
        assert module.CuRoboGraspAdapter._valid_batch(fake, arm, solutions) == [True, False]
    assert module.CuRoboGraspAdapter._valid_batch(fake, arm, solutions) == [True, True]
