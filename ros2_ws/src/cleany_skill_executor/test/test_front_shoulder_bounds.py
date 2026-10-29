"""Front-sector limits apply to proposals and commands even without collisions."""
from types import SimpleNamespace as NS

import pytest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cleany_skill_executor.core.hardware_execution import (
    JointRangeValidationError, PhysicalJointLimit,
)
from cleany_skill_executor.core.physical_grasp_selection import (
    ARM_JOINT_NAMES, REQUIRED_JOINT_NAMES, JointSolution, PlannedPath,
)
from cleany_skill_executor.hardware_execution import HardwareExecution
from cleany_skill_executor.physical_moveit_adapter import MoveItAdapterConfig, MoveItGraspAdapter


@pytest.fixture(params=[('left', 1., (-2.16, 0.)),
                        ('right', -1., (0., 2.0033789089786476))])
def sector(request):
    return request.param


def test_planner_rejects_forbidden_ik_and_path_interior_before_collision_service(sector):
    arm, sign, bounds = sector
    names = ARM_JOINT_NAMES[arm]
    adapter = MoveItGraspAdapter(object(), MoveItAdapterConfig(validate_motion_paths=True),
        ik_client=object(), fk_client=object(), validity_client=object(), plan_client=object())
    adapter.set_current_state(JointState(name=list(REQUIRED_JOINT_NAMES),
                                        position=[0.]*len(REQUIRED_JOINT_NAMES)))
    adapter._joint_limits = {n: (-4., 4., 1.) for n in REQUIRED_JOINT_NAMES}
    adapter._joint_limits[names[0]] = (*bounds, 1.)
    adapter._verify_local_fk = lambda *_: None
    adapter._call = lambda *_: pytest.fail('Forbidden shoulder must fail before collision RPC')
    legal = (-sign*.2, 0., 0., 0., 0.)
    forbidden = (sign*.1, 0., 0., 0., 0.)
    assert not adapter.state_is_valid(arm, JointSolution(names, forbidden))
    assert adapter.last_state_validity_rejection['reason'] == 'joint_limits'
    assert adapter._validated_cubic_path(arm, PlannedPath(names, (legal, forbidden, legal))) is None
    adapter._batch_validity = NS(batch_size=32, check=lambda *_: NS(first_invalid_index=-1))
    accepted = adapter._validated_cubic_path(arm, PlannedPath(names, (legal, (0.,)*5)))
    assert accepted is not None
    assert all(bounds[0] <= q[0] <= bounds[1] for q in accepted.points)


def test_return_transport_blocks_forbidden_midpoint_with_collision_checks_disabled(monkeypatch, sector):
    import cleany_skill_executor.hardware_execution as module

    arm, sign, bounds = sector
    names = ARM_JOINT_NAMES[arm]
    legal = (-sign*.2, 0., 0., 0., 0.)
    forbidden = (sign*.1, 0., 0., 0., 0.)
    state = JointState(name=list(names), position=list(legal))
    limits = {n: PhysicalJointLimit(-3., 3., 1., 2., 2048., -1., 0., 0., 4095.)
              for n in names}
    limits[names[0]] = PhysicalJointLimit(*bounds, 1., 2., 2048., -1., 0., 0., 4095.)
    settings = {'hardware_velocity_rad_s': 1., 'hardware_acceleration_rad_s2': 2.,
                'hardware_path_step_rad': .01, 'hardware_path_simplification_tolerance_rad': .002,
                'hardware_start_tolerance_rad': .005, 'hardware_start_blend_max_rad': .03}
    fake = NS(verify=lambda _: limits, value=settings.__getitem__, arm_holds={})
    monkeypatch.setattr(HardwareExecution, '_wait_before_execution', lambda *_: state)
    monkeypatch.setattr(module, 'ActionClient', lambda *_: pytest.fail('Forbidden command dispatched'))
    trajectory = JointTrajectory(joint_names=list(names), points=[
        JointTrajectoryPoint(positions=list(q)) for q in (legal, forbidden, legal)])
    with pytest.raises(JointRangeValidationError) as rejected:
        HardwareExecution._execute(fake, arm, trajectory, 'arm', check_collisions=False)
    assert rejected.value.joint_name == names[0]
