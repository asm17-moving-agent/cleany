from cleany_skill_executor.core.arm_health import AVAILABLE, UNAVAILABLE, UNKNOWN, ArmHealth, ArmHealthConfig


def test_arm_requires_active_controllers_and_fresh_finite_feedback():
    h = ArmHealth(ArmHealthConfig(controllers=('arm_controller',), joints=('a', 'b'), joint_timeout_s=.3,
                                  controller_timeout_s=1.))
    assert h.status(0.)[0] == UNKNOWN
    h.controller_states({'arm_controller': 'inactive'}, 0.)
    assert h.status(0.)[0] == UNAVAILABLE
    h.controller_states({'arm_controller': 'active'}, 0.)
    assert h.status(0.)[0] == UNKNOWN  # Never observed: unverified, not lost.
    h.joint_state(['a', 'b'], [0., 0.], 0.)
    assert h.status(.1)[0] == AVAILABLE
    assert h.status(.5)[0] == UNAVAILABLE
    h.joint_state(['a', 'b'], [float('nan'), 0.], .6)
    assert h.status(.6)[0] == UNAVAILABLE
    assert h.status(2.)[0] == UNKNOWN
