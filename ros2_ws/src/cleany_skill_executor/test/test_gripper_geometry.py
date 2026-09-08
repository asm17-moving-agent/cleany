import pytest
from cleany_skill_executor.core.gripper import aperture_centering_offset, opening_to_gripper_position
from cleany_skill_executor.core.gripper import is_gripper_contact_stall


def test_carry_can_tighten_but_initial_grasp_requires_stall():
    args = dict(start=1.4, actual=-.05, command=-.3, velocity=-.26,
                minimum_motion=.1, minimum_residual=.05, maximum_velocity=.2)
    assert not is_gripper_contact_stall(**args)
    assert is_gripper_contact_stall(**args, allow_closing_motion=True)
    assert not is_gripper_contact_stall(**dict(args, actual=-.29), allow_closing_motion=True)
    assert not is_gripper_contact_stall(**dict(args, velocity=.26), allow_closing_motion=True)
    assert not is_gripper_contact_stall(**dict(args, velocity=float('nan')), allow_closing_motion=True)


@pytest.mark.parametrize('opening', [.024,.05,.088])
def test_per_width_correction_aligns_object_edge_with_fixed_jaw(opening):
    correction = aperture_centering_offset(opening,.008,.008)
    assert -correction+(opening-.008)/2 == pytest.approx(.008)
    if opening == .024:
        assert correction == pytest.approx(0)


def test_small_object_command_is_more_closed_and_inside_joint_limits():
    def command(opening):
        return opening_to_gripper_position(required_opening_m=opening,opening_reduction_m=.018,
            reference_aperture_m=.05,reference_position_rad=.3,aperture_m_per_rad=.065,
            minimum_position_rad=-.3,maximum_position_rad=1.4)
    assert -.3 <= command(.024) < command(.05) < command(.088) <= 1.4


def test_fixed_jaw_clearance_moves_edge_away_from_inner_surface():
    correction = aperture_centering_offset(.088, .008, .008, .003)
    edge_in_tool = (.088-.008)/2-correction
    assert .008-edge_in_tool == pytest.approx(.003)
    with pytest.raises(ValueError):
        aperture_centering_offset(.088, .008, .008, .009)
