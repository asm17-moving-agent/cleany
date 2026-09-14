from types import SimpleNamespace

import pytest

from cleany_skill_executor.core.gripper import (
    aperture_centering_offset,
    is_gripper_contact_stall,
    opening_to_gripper_position,
)
import cleany_skill_executor.nearest_pregrasp_coordinator as module


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


@pytest.mark.parametrize('opening,expected', [(.04, .3), (.08, .5), (.20, 1.2)])
def test_width_aware_command_is_scaled_and_clamped(opening, expected):
    assert opening_to_gripper_position(
        required_opening_m=opening, opening_reduction_m=.01,
        reference_aperture_m=.05, reference_position_rad=.3, aperture_m_per_rad=.1,
        minimum_position_rad=.3, maximum_position_rad=1.2) == pytest.approx(expected)


@pytest.mark.parametrize('actual,velocity,expected', [
    (.78, .01, True), (.78, .20, False), (1.15, 0., False), (.30, 0., False),
])
def test_initial_contact_requires_motion_residual_and_low_speed(actual, velocity, expected):
    assert is_gripper_contact_stall(
        start=1.2, actual=actual, command=.3, velocity=velocity,
        minimum_motion=.1, minimum_residual=.05, maximum_velocity=.05) is expected


def test_fixed_jaw_clearance_moves_edge_away_from_inner_surface():
    correction = aperture_centering_offset(.088, .008, .008, .003)
    edge_in_tool = (.088-.008)/2-correction
    assert .008-edge_in_tool == pytest.approx(.003)
    with pytest.raises(ValueError):
        aperture_centering_offset(.088, .008, .008, .009)


def run(monkeypatch, readings):
    now = [0.0]
    iterator = iter(readings)
    values = {'gripper_contact_feedback_timeout_sec': .6,
              'gripper_contact_stable_duration_sec': .1,
              'gripper_contact_max_velocity_rad_s': .05}
    node = SimpleNamespace(
        _gripper_feedback_stamps={'left_gripper_joint': 1},
        _joint_positions={'left_gripper_joint': -.02},
        _joint_velocities={'left_gripper_joint': .257},
        get_parameter=lambda key: SimpleNamespace(value=values[key]),
        get_logger=lambda: SimpleNamespace(info=lambda _: None))
    node._gripper_contact_stalled = lambda *_: (
        abs(node._joint_velocities['left_gripper_joint']) <= .05
        and abs(node._joint_positions['left_gripper_joint'] + .3) > .05)
    def spin(*_, **__):
        now[0] += .05
        reading = next(iterator, None)
        if reading is not None:
            stamp, position, velocity = reading
            node._gripper_feedback_stamps['left_gripper_joint'] = stamp
            node._joint_positions['left_gripper_joint'] = position
            node._joint_velocities['left_gripper_joint'] = velocity
    monkeypatch.setattr(module.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    return module.NearestPregraspCoordinator._wait_for_gripper_contact(node, 'left', 1.2, -.3)


def test_transient_velocity_settles_without_reclosing(monkeypatch):
    assert run(monkeypatch, [(10, -.02, .257), (100_000_000, -.02, 0.),
                            (150_000_000, -.02, .01), (210_000_000, -.02, 0.)])


@pytest.mark.parametrize('readings', [[], [(10, -.02, 0.)] * 12,
    [(i*50_000_000, -.02, .257) for i in range(1, 13)],
    [(100_000_000, -.02, 0.), (150_000_000, -.02, .2), (210_000_000, -.02, 0.)]])
def test_missing_replayed_moving_or_interrupted_contact_fails(monkeypatch, readings):
    with pytest.raises(RuntimeError, match='did not stabilize'):
        run(monkeypatch, readings)


def test_empty_full_close_is_not_contact(monkeypatch):
    assert not run(monkeypatch, [(100_000_000, -.3, 0.)])
