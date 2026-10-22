"""Height feedback under static load, with no ROS or motor connection."""
import math
import pytest

from cleany_skill_executor.core.height_correction import (
    HeightCorrection, HeightCorrectionError, HeightCorrectionPolicy,
    validate_height_correctable_endpoint,
)
from cleany_skill_executor.core.cartesian import CartesianPose


def policy(**kwargs):
    return HeightCorrectionPolicy(enabled=True, **kwargs)


def test_logged_lift_error_converges_without_moving_the_physical_goal():
    goal = .5743888152472258
    actual = .5557920195791695
    sag = goal - actual
    controller = HeightCorrection(goal, goal, policy())
    commands = []
    while (command := controller.next_command(actual)) is not None:
        commands.append(command)
        actual = command - sag
    assert len(commands) == 1
    assert actual == pytest.approx(goal + .020 - sag)
    assert controller.goal_z == goal
    assert abs(goal - actual) <= .005
    assert commands[0] == pytest.approx(goal + .020)


def test_logged_thirty_two_mm_shortfall_converges():
    goal, sag = .5408224849074599, .0319983857472429
    correction = HeightCorrection(goal, goal, policy())
    actual, commands = goal-sag, []
    while (command := correction.next_command(actual)) is not None:
        commands.append(command)
        actual = command-sag
    assert len(commands) == 1
    assert correction.goal_z == goal
    assert goal-actual <= .010
    assert commands[-1]-goal <= .040
    assert commands[0] == pytest.approx(goal + .035)


@pytest.mark.parametrize('shortfall_mm,step_mm', [
    (.1, 5.), (1., 5.), (5., 5.), (8., 10.), (10., 10.), (11., 15.), (15., 15.), (15.0001, 20.),
    (21., 25.), (31.9983857472429, 35.), (35.1138972905688, 40.),
    (43.382882998641126, 45.), (75., 75.), (101., 105.),
])
def test_entire_shortfall_rounds_up_to_five_mm_grid(shortfall_mm, step_mm):
    controller = HeightCorrection(.5, .5, policy())
    assert controller.next_command(.5-shortfall_mm/1000) == pytest.approx(.5+step_mm/1000)
    assert controller.attempts == 1


@pytest.mark.parametrize('xyz,angle,accepted', [
    ((.316879,.056145,.508824), 5.589, True),
    ((.351,.058715,.508824), 0., False),
    ((.319833,.058715,.508824), 21., False),
    ((.319833,.058715,.591823), 0., False),
    ((.319833,.058715,.571823), 0., False),
    ((.319833,.058715,.489822), 0., True),
])
def test_height_shortfall_keeps_xy_rotation_and_overshoot_guards(xyz, angle, accepted):
    half = math.radians(angle)/2
    actual = CartesianPose(xyz, (0.,0.,math.sin(half),math.cos(half)))
    target = CartesianPose((.319833,.058715,.540822), (0.,0.,0.,1.))
    if accepted:
        validate_height_correctable_endpoint(actual, target, .030, math.radians(20.))
    else:
        with pytest.raises(ValueError):
            validate_height_correctable_endpoint(actual, target, .030, math.radians(20.))


def test_height_correction_accumulates_from_sent_command_not_measured_goal():
    goal = .5318423315105774
    sent = .5419910580592112
    actual = .5243888133737824
    controller = HeightCorrection(goal, sent, policy())
    next_command = controller.next_command(actual)
    assert next_command == pytest.approx(sent + .010)
    assert next_command > sent
    assert controller.next_command(actual + next_command-sent) is None


@pytest.mark.parametrize('actual', [.5, .501, .52])
def test_reached_goal_requires_no_additional_command(actual):
    assert HeightCorrection(.5, .52, policy()).next_command(actual) is None


@pytest.mark.parametrize('actual', [.48, .479, .4805, .49, .5, .52])
def test_remaining_error_never_causes_a_second_command_or_height_failure(actual):
    controller = HeightCorrection(.5, .5, policy())
    controller.next_command(.48)
    assert controller.next_command(actual) is None
    assert controller.attempts == 1
    assert controller.command_z == pytest.approx(.52)


def test_logged_forty_three_mm_error_can_correct_with_a_legacy_config():
    settings = HeightCorrectionPolicy.from_config({
        'lift_height_correction_enabled': True, 'lift_height_correction_max_total_m': .040,
        'lift_height_correction_max_attempts': 3, 'lift_height_correction_min_progress_m': .001,
        'lift_height_tolerance_m': .010})
    goal, sent, actual = .5877117403295785, .587711741975668, .5443288573309374
    controller = HeightCorrection(goal, sent, settings)
    command = controller.next_command(actual)
    assert command == pytest.approx(sent+.045)
    assert controller.next_command(actual) is None
    assert controller.attempts == 1


@pytest.mark.parametrize('actual', [float('nan'), float('inf'), -1e308])
def test_bad_feedback_never_triggers_another_motion(actual):
    controller = HeightCorrection(.5, .5, policy())
    with pytest.raises(HeightCorrectionError, match='nonfinite'):
        controller.next_command(actual)
    assert controller.attempts == 0


@pytest.mark.parametrize('field,value', [
    ('enabled', 'true'), ('step_m', True), ('step_m', 0), ('step_m', float('nan')),
    ('step_m', float('inf')), ('step_m', -.005),
])
def test_bad_configuration_is_rejected(field, value):
    with pytest.raises(ValueError, match='lift_height_correction'):
        HeightCorrectionPolicy.from_config({'lift_height_correction_'+field: value})


def test_absent_configuration_preserves_disabled_behavior():
    settings = HeightCorrectionPolicy.from_config({})
    assert settings == HeightCorrectionPolicy()
    with pytest.raises(HeightCorrectionError, match='disabled'):
        HeightCorrection(.5, .5, settings).next_command(.48)
