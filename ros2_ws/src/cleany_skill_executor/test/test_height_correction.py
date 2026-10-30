import pytest
from cleany_skill_executor.core.height_correction import HeightCorrection, HeightCorrectionError, HeightCorrectionPolicy

def policy(**kwargs):
    return HeightCorrectionPolicy(enabled=True, **kwargs)


@pytest.mark.parametrize('shortfall_mm,step_mm', [
    (.1, 5.), (1., 5.), (5., 5.), (8., 10.), (10., 10.), (11., 15.), (15., 15.), (15.0001, 20.),
    (18.5967956680563, 20.), (21., 25.), (31.9983857472429, 35.), (35.1138972905688, 40.),
    (43.382882998641126, 45.), (75., 75.), (101., 105.),
])
def test_entire_shortfall_rounds_up_to_five_mm_grid(shortfall_mm, step_mm):
    controller = HeightCorrection(.5, .5, policy())
    assert controller.next_command(.5-shortfall_mm/1000) == pytest.approx(.5+step_mm/1000)
    assert controller.attempts == 1
    assert controller.goal_z == .5
    assert controller.next_command(.5-shortfall_mm/1000) is None


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


@pytest.mark.parametrize('shortfall,expected_step', [
    (.009999, None), (.010, None), (.010001, .015), (.011, .015),
])
def test_tolerance_boundary_skips_or_corrects_the_complete_shortfall(shortfall, expected_step):
    controller = HeightCorrection(.5, .52, policy(tolerance_m=.010))
    command = controller.next_command(.5-shortfall)
    if expected_step is None:
        assert command is None
        assert controller.attempts == 0
    else:
        assert command == pytest.approx(.52+expected_step)
        assert controller.attempts == 1


@pytest.mark.parametrize('value', [True, '0.01', -.001, float('nan'), float('inf')])
def test_invalid_height_tolerance_is_rejected(value):
    with pytest.raises(ValueError, match='lift_height_tolerance_m'):
        HeightCorrectionPolicy.from_config({'lift_height_tolerance_m': value})


@pytest.mark.parametrize('actual', [.48, .479, .4805, .49, .5, .52])
def test_remaining_error_never_causes_a_second_command_or_height_failure(actual):
    controller = HeightCorrection(.5, .5, policy())
    controller.next_command(.48)
    assert controller.next_command(actual) is None
    assert controller.attempts == 1
    assert controller.command_z == pytest.approx(.52)


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
