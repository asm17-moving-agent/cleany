import math
import pytest
from cleany_base_driver.core import (
    EncoderAdapter, Geometry, Limits, finite_twist_axes, newer,
    wheel_speeds,
)


def test_validation_and_nonfinite_command():
    with pytest.raises(ValueError):
        Geometry(math.inf, 1, 1)
    g, l = Geometry(.1, .4, .4), Limits(1, 1, 1, 10, .2)
    assert wheel_speeds(math.nan, 0, 0, g, l) == (0.,) * 4
    with pytest.raises(ValueError):
        Limits(1, 1, 1, 10.1, .2)


def test_ik_clamp_and_proportional_wheel_limit():
    g, l = Geometry(.1, .4, .4), Limits(1, 1, 1, 2, .2)
    result = wheel_speeds(100, 0, 0, g, l)
    assert max(map(abs, result)) == pytest.approx(2)
    assert len(set(result)) == 1


def test_mecanum_lateral_and_yaw_signs():
    g, l = Geometry(.1, .4, .4), Limits(1, 1, 1, 10, .2)
    assert wheel_speeds(0, 1, 0, g, l) == pytest.approx((-10, 10, 10, -10))
    assert wheel_speeds(0, 0, 1, g, l) == pytest.approx((-4, 4, -4, 4))


def test_modular_sequences_and_encoder_rollover_reboot():
    assert newer(0, 0xffffffff)
    assert not newer(0x80000000, 0)
    a = EncoderAdapter()
    assert a.update(1, (0x7fffffff,)*4) == (0.,)*4
    result = a.update(1, (-0x80000000,)*4)
    assert result[0] == pytest.approx(6.283185307179586/3172)
    before = result
    assert a.update(2, (100,)*4) == before
    assert a.update(2, (101,)*4)[0] == pytest.approx(before[0] + 6.283185307179586/3172)
    after = a.update(2, (102,)*4)
    assert after[0] == pytest.approx(before[0] + 2*6.283185307179586/3172)


@pytest.mark.parametrize('axis', range(6))
@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_all_twist_axes_reject_nonfinite(axis, bad):
    g, l = Geometry(.1, .4, .4), Limits(1, 1, 1, 10, .2)
    values = [0.] * 6
    values[axis] = bad
    assert not finite_twist_axes(*values)
    assert wheel_speeds(values[0], values[1], values[5], g, l) == (0.,) * 4


def test_encoder_rollover_negative_direction():
    a = EncoderAdapter()
    a.update(1, (-0x80000000,)*4)
    result = a.update(1, (0x7fffffff,)*4)
    assert result[0] == pytest.approx(-6.283185307179586/3172)


def test_encoder_feedback_gap_rebases_without_synthetic_motion():
    a = EncoderAdapter()
    a.update(8, (10,)*4)
    position = a.update(8, (20,)*4)
    a.rebase()
    assert a.update(8, (400,)*4) == position
    assert a.update(8, (401,)*4)[0] == pytest.approx(
        position[0] + 6.283185307179586/3172)
