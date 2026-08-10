import pytest

from cleany_mujoco_sim.base_command import (
    ChassisCommand,
    CommandLimits,
    are_finite_values,
    bounded_command,
    stopped_command,
)


def test_bounded_command_preserves_values_within_limits():
    limits = CommandLimits(max_linear_x=0.3, max_linear_y=0.2, max_angular_z=0.8)
    command = ChassisCommand(linear_x=0.1, linear_y=-0.2, angular_z=0.4)

    assert bounded_command(command, limits) == command


def test_bounded_command_clamps_each_axis_in_both_directions():
    limits = CommandLimits(max_linear_x=0.3, max_linear_y=0.2, max_angular_z=0.8)

    assert bounded_command(
        ChassisCommand(linear_x=0.5, linear_y=-0.4, angular_z=1.2),
        limits,
    ) == ChassisCommand(linear_x=0.3, linear_y=-0.2, angular_z=0.8)
    assert bounded_command(
        ChassisCommand(linear_x=-0.5, linear_y=0.4, angular_z=-1.2),
        limits,
    ) == ChassisCommand(linear_x=-0.3, linear_y=0.2, angular_z=-0.8)


@pytest.mark.parametrize(
    'value',
    [0.0, -0.1, float('inf'), float('-inf'), float('nan')],
)
def test_command_limits_require_positive_finite_values(value: float):
    with pytest.raises(ValueError):
        CommandLimits(max_linear_x=value, max_linear_y=0.2, max_angular_z=0.8)


@pytest.mark.parametrize('value', [float('inf'), float('-inf'), float('nan')])
def test_bounded_command_rejects_non_finite_values(value: float):
    limits = CommandLimits(max_linear_x=0.3, max_linear_y=0.2, max_angular_z=0.8)

    with pytest.raises(ValueError):
        bounded_command(ChassisCommand(value, 0.0, 0.0), limits)


def test_are_finite_values_checks_every_value():
    assert are_finite_values((0.1, 0.0, -0.2))
    assert not are_finite_values((0.1, 0.0, float('nan')))


def test_stopped_command_zeros_every_supported_axis():
    assert stopped_command() == ChassisCommand(0.0, 0.0, 0.0)
