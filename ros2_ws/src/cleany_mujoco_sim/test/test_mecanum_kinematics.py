import pytest

from cleany_mujoco_sim.base_command import ChassisCommand
from cleany_mujoco_sim.mecanum_kinematics import (
    MecanumGeometry,
    WheelSpeedLimit,
    WheelSpeeds,
    limited_wheel_speeds,
    stopped_wheel_speeds,
    wheel_speeds_from_chassis,
)


@pytest.fixture
def geometry() -> MecanumGeometry:
    return MecanumGeometry(
        wheel_radius=0.1,
        wheelbase_length=0.4,
        track_width=0.2,
    )


def test_forward_command_sets_all_wheels_to_the_same_speed(
    geometry: MecanumGeometry,
):
    speeds = wheel_speeds_from_chassis(
        ChassisCommand(linear_x=0.2, linear_y=0.0, angular_z=0.0),
        geometry,
    )

    assert speeds == WheelSpeeds(2.0, 2.0, 2.0, 2.0)


def test_left_command_uses_diagonal_wheel_pairs(
    geometry: MecanumGeometry,
):
    speeds = wheel_speeds_from_chassis(
        ChassisCommand(linear_x=0.0, linear_y=0.2, angular_z=0.0),
        geometry,
    )

    assert speeds == WheelSpeeds(-2.0, 2.0, 2.0, -2.0)


def test_counterclockwise_command_reverses_left_wheels(
    geometry: MecanumGeometry,
):
    speeds = wheel_speeds_from_chassis(
        ChassisCommand(linear_x=0.0, linear_y=0.0, angular_z=1.0),
        geometry,
    )

    assert (
        speeds.front_left,
        speeds.front_right,
        speeds.rear_left,
        speeds.rear_right,
    ) == pytest.approx((-3.0, 3.0, -3.0, 3.0))


def test_combined_command_uses_every_chassis_axis(
    geometry: MecanumGeometry,
):
    speeds = wheel_speeds_from_chassis(
        ChassisCommand(linear_x=0.2, linear_y=-0.1, angular_z=-0.5),
        geometry,
    )

    assert (
        speeds.front_left,
        speeds.front_right,
        speeds.rear_left,
        speeds.rear_right,
    ) == pytest.approx((4.5, -0.5, 2.5, 1.5))


def test_wheel_speed_limit_preserves_ratios():
    speeds = WheelSpeeds(4.0, -2.0, 1.0, -0.5)

    limited = limited_wheel_speeds(speeds, WheelSpeedLimit(max_abs=2.0))

    assert limited == WheelSpeeds(2.0, -1.0, 0.5, -0.25)


def test_wheel_speed_limit_preserves_values_within_limit():
    speeds = WheelSpeeds(1.0, -2.0, 0.5, -0.25)

    assert limited_wheel_speeds(
        speeds,
        WheelSpeedLimit(max_abs=2.0),
    ) is speeds


@pytest.mark.parametrize('value', [0.0, -0.1, float('inf'), float('nan')])
def test_mecanum_geometry_requires_positive_finite_dimensions(value: float):
    with pytest.raises(ValueError):
        MecanumGeometry(
            wheel_radius=value,
            wheelbase_length=0.4,
            track_width=0.2,
        )


@pytest.mark.parametrize('value', [0.0, -0.1, float('inf'), float('nan')])
def test_wheel_speed_limit_requires_positive_finite_value(value: float):
    with pytest.raises(ValueError):
        WheelSpeedLimit(max_abs=value)


@pytest.mark.parametrize('value', [float('inf'), float('-inf'), float('nan')])
def test_wheel_speed_limit_rejects_non_finite_wheel_speed(value: float):
    with pytest.raises(ValueError):
        limited_wheel_speeds(
            WheelSpeeds(value, 0.0, 0.0, 0.0),
            WheelSpeedLimit(max_abs=2.0),
        )


def test_stopped_wheel_speeds_zeros_every_wheel():
    assert stopped_wheel_speeds() == WheelSpeeds(0.0, 0.0, 0.0, 0.0)
