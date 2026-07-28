import pytest

from cleany_mujoco_sim.mecanum_kinematics import WheelSpeeds
from cleany_mujoco_sim.wheel_speed_controller import (
    PidGains,
    VelocityControllerConfig,
    VelocityPid,
    WheelSpeedController,
    WheelVoltages,
)


@pytest.fixture
def config() -> VelocityControllerConfig:
    return VelocityControllerConfig(
        gains=PidGains(kp=1.0, ki=5.0, kd=0.0),
        voltage_limit=10.8,
        no_load_speed=10.8,
    )


def test_pid_uses_feedforward_at_zero_error(config: VelocityControllerConfig):
    pid = VelocityPid(config)

    assert pid.update(target=5.4, measured=5.4, dt=0.002) == pytest.approx(5.4)


def test_pid_feedback_follows_error_sign(config: VelocityControllerConfig):
    positive_pid = VelocityPid(config)
    negative_pid = VelocityPid(config)

    positive = positive_pid.update(target=2.0, measured=0.0, dt=0.002)
    negative = negative_pid.update(target=-2.0, measured=0.0, dt=0.002)

    assert positive > 0.0
    assert negative < 0.0


def test_pid_clamps_voltage_and_prevents_integral_windup(
    config: VelocityControllerConfig,
):
    pid = VelocityPid(config)

    for _ in range(100):
        voltage = pid.update(target=100.0, measured=0.0, dt=0.002)

    assert voltage == pytest.approx(10.8)
    assert pid.integral_error == pytest.approx(0.0)


def test_pid_reset_clears_integral_state(config: VelocityControllerConfig):
    pid = VelocityPid(config)
    pid.update(target=2.0, measured=1.9, dt=0.002)

    assert pid.integral_error > 0.0

    pid.reset()

    assert pid.integral_error == 0.0


@pytest.mark.parametrize(
    ('target', 'measured', 'dt'),
    [
        (float('nan'), 0.0, 0.002),
        (0.0, float('inf'), 0.002),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, -0.1),
    ],
)
def test_pid_rejects_invalid_inputs(
    config: VelocityControllerConfig,
    target: float,
    measured: float,
    dt: float,
):
    with pytest.raises(ValueError):
        VelocityPid(config).update(target, measured, dt)


@pytest.mark.parametrize('value', [-0.1, float('inf'), float('nan')])
def test_pid_gains_require_non_negative_finite_values(value: float):
    with pytest.raises(ValueError):
        PidGains(kp=value, ki=0.0, kd=0.0)


@pytest.mark.parametrize('value', [0.0, -0.1, float('inf'), float('nan')])
def test_controller_limits_require_positive_finite_values(value: float):
    with pytest.raises(ValueError):
        VelocityControllerConfig(
            gains=PidGains(kp=1.0, ki=0.0, kd=0.0),
            voltage_limit=value,
            no_load_speed=10.8,
        )


def test_wheel_controllers_keep_each_wheel_independent(
    config: VelocityControllerConfig,
):
    controller = WheelSpeedController(config)

    voltages = controller.update(
        target=WheelSpeeds(1.0, 0.0, 0.0, 0.0),
        measured=WheelSpeeds(0.0, 0.0, 0.0, 0.0),
        dt=0.002,
    )

    assert voltages == WheelVoltages(
        front_left=pytest.approx(2.01),
        front_right=0.0,
        rear_left=0.0,
        rear_right=0.0,
    )
