from __future__ import annotations

from dataclasses import dataclass
from math import isfinite

from cleany_mujoco_sim.mecanum_kinematics import WheelSpeeds


@dataclass(frozen=True)
class PidGains:
    kp: float
    ki: float
    kd: float

    def __post_init__(self) -> None:
        values = (self.kp, self.ki, self.kd)
        if not all(isfinite(value) and value >= 0.0 for value in values):
            raise ValueError('PID gains must be non-negative and finite')


@dataclass(frozen=True)
class VelocityControllerConfig:
    gains: PidGains
    voltage_limit: float
    no_load_speed: float

    def __post_init__(self) -> None:
        values = (self.voltage_limit, self.no_load_speed)
        if not all(isfinite(value) and value > 0.0 for value in values):
            raise ValueError('controller limits must be positive and finite')


@dataclass(frozen=True)
class WheelVoltages:
    front_left: float
    front_right: float
    rear_left: float
    rear_right: float


class VelocityPid:
    def __init__(self, config: VelocityControllerConfig) -> None:
        self._config = config
        self._integral_error = 0.0
        self._previous_measurement: float | None = None

    @property
    def integral_error(self) -> float:
        return self._integral_error

    def update(self, target: float, measured: float, dt: float) -> float:
        if not all(isfinite(value) for value in (target, measured, dt)) or dt <= 0.0:
            raise ValueError('target, measurement, and positive dt must be finite')

        error = target - measured
        derivative = 0.0
        if self._previous_measurement is not None:
            derivative = -(measured - self._previous_measurement) / dt
        self._previous_measurement = measured

        gains = self._config.gains
        feedforward = (
            target / self._config.no_load_speed
        ) * self._config.voltage_limit
        proportional = gains.kp * error
        derivative_output = gains.kd * derivative

        candidate_integral = self._integral_error
        if gains.ki > 0.0:
            candidate_integral += error * dt
            integral_limit = self._config.voltage_limit / gains.ki
            candidate_integral = max(
                -integral_limit,
                min(integral_limit, candidate_integral),
            )

        candidate_voltage = (
            feedforward
            + proportional
            + gains.ki * candidate_integral
            + derivative_output
        )
        saturating_high = (
            candidate_voltage > self._config.voltage_limit and error > 0.0
        )
        saturating_low = (
            candidate_voltage < -self._config.voltage_limit and error < 0.0
        )
        if not saturating_high and not saturating_low:
            self._integral_error = candidate_integral

        voltage = (
            feedforward
            + proportional
            + gains.ki * self._integral_error
            + derivative_output
        )
        return max(
            -self._config.voltage_limit,
            min(self._config.voltage_limit, voltage),
        )

    def reset(self) -> None:
        self._integral_error = 0.0
        self._previous_measurement = None


class WheelSpeedController:
    def __init__(self, config: VelocityControllerConfig) -> None:
        self._front_left = VelocityPid(config)
        self._front_right = VelocityPid(config)
        self._rear_left = VelocityPid(config)
        self._rear_right = VelocityPid(config)

    def update(
        self,
        target: WheelSpeeds,
        measured: WheelSpeeds,
        dt: float,
    ) -> WheelVoltages:
        return WheelVoltages(
            front_left=self._front_left.update(
                target.front_left, measured.front_left, dt
            ),
            front_right=self._front_right.update(
                target.front_right, measured.front_right, dt
            ),
            rear_left=self._rear_left.update(
                target.rear_left, measured.rear_left, dt
            ),
            rear_right=self._rear_right.update(
                target.rear_right, measured.rear_right, dt
            ),
        )

    def reset(self) -> None:
        self._front_left.reset()
        self._front_right.reset()
        self._rear_left.reset()
        self._rear_right.reset()
