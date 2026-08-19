from __future__ import annotations

from dataclasses import dataclass
from math import isfinite

from cleany_mujoco_sim.base_command import ChassisCommand


@dataclass(frozen=True)
class WheelSpeeds:
    front_left: float
    front_right: float
    rear_left: float
    rear_right: float


@dataclass(frozen=True)
class MecanumGeometry:
    wheel_radius: float
    wheelbase_length: float
    track_width: float

    def __post_init__(self) -> None:
        dimensions = (self.wheel_radius, self.wheelbase_length, self.track_width)
        if not all(isfinite(value) and value > 0.0 for value in dimensions):
            raise ValueError('mecanum dimensions must be positive and finite')


@dataclass(frozen=True)
class WheelSpeedLimit:
    max_abs: float

    def __post_init__(self) -> None:
        if not isfinite(self.max_abs) or self.max_abs <= 0.0:
            raise ValueError('wheel speed limit must be positive and finite')


def wheel_speeds_from_chassis(
    command: ChassisCommand,
    geometry: MecanumGeometry,
) -> WheelSpeeds:
    rotation_radius = (geometry.wheelbase_length + geometry.track_width) / 2.0
    rotational_velocity = rotation_radius * command.angular_z
    inverse_wheel_radius = 1.0 / geometry.wheel_radius

    return WheelSpeeds(
        front_left=(
            command.linear_x - command.linear_y - rotational_velocity
        ) * inverse_wheel_radius,
        front_right=(
            command.linear_x + command.linear_y + rotational_velocity
        ) * inverse_wheel_radius,
        rear_left=(
            command.linear_x + command.linear_y - rotational_velocity
        ) * inverse_wheel_radius,
        rear_right=(
            command.linear_x - command.linear_y + rotational_velocity
        ) * inverse_wheel_radius,
    )


def limited_wheel_speeds(
    speeds: WheelSpeeds,
    limit: WheelSpeedLimit,
) -> WheelSpeeds:
    values = (
        speeds.front_left,
        speeds.front_right,
        speeds.rear_left,
        speeds.rear_right,
    )
    if not all(isfinite(value) for value in values):
        raise ValueError('wheel speeds must be finite')

    peak_speed = max(abs(value) for value in values)
    if peak_speed <= limit.max_abs:
        return speeds

    scale = limit.max_abs / peak_speed
    return WheelSpeeds(
        front_left=speeds.front_left * scale,
        front_right=speeds.front_right * scale,
        rear_left=speeds.rear_left * scale,
        rear_right=speeds.rear_right * scale,
    )


def stopped_wheel_speeds() -> WheelSpeeds:
    return WheelSpeeds(
        front_left=0.0,
        front_right=0.0,
        rear_left=0.0,
        rear_right=0.0,
    )
