from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class ChassisCommand:
    linear_x: float
    linear_y: float
    angular_z: float


@dataclass(frozen=True)
class CommandLimits:
    max_linear_x: float
    max_linear_y: float
    max_angular_z: float

    def __post_init__(self) -> None:
        for value in (self.max_linear_x, self.max_linear_y, self.max_angular_z):
            if not isfinite(value) or value <= 0.0:
                raise ValueError('command limits must be positive and finite')


def are_finite_values(values: Iterable[float]) -> bool:
    return all(isfinite(value) for value in values)


def bounded_command(command: ChassisCommand, limits: CommandLimits) -> ChassisCommand:
    values = (command.linear_x, command.linear_y, command.angular_z)
    if not are_finite_values(values):
        raise ValueError('command values must be finite')

    return ChassisCommand(
        linear_x=max(-limits.max_linear_x, min(limits.max_linear_x, command.linear_x)),
        linear_y=max(-limits.max_linear_y, min(limits.max_linear_y, command.linear_y)),
        angular_z=max(-limits.max_angular_z, min(limits.max_angular_z, command.angular_z)),
    )
