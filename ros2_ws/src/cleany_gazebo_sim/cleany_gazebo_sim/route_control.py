from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float


@dataclass(frozen=True)
class RouteCommand:
    linear_x: float
    angular_z: float
    waypoint_index: int
    completed: bool


@dataclass(frozen=True)
class RouteLimits:
    max_linear_speed: float
    max_angular_speed: float
    heading_gain: float
    position_tolerance: float
    turn_in_place_threshold: float

    def __post_init__(self) -> None:
        values = (
            self.max_linear_speed,
            self.max_angular_speed,
            self.heading_gain,
            self.position_tolerance,
            self.turn_in_place_threshold,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('route limits must be positive and finite')


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def waypoints_from_flat(values: Sequence[float]) -> tuple[Waypoint, ...]:
    if len(values) < 4 or len(values) % 2 != 0:
        raise ValueError('waypoints_xy must contain at least two x/y pairs')
    converted = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in converted):
        raise ValueError('waypoints_xy must contain only finite values')
    return tuple(
        Waypoint(converted[index], converted[index + 1])
        for index in range(0, len(converted), 2)
    )


class RouteTracker:
    def __init__(
        self, waypoints: Sequence[Waypoint], limits: RouteLimits
    ) -> None:
        if len(waypoints) < 2:
            raise ValueError('route requires at least two waypoints')
        self._waypoints = tuple(waypoints)
        self._limits = limits
        self._index = 0

    @property
    def waypoint_index(self) -> int:
        return self._index

    @property
    def waypoint_count(self) -> int:
        return len(self._waypoints)

    def command(self, pose: Pose2D) -> RouteCommand:
        if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
            raise ValueError('pose must contain only finite values')

        while self._index < len(self._waypoints):
            target = self._waypoints[self._index]
            distance = math.hypot(target.x - pose.x, target.y - pose.y)
            if distance > self._limits.position_tolerance:
                break
            self._index += 1

        if self._index >= len(self._waypoints):
            return RouteCommand(0.0, 0.0, self._index, True)

        target = self._waypoints[self._index]
        dx = target.x - pose.x
        dy = target.y - pose.y
        distance = math.hypot(dx, dy)
        heading_error = normalize_angle(math.atan2(dy, dx) - pose.yaw)
        angular = max(
            -self._limits.max_angular_speed,
            min(
                self._limits.max_angular_speed,
                self._limits.heading_gain * heading_error,
            ),
        )
        if abs(heading_error) >= self._limits.turn_in_place_threshold:
            linear = 0.0
        else:
            linear = min(self._limits.max_linear_speed, distance)
            linear *= max(0.25, math.cos(heading_error))
        return RouteCommand(linear, angular, self._index, False)
