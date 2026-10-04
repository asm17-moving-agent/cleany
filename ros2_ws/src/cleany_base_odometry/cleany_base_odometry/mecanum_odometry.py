from __future__ import annotations

from dataclasses import dataclass
from collections import deque
from math import atan2, cos, isfinite, sin


@dataclass(frozen=True)
class MecanumGeometry:
    """Physical dimensions used by four-wheel Mecanum kinematics."""

    wheel_radius_m: float
    wheelbase_m: float
    wheel_separation_m: float

    def __post_init__(self) -> None:
        values = (
            self.wheel_radius_m,
            self.wheelbase_m,
            self.wheel_separation_m,
        )
        if not all(isfinite(value) and value > 0.0 for value in values):
            raise ValueError('Mecanum geometry values must be positive and finite')

    @property
    def center_to_wheel_sum_m(self) -> float:
        """Return the longitudinal plus lateral center-to-wheel offsets."""
        return (self.wheelbase_m + self.wheel_separation_m) / 2.0


@dataclass(frozen=True)
class WheelPositions:
    """Cumulative wheel angles in radians."""

    front_left: float
    front_right: float
    rear_left: float
    rear_right: float

    def __post_init__(self) -> None:
        if not all(isfinite(value) for value in self.as_tuple()):
            raise ValueError('Wheel positions must be finite')

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (
            self.front_left,
            self.front_right,
            self.rear_left,
            self.rear_right,
        )


@dataclass(frozen=True)
class OdometryEstimate:
    """Integrated planar pose and body-frame velocity."""

    x_m: float
    y_m: float
    yaw_rad: float
    linear_x_mps: float
    linear_y_mps: float
    angular_z_rps: float


class MecanumOdometry:
    """Integrate cumulative wheel positions into a planar odometry estimate."""

    def __init__(self, geometry: MecanumGeometry, *, velocity_window_s: float = 0.0) -> None:
        if not isfinite(velocity_window_s) or velocity_window_s < 0.0:
            raise ValueError('Velocity window must be finite and non-negative')
        self._geometry = geometry
        self._velocity_window_s = velocity_window_s
        self._velocity_history: deque[tuple[float, WheelPositions]] = deque()
        self._previous_positions: WheelPositions | None = None
        self._previous_stamp_s: float | None = None
        self._x_m = 0.0
        self._y_m = 0.0
        self._yaw_rad = 0.0

    def update(
        self,
        positions: WheelPositions,
        stamp_s: float,
    ) -> OdometryEstimate | None:
        """Consume one wheel sample, returning no estimate for the baseline."""
        if not isfinite(stamp_s):
            raise ValueError('Sample timestamp must be finite')

        previous_positions = self._previous_positions
        previous_stamp_s = self._previous_stamp_s
        self._previous_positions = positions
        self._previous_stamp_s = stamp_s
        if previous_positions is None or previous_stamp_s is None:
            self._velocity_history.clear()
            self._velocity_history.append((stamp_s, positions))
            return None

        dt_s = stamp_s - previous_stamp_s
        if dt_s <= 0.0:
            self._velocity_history.clear()
            self._velocity_history.append((stamp_s, positions))
            return None

        current = positions.as_tuple()
        previous = previous_positions.as_tuple()
        front_left, front_right, rear_left, rear_right = (
            current[index] - previous[index] for index in range(4)
        )

        radius_scale = self._geometry.wheel_radius_m / 4.0
        forward_m = radius_scale * (
            front_left + front_right + rear_left + rear_right
        )
        lateral_m = radius_scale * (
            -front_left + front_right + rear_left - rear_right
        )
        yaw_delta_rad = radius_scale * (
            -front_left + front_right - rear_left + rear_right
        ) / self._geometry.center_to_wheel_sum_m

        midpoint_yaw = self._yaw_rad + yaw_delta_rad / 2.0
        self._x_m += (
            cos(midpoint_yaw) * forward_m
            - sin(midpoint_yaw) * lateral_m
        )
        self._y_m += (
            sin(midpoint_yaw) * forward_m
            + cos(midpoint_yaw) * lateral_m
        )
        self._yaw_rad = atan2(
            sin(self._yaw_rad + yaw_delta_rad),
            cos(self._yaw_rad + yaw_delta_rad),
        )

        velocity_delta = (forward_m, lateral_m, yaw_delta_rad)
        velocity_dt = dt_s
        if self._velocity_window_s:
            # Pose still integrates every received encoder sample. Only twist
            # uses cumulative counts over a longer observation interval; tick
            # quantization must not become alternating instantaneous speeds.
            if dt_s > self._velocity_window_s:
                self._velocity_history.clear()
            self._velocity_history.append((stamp_s, positions))
            cutoff = stamp_s - self._velocity_window_s
            while len(self._velocity_history) > 1 and self._velocity_history[1][0] <= cutoff:
                self._velocity_history.popleft()
            old_stamp, old_positions = self._velocity_history[0]
            velocity_dt = stamp_s - old_stamp
            if velocity_dt + 1e-9 < self._velocity_window_s:
                # After startup/clock reset/data gap, do not label an
                # unobservable velocity as a fresh measured zero.
                return None
            fl, fr, rl, rr = (
                current[index] - old_positions.as_tuple()[index]
                for index in range(4)
            )
            velocity_delta = (
                radius_scale * (fl + fr + rl + rr),
                radius_scale * (-fl + fr + rl - rr),
                radius_scale * (-fl + fr - rl + rr) / self._geometry.center_to_wheel_sum_m,
            )

        return OdometryEstimate(
            x_m=self._x_m,
            y_m=self._y_m,
            yaw_rad=self._yaw_rad,
            linear_x_mps=velocity_delta[0] / velocity_dt,
            linear_y_mps=velocity_delta[1] / velocity_dt,
            angular_z_rps=velocity_delta[2] / velocity_dt,
        )
