"""Fixed factory gyro calibration and synchronized IMU input validation."""
from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from math import isclose, isfinite


class ImuValidationError(ValueError):
    """An IMU sample violates the sensor or acquisition-time contract."""


def _finite_values(values: Sequence[float], count: int, name: str) -> tuple[float, ...]:
    if (len(values) != count
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not isfinite(v) for v in values)):
        raise ValueError(f'{name} must contain {count} finite numbers')
    return tuple(float(v) for v in values)


@dataclass(frozen=True)
class FactoryCalibration:
    """Immutable sensor-axis gyro bias (rad/s) and covariance ((rad/s)^2)."""

    bias: tuple[float, ...]
    covariance: tuple[float, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, 'bias', _finite_values(self.bias, 3, 'gyro bias'))
        covariance = _finite_values(self.covariance, 9, 'gyro covariance')
        a, b, c, d, e, f, g, h, i = covariance
        if (not all(isclose(x, y, rel_tol=1e-9, abs_tol=1e-15)
                    for x, y in ((b, d), (c, g), (f, h)))
                or a <= 0 or a * e - b * d <= 0
                or a * (e * i - f * h) - b * (d * i - f * g)
                + c * (d * h - e * g) <= 0):
            raise ValueError('gyro covariance must be symmetric positive definite')
        object.__setattr__(self, 'covariance', covariance)

    def correct(self, angular_velocity: Sequence[float]) -> tuple[float, ...]:
        gyro = _finite_values(angular_velocity, 3, 'angular velocity')
        return tuple(value - bias for value, bias in zip(gyro, self.bias))


class ImuAdapter:
    """Check Agent-synchronized ROS acquisition times without restamping samples."""

    def __init__(self, calibration: FactoryCalibration, frame_id: str = 'imu_link',
                 max_transport_delay_s: float = 0.1,
                 max_clock_jump_s: float = 0.05) -> None:
        if (not isinstance(frame_id, str) or not frame_id or frame_id.startswith('/')
                or any(char.isspace() for char in frame_id)):
            raise ValueError('frame_id must be a nonempty relative sensor-frame name')
        if any(not isfinite(value) or not 0 < value <= 60
               for value in (max_transport_delay_s, max_clock_jump_s)):
            raise ValueError('timing limits must be positive and finite')
        self.calibration = calibration
        self.frame_id = frame_id
        self.delay_ns = int(max_transport_delay_s * 1e9)
        self.jump_ns = int(max_clock_jump_s * 1e9)
        self.last_stamp: int | None = None
        self.last_receipt: tuple[int, int] | None = None
        self.clock_faulted = False

    def process(self, stamp_ns: int, frame_id: str, angular_velocity: Sequence[float],
                linear_acceleration: Sequence[float], receipt_ros_ns: int,
                receipt_steady_ns: int) -> tuple[float, ...]:
        """Subtract fixed bias only after validation; rejected samples do not advance time."""
        if self.clock_faulted:
            raise ImuValidationError('ROS clock jump; restart IMU bringup')
        if frame_id != self.frame_id:
            raise ImuValidationError('unexpected sensor frame')
        try:
            corrected = self.calibration.correct(angular_velocity)
            _finite_values(linear_acceleration, 3, 'linear acceleration')
        except ValueError as error:
            raise ImuValidationError(str(error)) from error
        if not 0 < stamp_ns <= 0x7fffffff * 1_000_000_000 + 999_999_999:
            raise ImuValidationError('invalid ROS acquisition timestamp')
        if self.last_receipt is not None:
            ros0, steady0 = self.last_receipt
            elapsed = receipt_steady_ns - steady0
            if elapsed < 0 or abs((receipt_ros_ns - ros0) - elapsed) > self.jump_ns:
                # Keep downstream estimator histories on a continuous ROS clock.
                self.clock_faulted = True
                raise ImuValidationError('ROS clock jump; restart IMU bringup')
        age = receipt_ros_ns - stamp_ns
        if age > self.delay_ns:
            raise ImuValidationError('transport backlog exceeds limit')
        if age < -self.delay_ns:
            raise ImuValidationError('acquisition time is ahead of receipt')
        if self.last_stamp is not None and stamp_ns <= self.last_stamp:
            raise ImuValidationError('duplicate/out-of-order sample')
        self.last_stamp = stamp_ns
        self.last_receipt = (receipt_ros_ns, receipt_steady_ns)
        return corrected
