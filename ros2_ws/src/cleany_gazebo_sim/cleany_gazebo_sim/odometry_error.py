"""Stateful simulation-only error model for planar wheel odometry."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, exp, hypot, isfinite, sin, sqrt
from random import Random


@dataclass(frozen=True)
class Pose2D:
    """Planar pose in an odometry frame."""

    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class OdometryErrorParameters:
    """Parameters for a stateful, simulation-only odometry error model."""

    forward_scale: float = 1.0
    lateral_scale: float = 1.0
    rotation_scale: float = 1.0
    forward_noise_stddev_per_sqrt_m: float = 0.0
    lateral_noise_stddev_per_sqrt_m: float = 0.0
    rotation_noise_stddev_per_sqrt_rad: float = 0.0
    yaw_bias_walk_stddev_per_sqrt_m: float = 0.0
    max_abs_yaw_bias_rad_per_m: float = 0.0
    yaw_drift_rad_per_forward_m: float = 0.0
    slip_events_per_second: float = 0.0
    slip_duration_min_sec: float = 0.0
    slip_duration_max_sec: float = 0.0
    slip_gain_min: float = 1.0
    slip_gain_max: float = 1.0
    random_seed: int = 42

    def __post_init__(self) -> None:
        finite_values = (
            self.forward_scale,
            self.lateral_scale,
            self.rotation_scale,
            self.forward_noise_stddev_per_sqrt_m,
            self.lateral_noise_stddev_per_sqrt_m,
            self.rotation_noise_stddev_per_sqrt_rad,
            self.yaw_bias_walk_stddev_per_sqrt_m,
            self.max_abs_yaw_bias_rad_per_m,
            self.yaw_drift_rad_per_forward_m,
            self.slip_events_per_second,
            self.slip_duration_min_sec,
            self.slip_duration_max_sec,
            self.slip_gain_min,
            self.slip_gain_max,
        )
        if not all(isfinite(value) for value in finite_values):
            raise ValueError('Odometry error parameters must be finite')
        if min(
            self.forward_scale,
            self.lateral_scale,
            self.rotation_scale,
            self.slip_gain_min,
            self.slip_gain_max,
        ) <= 0.0:
            raise ValueError('Scale and slip gain parameters must be positive')
        if min(
            self.forward_noise_stddev_per_sqrt_m,
            self.lateral_noise_stddev_per_sqrt_m,
            self.rotation_noise_stddev_per_sqrt_rad,
            self.yaw_bias_walk_stddev_per_sqrt_m,
            self.max_abs_yaw_bias_rad_per_m,
            self.slip_events_per_second,
            self.slip_duration_min_sec,
            self.slip_duration_max_sec,
        ) < 0.0:
            raise ValueError(
                'Noise, bias, rate and duration must be non-negative'
            )
        if self.slip_duration_min_sec > self.slip_duration_max_sec:
            raise ValueError('Minimum slip duration must not exceed maximum')
        if self.slip_gain_min > self.slip_gain_max:
            raise ValueError('Minimum slip gain must not exceed maximum')

    @property
    def is_ideal(self) -> bool:
        return (
            self.forward_scale == 1.0
            and self.lateral_scale == 1.0
            and self.rotation_scale == 1.0
            and self.forward_noise_stddev_per_sqrt_m == 0.0
            and self.lateral_noise_stddev_per_sqrt_m == 0.0
            and self.rotation_noise_stddev_per_sqrt_rad == 0.0
            and self.yaw_bias_walk_stddev_per_sqrt_m == 0.0
            and self.yaw_drift_rad_per_forward_m == 0.0
            and self.slip_events_per_second == 0.0
        )


@dataclass(frozen=True)
class NoisyOdometryEstimate:
    """Perturbed pose, body velocity and internal diagnostic state."""

    pose: Pose2D
    linear_x_mps: float
    linear_y_mps: float
    angular_z_rps: float
    yaw_bias_rad_per_m: float
    slip_active: bool


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to the closed interval around zero."""

    return atan2(sin(angle_rad), cos(angle_rad))


class StatefulOdometryError:
    """Apply correlated errors to relative planar odometry increments."""

    def __init__(self, parameters: OdometryErrorParameters) -> None:
        self._parameters = parameters
        self._random = Random(parameters.random_seed)
        self._previous_raw_pose: Pose2D | None = None
        self._previous_stamp_s: float | None = None
        self._output_pose: Pose2D | None = None
        self._yaw_bias_rad_per_m = 0.0
        self._slip_until_s = float('-inf')
        self._slip_gain = 1.0

    def update(
        self, raw_pose: Pose2D, stamp_s: float
    ) -> NoisyOdometryEstimate:
        if not all(
            isfinite(value)
            for value in (
                raw_pose.x_m,
                raw_pose.y_m,
                raw_pose.yaw_rad,
                stamp_s,
            )
        ):
            raise ValueError('Pose and timestamp must be finite')
        if (
            self._previous_raw_pose is None
            or self._previous_stamp_s is None
            or stamp_s <= self._previous_stamp_s
        ):
            return self._initialize(raw_pose, stamp_s)

        previous_raw = self._previous_raw_pose
        previous_stamp_s = self._previous_stamp_s
        output_pose = self._output_pose
        assert output_pose is not None
        dt_s = stamp_s - previous_stamp_s

        world_x_m = raw_pose.x_m - previous_raw.x_m
        world_y_m = raw_pose.y_m - previous_raw.y_m
        previous_cos = cos(previous_raw.yaw_rad)
        previous_sin = sin(previous_raw.yaw_rad)
        raw_forward_m = previous_cos * world_x_m + previous_sin * world_y_m
        raw_lateral_m = -previous_sin * world_x_m + previous_cos * world_y_m
        raw_rotation_rad = wrap_angle(raw_pose.yaw_rad - previous_raw.yaw_rad)

        if self._parameters.is_ideal:
            estimate = NoisyOdometryEstimate(
                pose=raw_pose,
                linear_x_mps=raw_forward_m / dt_s,
                linear_y_mps=raw_lateral_m / dt_s,
                angular_z_rps=raw_rotation_rad / dt_s,
                yaw_bias_rad_per_m=0.0,
                slip_active=False,
            )
            self._remember(raw_pose, stamp_s, raw_pose)
            return estimate

        forward_m = raw_forward_m * self._parameters.forward_scale
        lateral_m = raw_lateral_m * self._parameters.lateral_scale
        rotation_rad = raw_rotation_rad * self._parameters.rotation_scale
        rotation_rad += (
            self._parameters.yaw_drift_rad_per_forward_m * raw_forward_m
        )

        translation_m = hypot(raw_forward_m, raw_lateral_m)
        if translation_m > 0.0:
            self._update_yaw_bias(translation_m)
            rotation_rad += self._yaw_bias_rad_per_m * raw_forward_m

        moving = translation_m > 0.0 or abs(raw_rotation_rad) > 0.0
        slip_active = self._update_slip(stamp_s, dt_s, moving)
        if slip_active:
            forward_m *= self._slip_gain
            lateral_m *= self._slip_gain
            rotation_rad *= self._slip_gain

        forward_m += self._distance_noise(
            self._parameters.forward_noise_stddev_per_sqrt_m,
            abs(raw_forward_m),
        )
        lateral_m += self._distance_noise(
            self._parameters.lateral_noise_stddev_per_sqrt_m,
            abs(raw_lateral_m),
        )
        rotation_rad += self._distance_noise(
            self._parameters.rotation_noise_stddev_per_sqrt_rad,
            abs(raw_rotation_rad),
        )

        midpoint_yaw = output_pose.yaw_rad + rotation_rad / 2.0
        next_pose = Pose2D(
            x_m=(
                output_pose.x_m
                + cos(midpoint_yaw) * forward_m
                - sin(midpoint_yaw) * lateral_m
            ),
            y_m=(
                output_pose.y_m
                + sin(midpoint_yaw) * forward_m
                + cos(midpoint_yaw) * lateral_m
            ),
            yaw_rad=wrap_angle(output_pose.yaw_rad + rotation_rad),
        )
        estimate = NoisyOdometryEstimate(
            pose=next_pose,
            linear_x_mps=forward_m / dt_s,
            linear_y_mps=lateral_m / dt_s,
            angular_z_rps=rotation_rad / dt_s,
            yaw_bias_rad_per_m=self._yaw_bias_rad_per_m,
            slip_active=slip_active,
        )
        self._remember(raw_pose, stamp_s, next_pose)
        return estimate

    def _initialize(
        self, raw_pose: Pose2D, stamp_s: float
    ) -> NoisyOdometryEstimate:
        self._yaw_bias_rad_per_m = 0.0
        self._slip_until_s = float('-inf')
        self._slip_gain = 1.0
        self._remember(raw_pose, stamp_s, raw_pose)
        return NoisyOdometryEstimate(raw_pose, 0.0, 0.0, 0.0, 0.0, False)

    def _remember(
        self, raw_pose: Pose2D, stamp_s: float, output_pose: Pose2D
    ) -> None:
        self._previous_raw_pose = raw_pose
        self._previous_stamp_s = stamp_s
        self._output_pose = output_pose

    def _distance_noise(self, coefficient: float, amount: float) -> float:
        if coefficient == 0.0 or amount == 0.0:
            return 0.0
        return self._random.gauss(0.0, coefficient * sqrt(amount))

    def _update_yaw_bias(self, translation_m: float) -> None:
        walk = self._parameters.yaw_bias_walk_stddev_per_sqrt_m
        if walk == 0.0:
            return
        self._yaw_bias_rad_per_m += self._random.gauss(
            0.0, walk * sqrt(translation_m)
        )
        limit = self._parameters.max_abs_yaw_bias_rad_per_m
        self._yaw_bias_rad_per_m = max(
            -limit, min(limit, self._yaw_bias_rad_per_m)
        )

    def _update_slip(
        self, stamp_s: float, dt_s: float, moving: bool
    ) -> bool:
        if stamp_s < self._slip_until_s:
            return True
        self._slip_gain = 1.0
        rate = self._parameters.slip_events_per_second
        if not moving or rate == 0.0:
            return False
        probability = 1.0 - exp(-rate * dt_s)
        if self._random.random() >= probability:
            return False
        duration = self._random.uniform(
            self._parameters.slip_duration_min_sec,
            self._parameters.slip_duration_max_sec,
        )
        self._slip_until_s = stamp_s + duration
        self._slip_gain = self._random.uniform(
            self._parameters.slip_gain_min,
            self._parameters.slip_gain_max,
        )
        return True
