"""Attempt-limited upward correction against an immutable physical height goal."""
from dataclasses import dataclass
import math

from .cartesian import CartesianPose, validate_pose_endpoint


def validate_height_correctable_endpoint(
    actual: CartesianPose, target: CartesianPose, position_tolerance_m: float,
    orientation_tolerance_rad: float,
) -> None:
    """Defer downward Z error, retaining XY, overshoot and rotation checks."""
    guarded = CartesianPose(
        (*actual.position[:2], max(actual.position[2], target.position[2])), actual.orientation)
    validate_pose_endpoint(guarded, target, position_tolerance_m, orientation_tolerance_rad)


def lift_height_tolerance(config: dict) -> float:
    """Allowed lift shortfall, independent of the Cartesian path corridor."""
    value = config.get('lift_height_tolerance_m', .010)
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError('lift_height_tolerance_m must be finite and positive')
    height = config.get('lift_height_m')
    if type(height) in (int, float) and math.isfinite(height) and 0 < height <= value:
        raise ValueError('lift_height_tolerance_m must be less than lift_height_m')
    return float(value)


@dataclass(frozen=True)
class HeightCorrectionPolicy:
    enabled: bool = False
    step_m: float = .005
    max_attempts: int = 3
    min_progress_m: float = .001

    def __post_init__(self) -> None:
        if type(self.enabled) is not bool:
            raise ValueError('lift_height_correction_enabled must be boolean')
        for name in ('step_m', 'min_progress_m'):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError('invalid lift_height_correction_'+name)
        if self.min_progress_m > self.step_m:
            raise ValueError('lift_height_correction_min_progress_m exceeds step_m')
        if type(self.max_attempts) is not int or not 1 <= self.max_attempts <= 3:
            raise ValueError('lift_height_correction_max_attempts must be between 1 and 3')

    @classmethod
    def from_config(cls, config: dict) -> 'HeightCorrectionPolicy':
        return cls(**{name: config['lift_height_correction_'+name]
                      for name in cls.__dataclass_fields__
                      if 'lift_height_correction_'+name in config})


def height_correction_enabled(config: dict) -> bool:
    """Only the enabled feedback controller may defer downward Z error."""
    return HeightCorrectionPolicy.from_config(config).enabled


class HeightCorrectionError(RuntimeError):
    """Further upward commands cannot be justified by the measured response."""


class HeightCorrection:
    def __init__(self, goal_z: float, command_z: float, tolerance_m: float,
                 policy: HeightCorrectionPolicy):
        if (not all(math.isfinite(v) for v in (goal_z, command_z, tolerance_m))
                or tolerance_m <= 0):
            raise ValueError('Height correction requires finite heights and positive tolerance')
        self.goal_z = goal_z
        self.initial_command_z = command_z
        self.command_z = command_z
        self.tolerance_m = tolerance_m
        self.policy = policy
        self.attempts = 0
        self.previous_error: float | None = None

    def next_command(self, actual_z: float) -> float | None:
        if not math.isfinite(actual_z):
            raise HeightCorrectionError('Measured height is nonfinite')
        error = self.goal_z - actual_z
        # The caller checks XY, orientation and upward overshoot independently.
        if error <= self.tolerance_m + 1e-9:
            return None
        if not self.policy.enabled:
            raise HeightCorrectionError('Height correction is disabled')
        if (self.previous_error is not None
                and self.previous_error - error < self.policy.min_progress_m):
            raise HeightCorrectionError('Height correction made insufficient upward progress')
        if self.attempts >= self.policy.max_attempts:
            raise HeightCorrectionError('Height correction attempt limit reached')
        # Round the complete measured shortfall upward to the command grid.
        # Suppress arithmetic noise at exact multiples (e.g. 15.000000000000014 mm).
        units = error / self.policy.step_m
        if not math.isfinite(units):
            raise HeightCorrectionError('Height correction command is nonfinite')
        step = math.ceil(units - 1e-9) * self.policy.step_m
        command_z = self.command_z + step
        if not math.isfinite(command_z):
            raise HeightCorrectionError('Height correction command is nonfinite')
        self.command_z = command_z
        self.previous_error = error
        self.attempts += 1
        return command_z
