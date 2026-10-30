"""One upward correction against an immutable physical height goal."""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class HeightCorrectionPolicy:
    enabled: bool = False
    step_m: float = .005
    tolerance_m: float = 0.

    def __post_init__(self) -> None:
        if type(self.enabled) is not bool:
            raise ValueError('lift_height_correction_enabled must be boolean')
        if (type(self.step_m) not in (int, float) or not math.isfinite(self.step_m)
                or self.step_m <= 0):
            raise ValueError('invalid lift_height_correction_step_m')
        if (type(self.tolerance_m) not in (int, float) or not math.isfinite(self.tolerance_m)
                or self.tolerance_m < 0):
            raise ValueError('invalid lift_height_tolerance_m')

    @classmethod
    def from_config(cls, config: dict) -> 'HeightCorrectionPolicy':
        fields = {'enabled': 'lift_height_correction_enabled',
                  'step_m': 'lift_height_correction_step_m',
                  'tolerance_m': 'lift_height_tolerance_m'}
        return cls(**{name: config[key] for name, key in fields.items() if key in config})


def height_correction_enabled(config: dict) -> bool:
    """Only the enabled one-shot correction may defer downward Z error."""
    return HeightCorrectionPolicy.from_config(config).enabled


class HeightCorrectionError(RuntimeError):
    """The measured shortfall cannot produce a valid upward command."""


class HeightCorrection:
    def __init__(self, goal_z: float, command_z: float, policy: HeightCorrectionPolicy):
        if not all(math.isfinite(v) for v in (goal_z, command_z)):
            raise ValueError('Height correction requires finite heights')
        self.goal_z = goal_z
        self.command_z = command_z
        self.policy = policy
        self.attempts = 0

    def next_command(self, actual_z: float) -> float | None:
        """Correct shortfall beyond tolerance once, rounded to the command grid."""
        if self.attempts:
            return None
        if not math.isfinite(actual_z):
            raise HeightCorrectionError('Measured height is nonfinite')
        error = self.goal_z - actual_z
        # The caller checks XY, orientation and upward overshoot independently.
        if error <= self.policy.tolerance_m + 1e-9:
            return None
        if not self.policy.enabled:
            raise HeightCorrectionError('Height correction is disabled')
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
        self.attempts += 1
        return command_z
