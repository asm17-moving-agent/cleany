"""Pure visualization mapping, deliberately not a motion-control calibration."""
from dataclasses import dataclass
import math
from typing import Literal

JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex',
          'wrist_flex', 'wrist_roll', 'gripper')
ENCODER_HALF_TICK_RAD = math.tau / 8192
JointLimitStatus = Literal[
    'within_limits', 'encoder_boundary',
    'outside_model_limits', 'outside_motor_limits',
]


@dataclass(frozen=True)
class JointMapping:
    name: str
    motor_id: int
    model: int
    zero_tick: float
    direction: int
    offset_rad: float
    homing_offset: int
    range_min: int
    range_max: int

    def radians(self, raw: int) -> float:
        if not 0 <= raw <= 4095:
            raise ValueError(f'{self.name}: invalid single-turn position {raw}')
        return self.offset_rad + self.direction * (raw - self.zero_tick) * math.tau / 4096


@dataclass(frozen=True)
class JointLimitAssessment:
    position_rad: float
    within_motor_range: bool
    within_model_limits: bool
    model_limit_excess_rad: float
    status: JointLimitStatus


def assess_joint_limits(
    mapping: JointMapping, raw_tick: int, model_limits_rad: tuple[float, float],
) -> JointLimitAssessment:
    """Classify preview readings without changing angles or control acceptance.

    Motor calibration and model bounds are independent checks. A model excess
    of at most half an encoder tick is labelled as a boundary reading, while
    the strict model-limit result remains false. This is not motion approval.
    """
    lower, upper = model_limits_rad
    if not math.isfinite(lower) or not math.isfinite(upper) or lower > upper:
        raise ValueError(f'{mapping.name}: invalid model limits {model_limits_rad}')
    angle = mapping.radians(raw_tick)
    within_motor = mapping.range_min <= raw_tick <= mapping.range_max
    excess = max(lower - angle, angle - upper, 0.0)
    status: JointLimitStatus
    if not within_motor:
        status = 'outside_motor_limits'
    elif excess == 0.0:
        status = 'within_limits'
    elif excess <= ENCODER_HALF_TICK_RAD:
        status = 'encoder_boundary'
    else:
        status = 'outside_model_limits'
    return JointLimitAssessment(angle, within_motor, excess == 0.0, excess, status)


def mappings(config: dict, calibrations: dict) -> list[JointMapping]:
    result = []
    for side, first in (('left', 1), ('right', 7)):
        if config['arms'][side]['first_id'] != first:
            raise ValueError(f'{side}: expected first motor ID {first}')
        calibration = calibrations[side]
        if set(calibration) != set(JOINTS):
            raise ValueError(f'{side}: incomplete calibration')
        for index, key in enumerate(JOINTS):
            cal = calibration[key]
            if cal['id'] != first + index or cal['drive_mode'] != 0:
                raise ValueError(f'{side}/{key}: calibration ID/drive mode mismatch')
            if not 0 <= cal['range_min'] < cal['range_max'] <= 4095:
                raise ValueError(f'{side}/{key}: invalid calibration range')
            params = dict(config['joints'][key])
            params.update(config.get('overrides', {}).get(side, {}).get(key, {}))
            zero = params['zero_tick']
            zero = cal['range_min'] if zero == 'range_min' else float(zero)
            direction = params['direction']
            offset = float(params['offset_rad'])
            if direction not in (-1, 1) or not math.isfinite(zero + offset):
                raise ValueError('Invalid visualization transform')
            result.append(JointMapping(
                f'{side}_{params["suffix"]}', first + index,
                2825 if key in ('shoulder_lift', 'elbow_flex') else 777,
                zero, direction, offset, cal['homing_offset'],
                cal['range_min'], cal['range_max'],
            ))
    return result
