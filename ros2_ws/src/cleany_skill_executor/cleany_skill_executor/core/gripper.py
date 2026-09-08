"""Pure gripper aperture and physical-contact helpers."""

from __future__ import annotations

import math


def aperture_centering_offset(required_opening_m: float, margin_m: float,
                              fixed_inner_x_m: float, clearance_m: float = 0.0) -> float:
    """Fixed-jaw tool correction: put the physical width between the jaws."""
    if (not all(math.isfinite(v) for v in (required_opening_m,margin_m,fixed_inner_x_m,clearance_m))
            or required_opening_m <= 0 or not 0 <= margin_m < required_opening_m
            or not 0 <= clearance_m <= margin_m/2):
        raise ValueError('Invalid aperture centering dimensions')
    return (required_opening_m-margin_m)/2-fixed_inner_x_m+clearance_m


def aligned_wrist_rolls(*, approach: tuple[float, float, float],
                       closing: tuple[float, float, float],
                       desired: tuple[float, float, float], current: float,
                       lower: float, upper: float, sign_invariant: bool) -> tuple[float, ...]:
    """Rotate a collinear-roll/TCP tool; caller must recheck FK and collisions."""
    if not all(math.isfinite(x) for x in (*approach, *closing, *desired, current, lower, upper)) or lower >= upper:
        raise ValueError('wrist alignment needs finite vectors and ordered limits')
    norm = math.sqrt(sum(x*x for x in approach))
    if norm < 1e-9:
        raise ValueError('wrist alignment axis is zero')
    axis = tuple(x/norm for x in approach)

    def projected(vector):
        dot = sum(x*y for x, y in zip(vector, axis, strict=True))
        result = tuple(x-dot*y for x, y in zip(vector, axis, strict=True))
        length = math.sqrt(sum(x*x for x in result))
        return tuple(x/length for x in result) if length > 1e-9 else None

    source, target = projected(closing), projected(desired)
    if source is None or target is None:
        return ()
    cross = (source[1]*target[2]-source[2]*target[1], source[2]*target[0]-source[0]*target[2],
             source[0]*target[1]-source[1]*target[0])
    angle = math.atan2(sum(x*y for x, y in zip(axis, cross, strict=True)),
                       sum(x*y for x, y in zip(source, target, strict=True)))
    candidates = []
    for reversal in ((0., math.pi) if sign_invariant else (0.,)):
        value = current+angle+reversal
        first, last = math.ceil((lower-value)/math.tau), math.floor((upper-value)/math.tau)
        candidates.extend(value+k*math.tau for k in range(first, last+1))
    return tuple(sorted(candidates, key=lambda x: abs(x-current)))


def opening_to_gripper_position(
    *,
    required_opening_m: float,
    opening_reduction_m: float,
    reference_aperture_m: float,
    reference_position_rad: float,
    aperture_m_per_rad: float,
    minimum_position_rad: float,
    maximum_position_rad: float,
) -> float:
    values = (
        required_opening_m,
        opening_reduction_m,
        reference_aperture_m,
        reference_position_rad,
        aperture_m_per_rad,
        minimum_position_rad,
        maximum_position_rad,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('gripper aperture calibration must be finite')
    if required_opening_m <= 0.0 or opening_reduction_m < 0.0:
        raise ValueError('gripper opening values must be positive')
    if reference_aperture_m <= 0.0 or aperture_m_per_rad <= 0.0:
        raise ValueError('gripper aperture calibration must be positive')
    if minimum_position_rad > maximum_position_rad:
        raise ValueError('gripper position limits are reversed')
    contact_aperture = max(0.0, required_opening_m - opening_reduction_m)
    position = reference_position_rad + (
        contact_aperture - reference_aperture_m
    ) / aperture_m_per_rad
    return min(maximum_position_rad, max(minimum_position_rad, position))


def is_gripper_contact_stall(
    *,
    start: float,
    actual: float,
    command: float,
    velocity: float,
    minimum_motion: float,
    minimum_residual: float,
    maximum_velocity: float,
    allow_closing_motion: bool = False,
) -> bool:
    values = (
        start,
        actual,
        command,
        velocity,
        minimum_motion,
        minimum_residual,
        maximum_velocity,
    )
    if not all(math.isfinite(value) for value in values):
        return False
    if minimum_motion <= 0.0 or minimum_residual <= 0.0:
        return False
    if maximum_velocity < 0.0 or command >= start:
        return False
    motion = start - actual
    residual = actual - command
    return (
        motion >= minimum_motion
        and residual >= minimum_residual
        # After an established grasp, tightening may continue during transport.
        # Residual collapse still rejects an empty closed jaw; opening motion
        # remains bounded. Initial acquisition always requires a true stall.
        and (velocity <= maximum_velocity if allow_closing_motion
             else abs(velocity) <= maximum_velocity)
    )
