"""Finite, direction-preserving checks for sampled Cartesian motion plans."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


def execution_wall_timeout(duration_sec: float, factor: float,
                           margin_sec: float, minimum_sec: float) -> float:
    """Bound execution in wall time without assuming a real-time simulator."""
    values = (duration_sec, factor, margin_sec, minimum_sec)
    if (not all(math.isfinite(value) for value in values)
            or duration_sec <= 0 or factor < 1 or margin_sec < 0 or minimum_sec <= 0):
        raise ValueError('Invalid trajectory execution wall timeout configuration')
    timeout = max(minimum_sec, duration_sec * factor + margin_sec)
    if not math.isfinite(timeout):
        raise ValueError('Trajectory execution wall timeout overflow')
    return timeout


def unit_quaternion(value: Quaternion) -> Quaternion:
    if len(value) != 4 or not all(math.isfinite(x) for x in value):
        raise ValueError('orientation must be a finite quaternion')
    norm = math.sqrt(sum(x*x for x in value))
    if norm < 1e-9:
        raise ValueError('orientation quaternion has zero length')
    return tuple(x/norm for x in value)


def orientation_error_rad(left: Quaternion, right: Quaternion) -> float:
    left, right = unit_quaternion(left), unit_quaternion(right)
    dot = abs(sum(a*b for a, b in zip(left, right, strict=True)))
    return 2 * math.acos(min(1., dot))


def interpolate_orientation(start: Quaternion, end: Quaternion, fraction: float) -> Quaternion:
    start, end = unit_quaternion(start), unit_quaternion(end)
    dot = sum(a*b for a, b in zip(start, end, strict=True))
    if dot < 0:
        end, dot = tuple(-x for x in end), -dot
    fraction = min(1., max(0., fraction))
    if dot > .9995:
        return unit_quaternion(tuple(a+(b-a)*fraction for a, b in zip(start, end, strict=True)))
    angle = math.acos(min(1., dot))
    a, b = math.sin((1-fraction)*angle), math.sin(fraction*angle)
    return unit_quaternion(tuple(a*x+b*y for x, y in zip(start, end, strict=True)))


@dataclass(frozen=True)
class CartesianPose:
    position: Vector3
    orientation: Quaternion

    def __post_init__(self):
        if len(self.position) != 3 or not all(math.isfinite(x) for x in self.position):
            raise ValueError('position must have three finite coordinates')
        unit_quaternion(self.orientation)


@dataclass(frozen=True)
class LineCorridor:
    center: Vector3
    orientation: Quaternion
    length: float
    radius: float


def line_corridor(start: Vector3, end: Vector3, radius: float) -> LineCorridor:
    """Cylinder along the segment, with one radius of endpoint padding."""
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError('corridor radius must be finite and positive')
    for point in (start, end):
        if len(point) != 3 or not all(math.isfinite(x) for x in point):
            raise ValueError('corridor endpoint must be finite')
    length = math.dist(start, end)
    if length < 1e-9:
        raise ValueError('corridor requires distinct endpoints')
    direction = tuple((b-a)/length for a, b in zip(start, end, strict=True))
    # Shortest rotation from the cylinder's local +Z to the segment direction.
    if direction[2] < -1 + 1e-12:
        orientation = (1., 0., 0., 0.)
    else:
        orientation = unit_quaternion((-direction[1], direction[0], 0., 1+direction[2]))
    return LineCorridor(tuple((a+b)/2 for a, b in zip(start, end, strict=True)),
                        orientation, length + 2*radius, radius)


def validate_pose_endpoint(actual: CartesianPose, target: CartesianPose,
                           position_tolerance_m: float, orientation_tolerance_rad: float) -> None:
    for limit in (position_tolerance_m, orientation_tolerance_rad):
        if not math.isfinite(limit) or limit <= 0:
            raise ValueError('Cartesian tolerances must be finite and positive')
    distance = math.dist(actual.position, target.position)
    angle = orientation_error_rad(actual.orientation, target.orientation)
    if distance > position_tolerance_m or angle > orientation_tolerance_rad:
        raise ValueError(f'Cartesian endpoint mismatch: position={distance:.6f}m '
                         f'orientation={math.degrees(angle):.3f}deg')


def validate_corridor_samples(samples: Sequence[CartesianPose], start: CartesianPose,
                              target: CartesianPose, radius_m: float,
                              orientation_tolerance_rad: float,
                              endpoint_orientation_tolerance_rad: float) -> tuple[float, float]:
    """Validate returned FK samples, not a continuous-collision certificate."""
    line_corridor(start.position, target.position, radius_m)
    if not math.isfinite(orientation_tolerance_rad) or orientation_tolerance_rad <= 0:
        raise ValueError('corridor orientation tolerance must be finite and positive')
    if len(samples) < 2:
        raise ValueError('Cartesian plan requires at least two FK samples')
    validate_pose_endpoint(samples[0], start, radius_m, endpoint_orientation_tolerance_rad)
    validate_pose_endpoint(samples[-1], target, radius_m, endpoint_orientation_tolerance_rad)
    length = math.dist(start.position, target.position)
    direction = tuple((b-a)/length for a, b in zip(start.position, target.position, strict=True))
    maximum_lateral = maximum_angle = 0.
    previous = 0.
    for sample in samples:
        offset = tuple(b-a for a, b in zip(start.position, sample.position, strict=True))
        progress = sum(a*b for a, b in zip(offset, direction, strict=True))
        lateral = math.sqrt(sum((x-progress*d)**2 for x, d in zip(offset, direction, strict=True)))
        expected = interpolate_orientation(start.orientation, target.orientation, progress/length)
        angle = orientation_error_rad(sample.orientation, expected)
        if (progress < -radius_m or progress > length+radius_m or
                progress < previous-radius_m or lateral > radius_m or
                angle > orientation_tolerance_rad):
            raise ValueError(f'Cartesian corridor violated: progress={progress:.6f}m '
                             f'lateral={lateral:.6f}m orientation={math.degrees(angle):.3f}deg')
        previous = max(previous, progress)
        maximum_lateral, maximum_angle = max(maximum_lateral, lateral), max(maximum_angle, angle)
    return maximum_lateral, maximum_angle


def sampled_time_scale(samples: Sequence[CartesianPose], times: Sequence[float],
                       translation_speed: float, rotation_speed: float,
                       translation_acceleration: float, margin: float) -> float:
    """Uniform slowdown from sampled FK rates; not a continuous speed proof."""
    limits = (translation_speed, rotation_speed, translation_acceleration, margin)
    if (len(samples) < 2 or len(times) != len(samples) or
            not all(math.isfinite(x) and x > 0 for x in limits) or margin < 1 or
            not all(math.isfinite(x) and x >= 0 for x in times)):
        raise ValueError('invalid Cartesian timing samples or limits')
    factor = 1.
    previous_velocity = (0., 0., 0.)
    previous_dt = None
    for i in range(1, len(samples)):
        dt = times[i]-times[i-1]
        if dt <= 0:
            raise ValueError('trajectory sample times must increase')
        velocity = tuple((b-a)/dt for a, b in zip(samples[i-1].position, samples[i].position, strict=True))
        speed = math.sqrt(sum(x*x for x in velocity))
        rotation = orientation_error_rad(samples[i-1].orientation, samples[i].orientation)/dt
        acceleration_dt = dt/2 if previous_dt is None else (previous_dt+dt)/2
        acceleration = math.dist(previous_velocity, velocity)/acceleration_dt
        factor = max(factor, speed/translation_speed, rotation/rotation_speed,
                     math.sqrt(acceleration/translation_acceleration))
        previous_velocity, previous_dt = velocity, dt
    factor = max(factor, math.sqrt(math.sqrt(sum(x*x for x in previous_velocity)) /
                                   (previous_dt/2*translation_acceleration)))
    return factor * margin
