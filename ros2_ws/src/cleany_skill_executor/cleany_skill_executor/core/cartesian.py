"""Finite, direction-preserving checks for sampled Cartesian motion plans."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence
from statistics import median

from .alignment import quaternion_axis, quaternion_from_rotation, rotation_from_quaternion


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


def validate_position_endpoint(actual: CartesianPose, target: CartesianPose,
                               position_tolerance_m: float) -> None:
    """Validate Cartesian translation without making orientation a stop condition."""
    if not math.isfinite(position_tolerance_m) or position_tolerance_m <= 0:
        raise ValueError('Cartesian position tolerance must be finite and positive')
    distance = math.dist(actual.position, target.position)
    if distance > position_tolerance_m:
        raise ValueError(f'Cartesian endpoint mismatch: position={distance:.6f}m')


def validate_corridor_samples(samples: Sequence[CartesianPose], start: CartesianPose,
                              target: CartesianPose, radius_m: float,
                              orientation_tolerance_rad: float | None,
                              endpoint_orientation_tolerance_rad: float | None) -> tuple[float, float]:
    """Validate returned FK samples, not a continuous-collision certificate."""
    line_corridor(start.position, target.position, radius_m)
    orientation_enabled = orientation_tolerance_rad is not None
    if orientation_enabled != (endpoint_orientation_tolerance_rad is not None):
        raise ValueError('Cartesian orientation tolerances must both be enabled or disabled')
    if orientation_enabled and (
            not math.isfinite(orientation_tolerance_rad)
            or orientation_tolerance_rad <= 0
            or not math.isfinite(endpoint_orientation_tolerance_rad)
            or endpoint_orientation_tolerance_rad <= 0):
        raise ValueError('corridor orientation tolerances must be finite and positive')
    if len(samples) < 2:
        raise ValueError('Cartesian plan requires at least two FK samples')
    if orientation_enabled:
        validate_pose_endpoint(
            samples[0], start, radius_m, endpoint_orientation_tolerance_rad)
        validate_pose_endpoint(
            samples[-1], target, radius_m, endpoint_orientation_tolerance_rad)
    else:
        validate_position_endpoint(samples[0], start, radius_m)
        validate_position_endpoint(samples[-1], target, radius_m)
    length = math.dist(start.position, target.position)
    direction = tuple((b-a)/length for a, b in zip(start.position, target.position, strict=True))
    maximum_lateral = maximum_angle = 0.
    previous = 0.
    for sample in samples:
        offset = tuple(b-a for a, b in zip(start.position, sample.position, strict=True))
        progress = sum(a*b for a, b in zip(offset, direction, strict=True))
        lateral = math.sqrt(sum((x-progress*d)**2 for x, d in zip(offset, direction, strict=True)))
        angle = 0.
        if orientation_enabled:
            expected = interpolate_orientation(
                start.orientation, target.orientation, progress/length)
            angle = orientation_error_rad(sample.orientation, expected)
        if (progress < -radius_m or progress > length+radius_m or
                progress < previous-radius_m or lateral > radius_m or
                (orientation_enabled and angle > orientation_tolerance_rad)):
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


def validate_height_correctable_endpoint(
    actual: CartesianPose, target: CartesianPose, position_tolerance_m: float,
    orientation_tolerance_rad: float,
) -> None:
    """Defer downward Z error, retaining XY, overshoot and rotation checks."""
    guarded = CartesianPose(
        (*actual.position[:2], max(actual.position[2], target.position[2])), actual.orientation)
    validate_pose_endpoint(guarded, target, position_tolerance_m, orientation_tolerance_rad)


def estimate_measured_path_pose(measured_start: CartesianPose, command_start: CartesianPose,
                                command_pose: CartesianPose) -> CartesianPose:
    """Apply commanded base-frame displacement/rotation to a measured origin.

    This estimate selects a point on a recorded path. Arrival still requires
    fresh measured FK; a supporting motor command is never the current TCP.
    """
    rotation = (rotation_from_quaternion(command_pose.orientation)
                @ rotation_from_quaternion(command_start.orientation).T
                @ rotation_from_quaternion(measured_start.orientation))
    return CartesianPose(
        tuple(actual+end-start for actual, start, end in zip(
            measured_start.position, command_start.position, command_pose.position, strict=True)),
        quaternion_from_rotation(rotation),
    )


def observation_corrected_pose(target: CartesianPose, reference_center: Vector3,
                               observed_center: Vector3,
                               maximum_correction_m: float) -> CartesianPose:
    """Translate a planned pose by the same displacement as a tracked surface."""
    if (not math.isfinite(maximum_correction_m) or maximum_correction_m <= 0
            or any(len(point) != 3 or not all(math.isfinite(value) for value in point)
                   for point in (reference_center, observed_center))):
        raise ValueError('Invalid visual correction inputs')
    correction = tuple(current-origin for origin, current in zip(
        reference_center, observed_center, strict=True))
    distance = math.sqrt(sum(value*value for value in correction))
    if distance > maximum_correction_m:
        raise ValueError(f'Visual target correction exceeds limit: {distance:.6f}m')
    return CartesianPose(
        tuple(value+offset for value, offset in zip(
            target.position, correction, strict=True)),
        target.orientation,
    )


def coordinate_median(points: Sequence[Vector3]) -> Vector3:
    """Return a component-wise median for an odd set of finite 3D samples."""
    if (len(points) < 1 or len(points) % 2 == 0
            or any(len(point) != 3 or not all(math.isfinite(value) for value in point)
                   for point in points)):
        raise ValueError('median center requires an odd set of finite 3D points')
    return tuple(float(median(point[axis] for point in points)) for axis in range(3))


@dataclass(frozen=True)
class GraspCaptureMetrics:
    distance_m: float
    approach_remaining_m: float
    lateral_error_m: float

    def inside(self, depth_m: float, lateral_tolerance_m: float,
               reverse_tolerance_m: float) -> bool:
        limits = (depth_m, lateral_tolerance_m, reverse_tolerance_m)
        if not all(math.isfinite(value) and value >= 0 for value in limits):
            raise ValueError('grasp capture limits must be finite and nonnegative')
        return (
            self.distance_m <= depth_m
            and -reverse_tolerance_m <= self.approach_remaining_m <= depth_m
            and self.lateral_error_m <= lateral_tolerance_m
        )


def grasp_capture_metrics(current: CartesianPose,
                          target: CartesianPose) -> GraspCaptureMetrics:
    """Resolve remaining TCP translation along and across the grasp approach axis."""
    displacement = tuple(
        target_value-current_value
        for current_value, target_value in zip(
            current.position, target.position, strict=True)
    )
    distance = math.sqrt(sum(value*value for value in displacement))
    approach = quaternion_axis(target.orientation, (0., -1., 0.))
    along = sum(value*axis for value, axis in zip(
        displacement, approach, strict=True))
    lateral = math.sqrt(max(0., distance*distance-along*along))
    return GraspCaptureMetrics(distance, along, lateral)


def bounded_cartesian_step(start: CartesianPose, target: CartesianPose,
                           maximum_translation_m: float) -> CartesianPose:
    """Return the next pose on a straight Cartesian segment with a bounded step."""
    if not math.isfinite(maximum_translation_m) or maximum_translation_m <= 0:
        raise ValueError('Cartesian step must be finite and positive')
    distance = math.dist(start.position, target.position)
    if distance <= maximum_translation_m:
        return target
    fraction = maximum_translation_m/distance
    return CartesianPose(
        tuple(a+(b-a)*fraction for a, b in zip(
            start.position, target.position, strict=True)),
        interpolate_orientation(start.orientation, target.orientation, fraction),
    )
