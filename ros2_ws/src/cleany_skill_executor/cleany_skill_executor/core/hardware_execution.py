"""Physical execution limits. No ROS, serial writes, or implicit calibration."""
from dataclasses import dataclass
from collections.abc import Sequence
import math


class ExecutionCanceled(RuntimeError):
    """Tracked cancellation; the execution owner must independently prove stop."""


class CollisionValidationError(RuntimeError):
    """A predicted collision rejected a trajectory before its next command."""

    def __init__(self, message: str, *, phase: str, sample_index: int,
                 contacts: tuple[dict, ...] = (), octomap_arm_contact: bool = False):
        super().__init__(message)
        self.phase = phase
        self.sample_index = sample_index
        self.contacts = contacts
        self.octomap_arm_contact = octomap_arm_contact
        self.motion_started = False

    @property
    def entry_blocked(self) -> bool:
        return self.phase == 'hold_bridge' or self.sample_index == 0


class CommandPathValidationError(RuntimeError):
    """The final command failed geometry validation before motion submission."""

    def __init__(self, message: str):
        super().__init__(message)
        self.motion_started = False


class JointRangeValidationError(ValueError):
    """Preserve the exact joint/sample and both model and encoder bounds."""

    def __init__(self, value: float, lower: float, upper: float, raw: int,
                 raw_min: float, raw_max: float, *, joint_name: str = '', sample_index: int = -1):
        self.value, self.lower, self.upper = value, lower, upper
        self.raw, self.raw_min, self.raw_max = raw, raw_min, raw_max
        self.joint_name, self.sample_index = joint_name, sample_index
        self.motion_started = False
        super().__init__(
            'Trajectory exceeds physical joint/calibration range: '
            f'joint={joint_name or "unknown"} sample={sample_index} '
            f'position={value:.9f}rad bounds=[{lower:.9f},{upper:.9f}]rad '
            f'raw={raw} raw_bounds=[{raw_min:g},{raw_max:g}]')

    def at(self, joint_name: str, sample_index: int) -> 'JointRangeValidationError':
        return JointRangeValidationError(self.value, self.lower, self.upper, self.raw,
            self.raw_min, self.raw_max, joint_name=joint_name, sample_index=sample_index)

import xml.etree.ElementTree as ET

from .physical_grasp_selection import ARM_JOINT_NAMES
from .joint_path import JointPathSample, executable_joint_path


@dataclass(frozen=True)
class ArmHold:
    """Last successfully sent endpoint and its un-biased physical target."""
    nominal: tuple[float, ...]
    command: tuple[float, ...]


class CommunicationRecovered(RuntimeError):
    """Interrupt an active goal so its remaining path can use fresh feedback."""

    def __init__(self, count: int):
        super().__init__(f'Motor READ communication recovered; generation={count}')
        self.count = count
        self.cancellation_confirmed = False


def bridge_joint_path(
    positions: Sequence[Sequence[float]], measured: Sequence[float], maximum_delta: float,
) -> list[tuple[float, ...]]:
    """Add the measured entry pose without changing the planned endpoint."""
    path = [tuple(float(value) for value in point) for point in positions]
    actual = tuple(float(value) for value in measured)
    if (len(path) < 2 or not actual or any(len(point) != len(actual) for point in path)
            or not all(math.isfinite(value) for point in [actual, *path] for value in point)
            or not math.isfinite(maximum_delta) or maximum_delta <= 0):
        raise ValueError('Invalid start-bridge path or limit')
    gap = max(abs(a-b) for a, b in zip(actual, path[0], strict=True))
    if gap > maximum_delta+1e-9:
        raise ValueError(f'Physical start changed too far for bridge: {gap:.6f}rad')
    return [actual, *path] if gap > 1e-9 else path


def remaining_joint_path(
    positions: Sequence[Sequence[float]], measured: Sequence[float], *, maximum_delta: float = .03,
) -> list[tuple[float, ...]]:
    """Keep the nearest remaining segment after a confirmed controller stop."""
    path = [tuple(float(value) for value in point) for point in positions]
    actual = tuple(float(value) for value in measured)
    if (len(path) < 2 or not actual or any(len(point) != len(actual) for point in path)
            or not all(math.isfinite(value) for point in [actual, *path] for value in point)
            or not math.isfinite(maximum_delta) or maximum_delta <= 0):
        raise ValueError('Invalid recovery path')
    # Project onto segments rather than a nearby endpoint, avoiding an unnecessary
    # return to the original trajectory start after the robot has already moved.
    best = (math.inf, 0)
    for index, (start, end) in enumerate(zip(path, path[1:])):
        delta = tuple(b-a for a, b in zip(start, end, strict=True))
        length = sum(value*value for value in delta)
        fraction = (max(0., min(1., sum((v-a)*d for v, a, d in zip(
            actual, start, delta, strict=True))/length)) if length > 1e-18 else 0.)
        distance = sum((v-(a+fraction*d))**2 for v, a, d in zip(
            actual, start, delta, strict=True))
        if distance < best[0]:
            best = distance, index
    if math.sqrt(best[0]) > maximum_delta+1e-9:
        raise ValueError('Recovered joints departed too far from the remaining path')
    return [actual, *path[best[1]+1:]]


def continue_joint_path(positions, hold: ArmHold, maximum_bias: float):
    """Join the active command without shifting the new path or destination.

    Any load compensation belongs to the caller's explicit command conversion.
    A previous phase's nominal/command offset must not move a new physical goal.
    The caller validates the resulting cubic and measured-to-command bridge.
    """
    path = executable_joint_path(list(positions), simplification_tolerance=0.)
    width = len(path.positions[0])
    if (len(hold.nominal) != width or len(hold.command) != width
            or not math.isfinite(maximum_bias) or maximum_bias <= 0
            or not all(math.isfinite(v) for v in (*hold.nominal, *hold.command))):
        raise ValueError('Invalid held arm command')
    bias = tuple(c-n for c, n in zip(hold.command, hold.nominal, strict=True))
    if any(abs(value) > maximum_bias + 1e-9 for value in bias):
        raise ValueError('Held arm bias exceeds correction limit')
    result = [tuple(float(value) for value in point) for point in path.positions]
    result[0] = hold.command
    return result


@dataclass(frozen=True)
class PhysicalJointLimit:
    lower: float
    upper: float
    velocity: float
    acceleration: float
    zero: float
    direction: float
    offset: float
    raw_min: float
    raw_max: float

    def position_bounds(self) -> tuple[float, float]:
        raw_positions = tuple(
            self.offset + self.direction * (raw - self.zero) * math.tau / 4096
            for raw in (self.raw_min, self.raw_max)
        )
        return (
            max(self.lower, min(raw_positions)),
            min(self.upper, max(raw_positions)),
        )


    def check(self, value: float, *, check_position_range: bool = True) -> None:
        if not math.isfinite(value):
            raise ValueError('Nonfinite physical joint position')
        raw = round(self.zero + self.direction * (value-self.offset)*4096/math.tau)
        if ((check_position_range and not self.lower <= value <= self.upper)
                or not self.raw_min <= raw <= self.raw_max):
            raise JointRangeValidationError(value, self.lower, self.upper, raw,
                                            self.raw_min, self.raw_max)


def physical_limits(description: str, arm: str) -> dict[str, PhysicalJointLimit]:
    if arm not in ARM_JOINT_NAMES:
        raise ValueError('Unknown arm')
    tree = ET.fromstring(description)
    systems = tree.findall('ros2_control')
    if len(systems) != 1:
        raise ValueError('Expected one physical hardware system')
    system = systems[0]
    if system.findtext('hardware/plugin') != 'cleany_feetech_hardware/FeetechSystem':
        raise ValueError('Not the Feetech physical driver')
    if system.findtext("hardware/param[@name='read_only']") != 'false':
        raise ValueError('Physical driver is read-only')
    required = {*ARM_JOINT_NAMES[arm], f'{arm}_gripper_joint'}
    result = {}
    for joint in system.findall('joint'):
        name = joint.get('name')
        if name not in required or joint.find('command_interface') is None:
            continue
        p = {v.get('name'): float(v.text) for v in joint.findall('param')}
        values = [p[k] for k in ('lower', 'upper', 'max_velocity', 'max_acceleration',
                                  'zero_tick', 'direction', 'offset_rad', 'range_min', 'range_max')]
        limit = PhysicalJointLimit(*values)
        if (not all(math.isfinite(v) for v in values) or limit.lower >= limit.upper
                or min(limit.velocity, limit.acceleration) <= 0 or abs(limit.direction) != 1):
            raise ValueError('Invalid physical limits')
        result[name] = limit
    if set(result) != required:
        raise ValueError('Selected arm/gripper are not all commandable')
    return result


@dataclass(frozen=True)
class BoundedJointPath:
    """One validated cubic represented by dense checks and exact command knots."""

    samples: list[JointPathSample]
    duration: float
    command_knots: tuple[JointPathSample, ...]


def prepare_bounded_path(names, positions, limits, *, velocity: float, acceleration: float,
                 step: float = .01, maximum_points: int = 10000,
                 simplification_tolerance: float = .002,
                 position_range_exemptions: Sequence[str] = (),
                 minimum_duration_sec: float = 1.):
    if not names or len(set(names)) != len(names) or not set(names) <= limits.keys():
        raise ValueError('Unexpected commanded joints')
    exempt = frozenset(position_range_exemptions)
    if not exempt <= set(names):
        raise ValueError('Position-range exemptions must name commanded joints')
    if (isinstance(minimum_duration_sec, bool) or not all(math.isfinite(x) and x > 0
            for x in (velocity, acceleration, step, minimum_duration_sec))):
        raise ValueError('Invalid execution limits')
    if any(len(p) != len(names) for p in positions):
        raise ValueError('Malformed trajectory positions')
    velocity_limits = [min(velocity, limits[n].velocity*.8) for n in names]
    acceleration_limits = [min(acceleration, limits[n].acceleration*.8) for n in names]
    path = executable_joint_path(
        list(positions), simplification_tolerance=simplification_tolerance).retimed(
            velocity_limits, acceleration_limits)
    samples = path.samples(step, 2, maximum_points)
    for index, sample in enumerate(samples):
        for name, value in zip(names, sample.positions):
            try:
                if name in exempt:
                    limits[name].check(value, check_position_range=False)
                else:
                    limits[name].check(value)
            except JointRangeValidationError as error:
                raise error.at(name, index) from error
    duration = max(minimum_duration_sec,
                   path.minimum_duration(velocity_limits, acceleration_limits)*1.05)
    knots = (path.sample(0, 0.), *(path.sample(i, 1.) for i in range(len(path.positions)-1)))
    return BoundedJointPath(samples, duration, knots)


def bounded_path(names, positions, limits, *, velocity: float, acceleration: float,
                 step: float = .01, maximum_points: int = 10000,
                 simplification_tolerance: float = .002,
                 position_range_exemptions: Sequence[str] = (),
                 minimum_duration_sec: float = 1.):
    prepared = prepare_bounded_path(names, positions, limits, velocity=velocity,
        acceleration=acceleration, step=step, maximum_points=maximum_points,
        simplification_tolerance=simplification_tolerance,
        position_range_exemptions=position_range_exemptions,
        minimum_duration_sec=minimum_duration_sec)
    return prepared.samples, prepared.duration
