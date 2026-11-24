"""Replay the actual controller splines without IK, simplification or rebiasing."""
from dataclasses import dataclass, field
from collections.abc import Callable, Sequence
from bisect import bisect_right
from functools import cached_property
import math

from numpy.polynomial.polynomial import polyroots

from .hardware_execution import (
    CommandPathValidationError, JointRangeValidationError, PhysicalJointLimit, joint_rate_limits,
)
from .joint_path import JointPathSample, spline_coefficients


@dataclass(frozen=True)
class CommandPoint:
    time_ns: int
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    accelerations: tuple[float, ...] = ()


@dataclass(frozen=True)
class RecordedTrajectory:
    joint_names: tuple[str, ...]
    points: tuple[CommandPoint, ...]

    @cached_property
    def _times(self) -> tuple[int, ...]:
        return tuple(point.time_ns for point in self.points)

    @cached_property
    def _segments(self) -> tuple:
        segments = []
        for first, last in zip(self.points, self.points[1:]):
            dt = (last.time_ns-first.time_ns)/1e9
            width = len(self.joint_names)
            if first.accelerations and last.accelerations:
                a0 = tuple(a*dt**2 for a in first.accelerations)
                a1 = tuple(a*dt**2 for a in last.accelerations)
            else:
                a0 = a1 = (None,)*width
            coefficients = tuple(
                spline_coefficients(q0, q1, dt*v0, dt*v1, b0, b1)
                for q0, q1, v0, v1, b0, b1 in zip(first.positions, last.positions,
                    first.velocities, last.velocities, a0, a1, strict=True))
            segments.append((dt, coefficients))
        return tuple(segments)

    def validate(self) -> None:
        width = len(self.joint_names)
        if (not width or len(set(self.joint_names)) != width or len(self.points) < 2
                or self.points[0].time_ns != 0):
            raise CommandPathValidationError('Malformed recorded arm trajectory')
        previous = -1
        for point in self.points:
            if (type(point.time_ns) is not int or point.time_ns <= previous
                    or len(point.positions) != width or len(point.velocities) != width
                    or (point.accelerations and len(point.accelerations) != width)
                    or not all(math.isfinite(q) for q in (
                        *point.positions, *point.velocities, *point.accelerations))):
                raise CommandPathValidationError('Malformed recorded arm trajectory point')
            previous = point.time_ns

    def reversed_with_roll(self, wrist_name: str, roll: float) -> 'RecordedTrajectory':
        self.validate()
        if wrist_name not in self.joint_names or not math.isfinite(roll):
            raise CommandPathValidationError('Reverse retreat requires a finite measured wrist roll')
        index = self.joint_names.index(wrist_name)
        end_ns = self.points[-1].time_ns
        points = []
        for point in reversed(self.points):
            positions, velocities = list(point.positions), [-v for v in point.velocities]
            positions[index], velocities[index] = float(roll), 0.
            accelerations = list(point.accelerations)
            if accelerations:
                accelerations[index] = 0.
            points.append(CommandPoint(end_ns-point.time_ns, tuple(positions),
                                       tuple(velocities), tuple(accelerations)))
        return RecordedTrajectory(self.joint_names, tuple(points))

    def reversed(self) -> 'RecordedTrajectory':
        """Reverse all joints, including the roll that was actually commanded."""
        self.validate()
        end_ns = self.points[-1].time_ns
        return RecordedTrajectory(self.joint_names, tuple(
            CommandPoint(end_ns-point.time_ns, point.positions,
                         tuple(-v for v in point.velocities), point.accelerations)
            for point in reversed(self.points)))

    def command_samples(self) -> list[JointPathSample]:
        self.validate()
        duration = self.points[-1].time_ns/1e9
        return [JointPathSample(point.time_ns/self.points[-1].time_ns,
                                point.positions, tuple(v*duration for v in point.velocities))
                for point in self.points]

    def sample_at(self, time_ns: int) -> CommandPoint:
        if not 0 <= time_ns <= self.points[-1].time_ns:
            raise CommandPathValidationError('Sample time is outside the recorded trajectory')
        index = bisect_right(self._times, time_ns)-1
        first = self.points[index]
        if time_ns == first.time_ns:
            return first
        last = self.points[index+1]
        dt, coefficients = self._segments[index]
        u = (time_ns-first.time_ns)/(last.time_ns-first.time_ns)
        rates = tuple(_differentiate(poly) for poly in coefficients)
        accelerations = (tuple(_evaluate(_differentiate(rate), u)/dt**2 for rate in rates)
                         if first.accelerations and last.accelerations else ())
        return CommandPoint(time_ns, tuple(_evaluate(poly, u) for poly in coefficients),
                            tuple(_evaluate(rate, u)/dt for rate in rates), accelerations)


@dataclass
class ArmCommandRecording:
    arm: str
    trajectories: list[RecordedTrajectory] = field(default_factory=list)
    interrupted: bool = False


def validate_command_history(paths: Sequence[RecordedTrajectory], names: tuple[str, ...]) -> None:
    if not paths:
        raise CommandPathValidationError('Reverse return has no completed command history')
    for path in paths:
        path.validate()
        if path.joint_names != names:
            raise CommandPathValidationError('Reverse return history belongs to another arm')
        if any(abs(v) > 1e-9 for p in (path.points[0], path.points[-1]) for v in p.velocities):
            raise CommandPathValidationError('Reverse return history must have stopped action boundaries')
    for first, second in zip(paths, paths[1:]):
        if any(abs(a-b) > 1e-9 for a, b in zip(
                first.points[-1].positions, second.points[0].positions, strict=True)):
            raise CommandPathValidationError('Reverse return command history is discontinuous')


def reversed_command_history(paths: Sequence[RecordedTrajectory]) -> RecordedTrajectory:
    if not paths:
        raise CommandPathValidationError('Reverse return has no completed command history')
    validate_command_history(paths, paths[0].joint_names)
    points, elapsed = [], 0
    quintic_segments = []
    for path in reversed(paths):
        backward = path.reversed()
        if points and backward.points[0].accelerations:
            previous = points[-1]
            acceleration = backward.points[0].accelerations
            if previous.accelerations and previous.accelerations != acceleration:
                raise CommandPathValidationError('Reverse history has unequal knot accelerations')
            points[-1] = CommandPoint(previous.time_ns, previous.positions,
                                      previous.velocities, acceleration)
        points.extend(CommandPoint(elapsed+p.time_ns, p.positions, p.velocities, p.accelerations)
                      for p in (backward.points if not points else backward.points[1:]))
        quintic_segments.extend(bool(a.accelerations and b.accelerations)
                                for a, b in zip(backward.points, backward.points[1:]))
        elapsed += backward.points[-1].time_ns
    if quintic_segments != [bool(a.accelerations and b.accelerations)
                            for a, b in zip(points, points[1:])]:
        raise CommandPathValidationError('Reverse history would change spline interpolation')
    return RecordedTrajectory(paths[0].joint_names, tuple(points))


def recorded_window(path: RecordedTrajectory, start_ns: int, end_ns: int) -> RecordedTrajectory:
    path.validate()
    if not 0 <= start_ns < end_ns <= path.points[-1].time_ns:
        raise CommandPathValidationError('Invalid recorded return window')
    points = (path.sample_at(start_ns),
              *(p for p in path.points if start_ns < p.time_ns < end_ns), path.sample_at(end_ns))
    return RecordedTrajectory(path.joint_names, tuple(
        CommandPoint(p.time_ns-start_ns, p.positions, p.velocities, p.accelerations) for p in points))


def _cubic_slope_bound(a: float, b: float, c: float) -> float:
    """Bound absolute dq/du on [0, 1] using the quadratic's endpoints and vertex."""
    bound = max(abs(c), abs(3*a+2*b+c))
    if a:
        vertex = -b/(3*a)
        if 0 < vertex < 1:
            bound = max(bound, abs((3*a*vertex+2*b)*vertex+c))
    return math.nextafter(bound, math.inf)


def _differentiate(coefficients):
    return tuple(index*value for index, value in enumerate(coefficients) if index)


def _evaluate(coefficients, fraction):
    result = 0.
    for coefficient in reversed(coefficients):
        result = result*fraction+coefficient
    return result


def _interior_roots(coefficients):
    """Find stationary points in normalized segment time."""
    coefficients = list(coefficients)
    while coefficients and coefficients[-1] == 0.:
        coefficients.pop()
    if len(coefficients) < 2:
        return ()
    if len(coefficients) == 2:
        roots = (-coefficients[0]/coefficients[1],)
    elif len(coefficients) == 3:
        c, b, a = coefficients
        discriminant = b*b-4*a*c
        if discriminant < 0.:
            return ()
        root = math.sqrt(discriminant)
        roots = ((-b-root)/(2*a), (-b+root)/(2*a))
    else:
        roots = polyroots(coefficients)
    return tuple(float(value.real) for value in roots
                 if abs(value.imag) <= 1e-9 and 0. < value.real < 1.)


def _segment_motion_bounds(coefficients):
    """Bound absolute normalized velocity and acceleration at spline extrema."""
    if len(coefficients) == 4:
        _, c, b, a = coefficients
        return _cubic_slope_bound(a, b, c), max(abs(2*b), abs(6*a+2*b))
    velocity = _differentiate(coefficients)
    acceleration = _differentiate(velocity)
    speed_bound = max(abs(_evaluate(velocity, u))
                      for u in (0., 1., *_interior_roots(acceleration)))
    acceleration_bound = max(abs(_evaluate(acceleration, u))
                             for u in (0., 1., *_interior_roots(_differentiate(acceleration))))
    return math.nextafter(speed_bound, math.inf), acceleration_bound


def retime_recorded_trajectory(
        path: RecordedTrajectory, limits: dict[str, PhysicalJointLimit], *,
        velocity: float, acceleration: float, minimum_duration_sec: float = .3,
        limit_scale: float = .8) -> RecordedTrajectory:
    """Uniformly retime saved splines using their analytical rate extrema.

    Positions and relative timing are retained. Velocities change inversely
    with time, and accelerations change inversely with squared time.
    """
    path.validate()
    if (not set(path.joint_names) <= limits.keys()
            or any(isinstance(value, bool) or not math.isfinite(value) or value <= 0
                   for value in (velocity, acceleration, minimum_duration_sec))):
        raise CommandPathValidationError('Invalid recorded timing limits')
    velocity_limits, acceleration_limits = joint_rate_limits(
        path.joint_names, limits, velocity=velocity, acceleration=acceleration,
        limit_scale=limit_scale)
    required_scale = 0.
    for dt, coefficients in path._segments:
        for poly, vmax, amax in zip(
                coefficients, velocity_limits, acceleration_limits, strict=True):
            v_bound, a_bound = _segment_motion_bounds(poly)
            required_scale = max(required_scale, v_bound/dt/vmax, math.sqrt(a_bound/amax)/dt)
    duration = path.points[-1].time_ns/1e9
    scale = max(minimum_duration_sec/duration, required_scale*1.05)
    result = RecordedTrajectory(path.joint_names, tuple(
        CommandPoint(round(point.time_ns*scale), point.positions,
                     tuple(value/scale for value in point.velocities),
                     tuple(value/scale**2 for value in point.accelerations))
        for point in path.points))
    result.validate()
    return result


def descending_height_time(path: RecordedTrajectory, height: Callable[[tuple[float, ...]], float],
                           target_z: float, *, step_rad: float = .01,
                           maximum_points: int = 2000, height_tolerance_m: float = 0.) -> int:
    """Find a point above the first downward crossing on the saved cubics.

    A positive tolerance may finish bisection once its upper-height endpoint
    is within that many meters of the target. Zero retains nanosecond search.
    Analytical spline slopes bound joint step sizes.
    This is a joint sampling bound, not a continuous Cartesian collision proof.
    """
    if (not math.isfinite(target_z) or not math.isfinite(step_rad) or step_rad <= 0
            or not math.isfinite(height_tolerance_m) or height_tolerance_m < 0
            or type(maximum_points) is not int or maximum_points < 2):
        raise CommandPathValidationError('Invalid reverse return height search')
    path.validate()
    def z(point):
        value = height(point.positions)
        if not math.isfinite(value):
            raise CommandPathValidationError('Reverse return FK height is not finite')
        return value
    previous_z = z(path.points[0])
    if previous_z <= target_z:
        raise CommandPathValidationError('Reverse return must start above the release height')
    examined = 1
    for index, (first, last) in enumerate(zip(path.points, path.points[1:])):
        _, coefficients = path._segments[index]
        bound = max(_segment_motion_bounds(poly)[0] for poly in coefficients)
        count = max(2, math.ceil(bound/step_rad))
        examined += count
        if examined > maximum_points:
            raise CommandPathValidationError('Reverse return exceeds its sample budget')
        previous = first.time_ns
        for i in range(1,count+1):
            current = first.time_ns+round((last.time_ns-first.time_ns)*i/count)
            current_z = z(path.sample_at(current))
            if current_z <= target_z:
                low, high = previous, current
                low_z = previous_z
                for _ in range(24):
                    if high-low <= 1 or (low > 0 and 0 < low_z-target_z <= height_tolerance_m):
                        break
                    middle = (low+high)//2
                    middle_z = z(path.sample_at(middle))
                    if middle_z > target_z:
                        low = middle
                        low_z = middle_z
                    else:
                        high = middle
                if low == 0:
                    raise CommandPathValidationError('Reverse height search has no positive travel')
                return low
            previous = current
            previous_z = current_z
    raise CommandPathValidationError('Recorded return path does not reach the release height')


def stopped_recorded_window(path: RecordedTrajectory, start_ns: int, end_ns: int,
                            limits: dict[str, PhysicalJointLimit], *, velocity: float,
                            acceleration: float, limit_scale: float = .8) -> RecordedTrajectory:
    """Keep source timing and velocities, easing the final partial stop only.

    A continuation starts at rest, so that short segment uses a smooth start
    and stop. All positions lie on the saved source cubics. The resulting JTC
    Hermite splines are slowed only as much as current rate limits require.
    """
    source = recorded_window(path, start_ns, end_ns)
    names = source.joint_names
    if not set(names) <= limits.keys() or any(not math.isfinite(x) or x <= 0
                                             for x in (velocity, acceleration)):
        raise CommandPathValidationError('Invalid replay timing limits')
    v_limits, a_limits = joint_rate_limits(
        names, limits, velocity=velocity, acceleration=acceleration, limit_scale=limit_scale)
    duration = source.points[-1].time_ns/1e9
    start_stopped = all(abs(v) < 1e-9 for v in source.points[0].velocities)
    end_stopped = all(abs(v) < 1e-9 for v in source.points[-1].velocities)
    if start_stopped and end_stopped and any(p.accelerations for p in source.points):
        return source
    brake = (0. if end_stopped else min(duration, max(
        abs(v)/(.5*a) for v,a in zip(source.points[-1].velocities, a_limits, strict=True))))
    if start_stopped:
        onset, output_duration = duration-brake, duration+brake
        def source_time(t):
            if brake == 0. or t <= onset:
                return min(t, duration), 1.
            u = min(1., (t-onset)/(2*brake))
            return onset+2*brake*(u-u**3+.5*u**4), 1-3*u*u+2*u**3
    else:
        # A short feedback continuation is bounded to the same stored path.
        onset, output_duration = 0., duration
        def source_time(t):
            u = min(1., t/output_duration)
            return duration*(10*u**3-15*u**4+6*u**5), 30*u*u*(1-u)**2
    times = {0, round(output_duration*1e9)}
    for point in source.points[1:-1]:
        wanted = point.time_ns/1e9
        if start_stopped and wanted <= onset:
            times.add(point.time_ns)
            continue
        low, high = onset, output_duration
        for _ in range(48):
            middle = (low+high)/2
            if source_time(middle)[0] < wanted:
                low = middle
            else:
                high = middle
        times.add(round((low+high)*.5e9))
    # Resolve the smooth time warp as well as all original spline knots.
    if not (start_stopped and end_stopped):
        times.update(round((onset+(output_duration-onset)*i/32)*1e9) for i in range(33))
    # Inverted source knots can coincide with a warp sample to within a few
    # nanoseconds. Merge those numerically redundant samples: differentiating
    # quantized positions across such an interval invents large accelerations.
    ordered_times = sorted(times)
    stable_times = [ordered_times[0]]
    for t_ns in ordered_times[1:-1]:
        if t_ns-stable_times[-1] >= 100_000 and ordered_times[-1]-t_ns >= 100_000:
            stable_times.append(t_ns)
    stable_times.append(ordered_times[-1])
    points = []
    for t_ns in stable_times:
        s, rate = source_time(t_ns/1e9)
        p = source.sample_at(max(0,min(source.points[-1].time_ns, round(s*1e9))))
        stopped = t_ns in (0, round(output_duration*1e9))
        positions = source.points[0].positions if t_ns == 0 else (
            source.points[-1].positions if stopped else p.positions)
        points.append(CommandPoint(t_ns, positions,
                                   (0.,)*len(names) if stopped else tuple(v*rate for v in p.velocities)))
    # Bound each emitted cubic analytically; no equal-time resampling of the
    # dense position list, which previously stretched a 5s history to 18s.
    scale = 1.
    for first,last in zip(points, points[1:]):
        dt = (last.time_ns-first.time_ns)/1e9
        for q0,q1,v0,v1,vl,al in zip(first.positions,last.positions,first.velocities,last.velocities,
                                    v_limits,a_limits,strict=True):
            a,b,c = 2*q0-2*q1+dt*(v0+v1), 3*q1-3*q0-dt*(2*v0+v1), dt*v0
            vmax = max(abs(c),abs(3*a+2*b+c))/dt
            if abs(a) > 1e-14 and 0 < -b/(3*a) < 1:
                u = -b/(3*a)
                vmax = max(vmax,abs(3*a*u*u+2*b*u+c)/dt)
            amax = max(abs(2*b),abs(6*a+2*b))/dt**2
            scale = max(scale,vmax/vl,math.sqrt(amax/al))
    if scale > 1.:
        scale *= 1.001
        points = [CommandPoint(round(p.time_ns*scale),p.positions,tuple(v/scale for v in p.velocities))
                  for p in points]
    result = RecordedTrajectory(names,tuple(points))
    result.validate()
    return result


def validate_recorded_trajectory(
        trajectory: RecordedTrajectory, limits: dict[str, PhysicalJointLimit], *,
        velocity: float, acceleration: float, step: float,
        maximum_points: int = 2000, limit_scale: float = .8) -> list[JointPathSample]:
    """Check Hermite extrema and rates, then sample the splines for collisions.

    The returned collision samples do not change the recorded command points.
    ROS JTC selects a quintic when both knots include accelerations.
    """
    trajectory.validate()
    names = trajectory.joint_names
    if (not set(names) <= limits.keys()
            or not all(math.isfinite(v) and v > 0 for v in (velocity, acceleration, step))):
        raise CommandPathValidationError('Invalid recorded trajectory limits')
    end_ns = trajectory.points[-1].time_ns
    velocity_limits, acceleration_limits = joint_rate_limits(
        names, limits, velocity=velocity, acceleration=acceleration, limit_scale=limit_scale)
    duration = end_ns/1e9
    result = []
    for index, (dt, coefficients) in enumerate(trajectory._segments):
        first, last = trajectory.points[index:index+2]
        fractions, maximum = {0., 1.}, 0.
        rates = tuple(_differentiate(poly) for poly in coefficients)
        for name, poly, rate, velocity_limit, acceleration_limit in zip(
                names, coefficients, rates, velocity_limits, acceleration_limits, strict=True):
            slope_bound, acceleration_bound = _segment_motion_bounds(poly)
            fractions.update(_interior_roots(rate))
            if (slope_bound/dt > velocity_limit+1e-7
                    or acceleration_bound/dt**2 > acceleration_limit+1e-7):
                raise CommandPathValidationError(f'Recorded trajectory exceeds joint rate limits: joint={name}')
            maximum = max(maximum, slope_bound)
        count = max(2, math.ceil(maximum/step))
        if count+len(result)+1 > maximum_points:
            raise CommandPathValidationError('Recorded trajectory exceeds collision sample budget')
        fractions.update(i/count for i in range(count+1))
        for u in sorted(fractions):
            if result and u == 0.:
                continue
            positions = (first.positions if u == 0. else last.positions if u == 1.
                         else tuple(_evaluate(poly, u) for poly in coefficients))
            for name, q in zip(names, positions, strict=True):
                try:
                    limits[name].check(q)
                except JointRangeValidationError as error:
                    raise error.at(name, len(result)) from error
            progress = (first.time_ns+u*(last.time_ns-first.time_ns))/end_ns
            derivatives = tuple(_evaluate(rate, u)*duration/dt for rate in rates)
            result.append(JointPathSample(progress, positions, derivatives))
            if len(result) > maximum_points:
                raise CommandPathValidationError('Recorded trajectory exceeds collision sample budget')
    return result
