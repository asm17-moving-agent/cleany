"""Replay the actual controller cubics without IK, simplification or rebiasing."""
from dataclasses import dataclass, field
from collections.abc import Callable, Sequence
from bisect import bisect_right
import math

from .hardware_execution import CommandPathValidationError, JointRangeValidationError, PhysicalJointLimit
from .joint_path import JointPathSample


@dataclass(frozen=True)
class CommandPoint:
    time_ns: int
    positions: tuple[float, ...]
    velocities: tuple[float, ...]


@dataclass(frozen=True)
class RecordedTrajectory:
    joint_names: tuple[str, ...]
    points: tuple[CommandPoint, ...]

    def validate(self) -> None:
        width = len(self.joint_names)
        if (not width or len(set(self.joint_names)) != width or len(self.points) < 2
                or self.points[0].time_ns != 0):
            raise CommandPathValidationError('Malformed recorded arm trajectory')
        previous = -1
        for point in self.points:
            if (type(point.time_ns) is not int or point.time_ns <= previous
                    or len(point.positions) != width or len(point.velocities) != width
                    or not all(math.isfinite(q) for q in (*point.positions, *point.velocities))):
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
            points.append(CommandPoint(end_ns-point.time_ns, tuple(positions), tuple(velocities)))
        return RecordedTrajectory(self.joint_names, tuple(points))

    def reversed(self) -> 'RecordedTrajectory':
        """Reverse all joints, including the roll that was actually commanded."""
        self.validate()
        end_ns = self.points[-1].time_ns
        return RecordedTrajectory(self.joint_names, tuple(
            CommandPoint(end_ns-point.time_ns, point.positions,
                         tuple(-v for v in point.velocities)) for point in reversed(self.points)))

    def command_samples(self) -> list[JointPathSample]:
        self.validate()
        duration = self.points[-1].time_ns/1e9
        return [JointPathSample(point.time_ns/self.points[-1].time_ns,
                                point.positions, tuple(v*duration for v in point.velocities))
                for point in self.points]

    def sample_at(self, time_ns: int) -> CommandPoint:
        if not 0 <= time_ns <= self.points[-1].time_ns:
            raise CommandPathValidationError('Sample time is outside the recorded trajectory')
        index = bisect_right([p.time_ns for p in self.points], time_ns)-1
        first = self.points[index]
        if time_ns == first.time_ns:
            return first
        last = self.points[index+1]
        dt = (last.time_ns-first.time_ns)/1e9
        u = (time_ns-first.time_ns)/(last.time_ns-first.time_ns)
        coefficients = [(2*q0-2*q1+dt*(v0+v1), 3*q1-3*q0-dt*(2*v0+v1), dt*v0, q0)
                        for q0, q1, v0, v1 in zip(first.positions, last.positions,
                                                 first.velocities, last.velocities, strict=True)]
        return CommandPoint(time_ns, tuple(((a*u+b)*u+c)*u+d for a,b,c,d in coefficients),
                            tuple((3*a*u*u+2*b*u+c)/dt for a,b,c,_ in coefficients))


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
    for path in reversed(paths):
        backward = path.reversed()
        points.extend(CommandPoint(elapsed+p.time_ns, p.positions, p.velocities)
                      for p in (backward.points if not points else backward.points[1:]))
        elapsed += backward.points[-1].time_ns
    return RecordedTrajectory(paths[0].joint_names, tuple(points))


def recorded_window(path: RecordedTrajectory, start_ns: int, end_ns: int) -> RecordedTrajectory:
    path.validate()
    if not 0 <= start_ns < end_ns <= path.points[-1].time_ns:
        raise CommandPathValidationError('Invalid recorded return window')
    points = (path.sample_at(start_ns),
              *(p for p in path.points if start_ns < p.time_ns < end_ns), path.sample_at(end_ns))
    return RecordedTrajectory(path.joint_names, tuple(
        CommandPoint(p.time_ns-start_ns, p.positions, p.velocities) for p in points))


def descending_height_time(path: RecordedTrajectory, height: Callable[[tuple[float, ...]], float],
                           target_z: float, *, start_ns: int = 0, step_rad: float = .01,
                           maximum_points: int = 2000) -> int:
    """Find the first downward crossing on the original time-indexed cubics."""
    if (not math.isfinite(target_z) or not math.isfinite(step_rad) or step_rad <= 0
            or type(maximum_points) is not int or maximum_points < 2):
        raise CommandPathValidationError('Invalid reverse return height search')
    remaining = recorded_window(path, start_ns, path.points[-1].time_ns)
    def z(point):
        value = height(point.positions)
        if not math.isfinite(value):
            raise CommandPathValidationError('Reverse return FK height is not finite')
        return value
    if z(remaining.points[0]) <= target_z:
        raise CommandPathValidationError('Reverse return must start above the release height')
    examined = 1
    for first, last in zip(remaining.points, remaining.points[1:]):
        dt = (last.time_ns-first.time_ns)/1e9
        coefficients = [(2*q0-2*q1+dt*(v0+v1), 3*q1-3*q0-dt*(2*v0+v1), dt*v0)
                        for q0,q1,v0,v1 in zip(first.positions,last.positions,
                                              first.velocities,last.velocities,strict=True)]
        bound = max(3*abs(a)+2*abs(b)+abs(c) for a,b,c in coefficients)
        count = max(2, math.ceil(bound/step_rad))
        examined += count
        if examined > maximum_points:
            raise CommandPathValidationError('Reverse return exceeds its sample budget')
        previous = first.time_ns
        for i in range(1,count+1):
            current = first.time_ns+round((last.time_ns-first.time_ns)*i/count)
            if z(remaining.sample_at(current)) <= target_z:
                low, high = previous, current
                for _ in range(24):
                    if high-low <= 1:
                        break
                    middle = (low+high)//2
                    if z(remaining.sample_at(middle)) > target_z:
                        low = middle
                    else:
                        high = middle
                return start_ns+low
            previous = current
    raise CommandPathValidationError('Recorded return path does not reach the release height')


def stopped_recorded_window(path: RecordedTrajectory, start_ns: int, end_ns: int,
                            limits: dict[str, PhysicalJointLimit], *, velocity: float,
                            acceleration: float) -> RecordedTrajectory:
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
    v_limits = [min(velocity, limits[n].velocity*.8) for n in names]
    a_limits = [min(acceleration, limits[n].acceleration*.8) for n in names]
    duration = source.points[-1].time_ns/1e9
    start_stopped = all(abs(v) < 1e-9 for v in source.points[0].velocities)
    end_stopped = all(abs(v) < 1e-9 for v in source.points[-1].velocities)
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
        maximum_points: int = 2000) -> list[JointPathSample]:
    """Check exact Hermite extrema/rates and sample the cubics for collisions.

    The returned collision samples do not change the recorded command points.
    ROS JTC reconstructs cubics from the saved positions, velocities and times.
    """
    trajectory.validate()
    names = trajectory.joint_names
    if (not set(names) <= limits.keys()
            or not all(math.isfinite(v) and v > 0 for v in (velocity, acceleration, step))):
        raise CommandPathValidationError('Invalid recorded trajectory limits')
    end_ns = trajectory.points[-1].time_ns
    duration = end_ns/1e9
    result = []
    for first, last in zip(trajectory.points, trajectory.points[1:]):
        dt = (last.time_ns-first.time_ns)/1e9
        coefficients, fractions, maximum = [], {0., 1.}, 0.
        for name, q0, q1, v0, v1 in zip(
                names, first.positions, last.positions, first.velocities, last.velocities, strict=True):
            a = 2*q0-2*q1+dt*(v0+v1)
            b = 3*q1-3*q0-dt*(2*v0+v1)
            c = dt*v0
            coefficients.append((a, b, c, q0))
            slope_bound = max(abs(c), abs(3*a+2*b+c))
            if abs(a) > 1e-14:
                extremum = -b/(3*a)
                if 0 < extremum < 1:
                    slope_bound = max(slope_bound, abs((3*a*extremum+2*b)*extremum+c))
                discriminant = 4*b*b-12*a*c
                if discriminant >= 0:
                    fractions.update(u for u in (
                        (-2*b-math.sqrt(discriminant))/(6*a),
                        (-2*b+math.sqrt(discriminant))/(6*a)) if 0 < u < 1)
            elif abs(b) > 1e-14:
                extremum = -c/(2*b)
                if 0 < extremum < 1:
                    fractions.add(extremum)
            acceleration_bound = max(abs(2*b), abs(6*a+2*b))/dt**2
            if (slope_bound/dt > min(velocity, limits[name].velocity*.8)+1e-7
                    or acceleration_bound > min(acceleration, limits[name].acceleration*.8)+1e-7):
                raise CommandPathValidationError(f'Recorded trajectory exceeds joint rate limits: joint={name}')
            maximum = max(maximum, slope_bound)
        count = max(2, math.ceil(maximum/step))
        if count+len(result)+1 > maximum_points:
            raise CommandPathValidationError('Recorded trajectory exceeds collision sample budget')
        fractions.update(i/count for i in range(count+1))
        for u in sorted(fractions):
            if result and u == 0.:
                continue
            positions = tuple(((a*u+b)*u+c)*u+d for a, b, c, d in coefficients)
            for name, q in zip(names, positions, strict=True):
                try:
                    limits[name].check(q)
                except JointRangeValidationError as error:
                    raise error.at(name, len(result)) from error
            progress = (first.time_ns+u*(last.time_ns-first.time_ns))/end_ns
            derivatives = tuple((3*a*u*u+2*b*u+c)*duration/dt for a, b, c, _ in coefficients)
            result.append(JointPathSample(progress, positions, derivatives))
            if len(result) > maximum_points:
                raise CommandPathValidationError('Recorded trajectory exceeds collision sample budget')
    return result
