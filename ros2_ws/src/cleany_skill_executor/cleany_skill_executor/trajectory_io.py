"""Explicit transport of validated cubic paths; no implicit execution."""
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def path_to_message(path):
    message = JointTrajectory()
    if path is None:
        return message
    message.joint_names = list(path.joint_names)
    for index, values in enumerate(path.points):
        point = JointTrajectoryPoint(positions=list(values))
        if path.times:
            nanoseconds = round(path.times[index]*1e9)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(nanoseconds, 1_000_000_000)
            point.velocities = list(path.velocities[index])
        message.points.append(point)
    return message


def command_samples_to_message(names, samples, duration: float, *, indices=None,
                               header_ns: int = 0, delay_ns: int = 0) -> JointTrajectory:
    """Encode dense samples or exact cubic knots with an optional initial hold.

    Derivatives are with respect to normalized progress, not wall-clock time.
    Keep acceleration fields empty so JTC reconstructs Hermite cubics.
    """
    if (not math.isfinite(duration) or duration <= 0 or len(samples) < 2
            or type(delay_ns) is not int or delay_ns < 0):
        raise ValueError('Invalid cubic command timing')
    indices = tuple(range(len(names))) if indices is None else tuple(indices)
    if len(indices) != len(names):
        raise ValueError('Command joint projection has the wrong size')
    message = JointTrajectory(joint_names=list(names))
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(header_ns, 10**9)
    if delay_ns:
        message.points.append(JointTrajectoryPoint(
            positions=[samples[0].positions[i] for i in indices], velocities=[0.]*len(indices)))
    previous_ns = 0 if delay_ns else -1
    for sample in samples:
        ns = delay_ns+round(sample.progress*duration*1e9)
        if ns <= previous_ns:
            raise ValueError('Cubic command times must increase after nanosecond rounding')
        previous_ns = ns
        point = JointTrajectoryPoint(
            positions=[sample.positions[i] for i in indices],
            velocities=[sample.derivatives[i]/duration for i in indices])
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(ns, 10**9)
        message.points.append(point)
    return message
