"""Explicit transport of the sampled cubic path; no implicit execution."""
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
