"""Semantic sensor checks; receipt alone cannot prove component availability."""
from math import isfinite


def valid_scan(msg, minimum_ratio: float) -> bool:
    if not msg.header.frame_id or not msg.ranges or not (
        isfinite(msg.range_min) and isfinite(msg.range_max) and 0 <= msg.range_min < msg.range_max
        and isfinite(msg.angle_increment) and msg.angle_increment > 0
    ):
        return False
    # +inf is a legitimate no-return beam; NaN, -inf and out-of-range values are not.
    usable = sum((isfinite(value) and msg.range_min <= value <= msg.range_max) or
                 value == float('inf') for value in msg.ranges)
    return usable / len(msg.ranges) >= minimum_ratio


def valid_pose(msg, maximum_variance: float) -> bool:
    pose, covariance = msg.pose.pose, msg.pose.covariance
    values = [pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    norm = sum(value * value for value in values[3:])
    return (bool(msg.header.frame_id) and all(isfinite(value) for value in values) and
            abs(norm - 1.) < .05 and all(isfinite(value) for value in covariance) and
            all(0 <= covariance[index] <= maximum_variance for index in (0, 7, 35)))


def valid_image(msg) -> bool:
    return (bool(msg.header.frame_id and msg.encoding) and msg.width > 0 and msg.height > 0 and
            msg.step > 0 and len(msg.data) == msg.step * msg.height)


class StampGuard:
    def __init__(self, maximum_age_s):
        self.maximum_age_s = maximum_age_s
        self.last = {}
        self.last_now = None

    def accept(self, source, stamp, now_ns):
        if self.last_now is not None and now_ns < self.last_now:
            self.last.clear()  # Clock resets invalidate previous sensor evidence.
        self.last_now = now_ns
        value = stamp.sec * 10**9 + stamp.nanosec
        valid = (value > 0 and value > self.last.get(source, 0) and
                 0 <= now_ns - value <= self.maximum_age_s * 1e9)
        if valid:
            self.last[source] = value
        return valid
