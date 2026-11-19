"""의미상 센서 검사이며 수신만으로 컴포넌트 가용성을 입증하지 않습니다."""
from math import isfinite

# AMCL 공분산 뺄셈에서 작은 음의 부동소수점 잔차가 남을 수 있습니다.
COVARIANCE_ROUNDOFF_TOLERANCE = 1e-9


def valid_scan(msg, minimum_ratio: float) -> bool:
    if not msg.header.frame_id or not msg.ranges or not (
        isfinite(msg.range_min) and isfinite(msg.range_max) and 0 <= msg.range_min < msg.range_max
        and isfinite(msg.angle_increment) and msg.angle_increment > 0
    ):
        return False
    # +inf는 유효한 무반사 beam이지만 NaN, -inf와 범위 밖 값은 아닙니다.
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
            all(-COVARIANCE_ROUNDOFF_TOLERANCE <= covariance[index] <= maximum_variance
                for index in (0, 7, 35)))


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
            self.last.clear()  # 시계 reset은 이전 센서 근거를 무효화합니다.
        self.last_now = now_ns
        value = stamp.sec * 10**9 + stamp.nanosec
        valid = (value > 0 and value > self.last.get(source, 0) and
                 0 <= now_ns - value <= self.maximum_age_s * 1e9)
        if valid:
            self.last[source] = value
        return valid
