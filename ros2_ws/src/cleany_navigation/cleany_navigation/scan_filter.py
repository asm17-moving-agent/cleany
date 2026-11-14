import math
import struct


def mask_close_returns(ranges, minimum=0.30):
    # ROS 전송의 거리는 float32이므로 반올림된 30cm 경계도 포함합니다.
    threshold = struct.unpack("f", struct.pack("f", minimum))[0]
    return [
        float("nan") if math.isfinite(v) and v <= threshold else float(v)
        for v in ranges
    ]


def mask_front_scan(
    ranges, angle_min, angle_increment, minimum=0.30, center=0.0, half_angle=math.pi / 2
):
    """설정된 각도 구간을 유지하며 제외 구간과 가까운 반사값은 NaN으로 가립니다."""
    result = mask_close_returns(ranges, minimum)
    for i in range(len(result)):
        angle = angle_min + i * angle_increment - center
        relative = math.atan2(math.sin(angle), math.cos(angle))
        if abs(relative) > half_angle + 1e-6:
            result[i] = float("nan")
    return result


def mask_self_returns(ranges, angle_min, angle_increment, *, translation, yaw, boxes):
    """베이스 좌표계 사각형 안의 확인된 로봇 반사값만 가리고 다른 ray는 보존합니다."""
    result = list(ranges)
    for i, distance in enumerate(result):
        if not math.isfinite(distance):
            continue
        angle = angle_min + i * angle_increment + yaw
        x = translation[0] + distance * math.cos(angle)
        y = translation[1] + distance * math.sin(angle)
        if any(x0 <= x <= x1 and y0 <= y <= y1 for x0, y0, x1, y1 in boxes):
            result[i] = float('nan')
    return result
