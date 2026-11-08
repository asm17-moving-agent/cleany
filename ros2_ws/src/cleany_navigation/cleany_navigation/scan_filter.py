import math
import struct


def mask_close_returns(ranges, minimum=0.30):
    # Ranges are float32 on the ROS wire, so include the rounded 30cm boundary.
    threshold = struct.unpack("f", struct.pack("f", minimum))[0]
    return [
        float("nan") if math.isfinite(v) and v <= threshold else float(v)
        for v in ranges
    ]


def mask_front_scan(
    ranges, angle_min, angle_increment, minimum=0.30, center=0.0, half_angle=math.pi / 2
):
    """Keep the configured angular sector; NaN masks excluded and close returns."""
    result = mask_close_returns(ranges, minimum)
    for i in range(len(result)):
        angle = angle_min + i * angle_increment - center
        relative = math.atan2(math.sin(angle), math.cos(angle))
        if abs(relative) > half_angle + 1e-6:
            result[i] = float("nan")
    return result


def mask_self_returns(ranges, angle_min, angle_increment, *, translation, yaw, boxes):
    """Mask confirmed robot returns in base-frame rectangles, preserving other rays."""
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
