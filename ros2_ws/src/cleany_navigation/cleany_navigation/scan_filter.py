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
    """Keep +/-90deg around LiDAR +X; NaN removes rear returns without clearing rays."""
    result = mask_close_returns(ranges, minimum)
    for i in range(len(result)):
        angle = angle_min + i * angle_increment - center
        relative = math.atan2(math.sin(angle), math.cos(angle))
        if abs(relative) > half_angle + 1e-6:
            result[i] = float("nan")
    return result
