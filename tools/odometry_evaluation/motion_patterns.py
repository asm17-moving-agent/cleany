"""Pure velocity vectors for one-way evaluation patterns."""

import math

PATTERN_DIRECTIONS = {
    "surge": (1.0, 0.0),
    "sway": (0.0, 1.0),
    "diagonal_left": (1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0)),
    "diagonal_right": (1.0 / math.sqrt(2.0), -1.0 / math.sqrt(2.0)),
}


def motion_vectors(pattern: str, speed: float) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return outbound and exact-opposite inbound (vx, vy, wz) commands."""
    if pattern == "rotation":
        if not math.isfinite(speed) or speed <= 0:
            raise ValueError("speed must be finite and positive")
        return (0.0, 0.0, speed), (0.0, 0.0, -speed)
    if pattern not in PATTERN_DIRECTIONS:
        raise ValueError(f"Unknown pattern: {pattern}")
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("speed must be finite and positive")
    x, y = PATTERN_DIRECTIONS[pattern]
    outbound = (x * speed, y * speed, 0.0)
    return outbound, tuple(-v for v in outbound)
