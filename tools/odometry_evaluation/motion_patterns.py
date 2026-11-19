"""편도 평가 패턴의 순수 속도 벡터입니다."""

import math

PATTERN_DIRECTIONS = {
    "surge": (1.0, 0.0),
    "sway": (0.0, 1.0),
    "diagonal_left": (1.0 / math.sqrt(2.0), 1.0 / math.sqrt(2.0)),
    "diagonal_right": (1.0 / math.sqrt(2.0), -1.0 / math.sqrt(2.0)),
}


def motion_vectors(pattern: str, speed: float) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """나가는 방향과 정확히 반대인 복귀 방향의 (vx, vy, wz) 명령을 반환합니다."""
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
