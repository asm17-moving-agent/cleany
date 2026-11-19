"""메카넘 명령 검증과 제한, encoder 피드백 변환을 처리합니다."""
from __future__ import annotations

from dataclasses import dataclass
from math import isfinite

UINT32 = 0xFFFFFFFF


def newer(new: int, old: int) -> bool:
    """uint32 래핑을 고려해 엄격한 반범위 규칙으로 sequence를 비교합니다."""
    delta = (new - old) & UINT32
    return 0 < delta < 0x80000000


def finite_twist_axes(*axes: float) -> bool:
    return len(axes) == 6 and all(isfinite(value) for value in axes)


@dataclass(frozen=True)
class Geometry:
    wheel_radius: float
    wheelbase: float
    separation: float

    def __post_init__(self) -> None:
        if not all(isfinite(x) and x > 0 for x in
                   (self.wheel_radius, self.wheelbase, self.separation)):
            raise ValueError('geometry dimensions must be positive and finite')


@dataclass(frozen=True)
class Limits:
    linear_x: float
    linear_y: float
    angular_z: float
    wheel_rad_s: float
    timeout_s: float

    def __post_init__(self) -> None:
        if not all(isfinite(x) and x > 0 for x in
                   (self.linear_x, self.linear_y, self.angular_z,
                    self.wheel_rad_s, self.timeout_s)):
            raise ValueError('limits and timeout must be positive and finite')
        if self.wheel_rad_s > 10.0:
            raise ValueError('wheel speed limit exceeds MCU contract (10 rad/s)')


def wheel_speeds(x: float, y: float, yaw: float, geometry: Geometry,
                 limits: Limits) -> tuple[float, float, float, float]:
    """차체 좌표계의 제한된 속도를 휠 각속도로 바꿉니다.

    차체 축별 속도를 먼저 제한하고 휠 한계를 넘으면 네 휠을 같은 비율로 줄입니다.
    유한하지 않은 입력 축이 있으면 네 목표를 모두 0으로 반환합니다.

    Args:
        x: 전방 속도(m/s).
        y: 좌측 속도(m/s).
        yaw: 반시계 방향 각속도(rad/s).
        geometry: 휠 반지름, 전후 거리와 좌우 간격(m).

    Returns:
        FL, FR, RL, RR 순서의 목표 휠 각속도(rad/s).
    """
    axes = (x, y, yaw)
    if not all(isfinite(v) for v in axes):
        return (0.,) * 4
    x = max(-limits.linear_x, min(limits.linear_x, x))
    y = max(-limits.linear_y, min(limits.linear_y, y))
    yaw = max(-limits.angular_z, min(limits.angular_z, yaw))
    rot = (geometry.wheelbase + geometry.separation) * yaw / 2
    raw = ((x-y-rot)/geometry.wheel_radius, (x+y+rot)/geometry.wheel_radius,
           (x+y-rot)/geometry.wheel_radius, (x-y+rot)/geometry.wheel_radius)
    peak = max(map(abs, raw))
    scale = min(1., limits.wheel_rad_s / peak) if peak else 1.
    return tuple(v*scale for v in raw)


class EncoderAdapter:
    """부호 있는 int32 count 표본을 연속적인 휠 회전량(rad)으로 바꿉니다."""
    def __init__(self, counts_per_revolution: int = 3172) -> None:
        if counts_per_revolution <= 0:
            raise ValueError('counts per revolution must be positive')
        self.scale = counts_per_revolution
        self.boot: int | None = None
        self.previous: tuple[int, ...] | None = None
        self.totals = [0, 0, 0, 0]

    def rebase(self) -> None:
        """누적 휠 회전량은 유지하고 다음 표본의 변화량만 버립니다."""
        self.previous = None

    def update(self, boot: int, counts: tuple[int, ...]) -> tuple[float, ...]:
        """MCU가 재부팅해도 누적 회전량을 유지하며 encoder count 래핑을 풉니다.

        부팅 식별자가 바뀌거나 rebase한 뒤 첫 표본은 새 기준점이 됩니다.
        같은 부팅 세션의 연속 표본 사이 실제 변화량은 int32 반범위 안에 있어야 합니다.

        Args:
            boot: encoder 초기화를 구분하는 MCU 부팅 식별자.
            counts: FL, FR, RL, RR 순서의 부호 있는 int32 count 네 개.

        Returns:
            같은 휠 순서의 누적 회전량(rad).

        Raises:
            ValueError: 입력이 부호 있는 int32 값 네 개가 아닌 경우.
        """
        if len(counts) != 4 or any(not -0x80000000 <= c <= 0x7fffffff for c in counts):
            raise ValueError('expected four signed int32 encoder counts')
        if self.boot != boot:
            self.boot = boot
        elif self.previous is not None:
            for i, (cur, prev) in enumerate(zip(counts, self.previous)):
                delta = ((cur - prev + 0x80000000) & UINT32) - 0x80000000
                self.totals[i] += delta
        self.previous = counts
        return tuple(v * 6.283185307179586 / self.scale for v in self.totals)
