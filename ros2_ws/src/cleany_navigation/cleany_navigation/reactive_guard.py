"""직접 관측 기반 평면 제동 외곽이며 voxel 지도나 pan 정렬을 사용하지 않습니다."""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import numpy as np

from cleany_navigation.voxel_guard import GuardConfig, twist_pose


@dataclass
class InputWatchdog:
    """소스 stamp와 수신 경과 시간을 각각 ROS 시계와 단조 시계로 검사합니다.

    모든 시간 단위는 초입니다. ROS 시각이 10ms보다 크게 역행하면 이 인스턴스에
    fault를 유지하며 멈춘 ROS 시각은 단조 경과 시간으로 검출합니다.
    """
    source_timeout_s: float = .5
    command_timeout_s: float = .5
    last_ros: float | None = None
    advanced_wall: float = -math.inf
    fault: str = ''
    sources: dict[str, tuple[float, float]] = field(default_factory=dict)

    def tick(self, now: float, wall: float) -> None:
        if not math.isfinite(now) or not math.isfinite(wall):
            raise ValueError('INVALID_CLOCK')
        if self.last_ros is not None and now < self.last_ros-.01:
            self.fault = 'CLOCK_RESET_RESTART_REQUIRED'
        if self.last_ros != now:
            self.advanced_wall = wall
        self.last_ros = now
        if self.fault:
            raise ValueError(self.fault)
        if wall-self.advanced_wall > self.command_timeout_s:
            raise ValueError('CLOCK_TIMEOUT')

    def check(self, now: float, wall: float, command_wall: float,
              raw_wall: float, required: tuple[str, ...]) -> float:
        self.tick(now, wall)
        if any(not 0 <= wall-t <= self.command_timeout_s for t in (command_wall, raw_wall)):
            raise ValueError('COMMAND_TIMEOUT')
        ages = []
        for name in required:
            stamp, received = self.sources.get(name, (-math.inf, -math.inf))
            if (not 0 <= now-stamp <= self.source_timeout_s
                    or not 0 <= wall-received <= self.source_timeout_s):
                raise ValueError('STALE_'+name.upper())
            ages.append(now-stamp)
        return max(ages, default=0.)


class PlanarBrakingGuard:
    """기존 제동 한계를 유지하며 주차된 전체 body를 XY에서 둘러쌉니다.

    반응 및 제동 시간 동안 전체 속도를 유지하면 비례 제동 거리보다 크게 잡습니다.
    표본 여유는 sweep 자세 사이의 움직임도 덮습니다. 관측 표면을 검사하며
    미관측 공간이나 움직이는 장애물을 검증하지 않습니다.
    """
    def __init__(self, low: np.ndarray, high: np.ndarray, config: GuardConfig):
        self.low, self.high = np.asarray(low, dtype=float), np.asarray(high, dtype=float)
        if (self.low.shape != (2,) or self.high.shape != (2,)
                or not np.isfinite([self.low, self.high]).all()
                or (self.high <= self.low).any()):
            raise ValueError('Invalid body footprint bounds')
        self.config = config
        self.radius = float(np.linalg.norm(np.maximum(np.abs(self.low), np.abs(self.high))))

    def hits(self, points: np.ndarray, velocity: np.ndarray, age: float, extra: float = 0.) -> bool:
        c = self.config
        speed = float(np.linalg.norm(velocity[:2]))
        horizon = c.reaction_s+age+max(speed/c.linear_deceleration,
                                      abs(velocity[2])/c.angular_deceleration)+extra
        if not math.isfinite(horizon) or not 0 <= horizon <= 5:
            raise ValueError('Unbounded braking horizon')
        padding = c.margin+(speed+abs(velocity[2])*self.radius)*c.step_s/2
        radius = self.radius+padding+speed*horizon
        nearby = points[(np.abs(points) <= radius).all(axis=1)]
        if not len(nearby):
            return False
        times = [0.] if not np.any(velocity) else np.linspace(0, horizon, int(np.ceil(horizon/c.step_s))+1)
        for seconds in times:
            pose = twist_pose(velocity, seconds)
            moved = (nearby-pose[:2, 3])@pose[:2, :2]
            if ((moved >= self.low-padding) & (moved <= self.high+padding)).all(axis=1).any():
                return True
        return False

    def evaluate(self, points: np.ndarray, command: np.ndarray, measured: np.ndarray,
                 source_age: float) -> tuple[np.ndarray, str]:
        """요청 twist를 제한하고 명령 및 측정 제동 sweep을 검사합니다.

        Args:
            points: 현재 베이스 좌표계의 관측 표면점 Nx2이며 미터 단위입니다.
            command: body 좌표계의 요청 vx, vy(m/s)와 yaw 속도(rad/s).
            measured: 같은 좌표계와 단위의 측정 twist이며 명령 제한을 적용하지 않습니다.
            source_age: 반응 시간에 더하는 관측 경과 시간(초).

        Returns:
            제한, 감속 또는 0으로 설정한 twist와 대응 판정 코드입니다. 빈 관측도
            통과할 수 있으므로 호출자는 이동 허가로 해석하기 전에 센서 최신성과
            관측 범위를 확인해야 합니다.

        Raises:
            ValueError: 입력이 무효하거나 제동 예측 구간이 무제한인 경우.
        """
        points, command, measured = [np.asarray(a, dtype=float) for a in (points, command, measured)]
        if (points.ndim != 2 or points.shape[1] != 2 or command.shape != (3,)
                or measured.shape != (3,) or not math.isfinite(source_age) or source_age < 0
                or any(not np.isfinite(a).all() for a in (points, command, measured))):
            raise ValueError('Invalid collision input')
        c = self.config
        # 충돌 검사 전에 serial guard와 동일한 명령 제한을 적용합니다.
        # 초과 속도를 포함한 실제 측정 속도는 sweep에 그대로 반영합니다.
        command = command.copy()
        command[:2] *= min(1., c.max_speed/max(np.linalg.norm(command[:2]), 1e-12))
        command[2] = np.clip(command[2], -c.max_yaw_rate, c.max_yaw_rate)
        if self.hits(points, measured, source_age) or self.hits(points, command, source_age):
            return np.zeros(3), 'STOP_OBSTACLE'
        if self.hits(points, command, source_age, c.slowdown_lookahead_s):
            return command*c.slowdown_ratio, 'SLOW_OBSTACLE'
        return command.copy(), 'PASS'
