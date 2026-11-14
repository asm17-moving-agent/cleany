"""System Supervisor용 팔 실행 계층 가용성이며 ROS 의존성이 없습니다."""
from dataclasses import dataclass
from math import isfinite

AVAILABLE, UNAVAILABLE, UNKNOWN = 'AVAILABLE', 'UNAVAILABLE', 'UNKNOWN'


@dataclass(frozen=True)
class ArmHealthConfig:
    controllers: tuple[str, ...]
    joints: tuple[str, ...]
    joint_timeout_s: float
    controller_timeout_s: float


class ArmHealth:
    """팔 제어기의 실행 가능 여부를 보고하며 이동의 안전성을 판단하지 않습니다.

    특정 팔 상태에서의 베이스 이동 허가는 Mission Runtime이 이동마다 확인하는
    조작 결과 safe_to_drive가 담당합니다. 한 번도 관측하지 않은 관절은 UNKNOWN,
    보고가 끊긴 관절은 UNAVAILABLE입니다.
    """

    def __init__(self, config: ArmHealthConfig):
        if not config.controllers or not config.joints:
            raise ValueError('arm controllers and joints must be configured')
        self.config = config
        self.controllers: tuple[dict, float] | None = None
        self.samples: dict[str, tuple[float, float]] = {}

    def controller_states(self, states: dict[str, str], now: float) -> None:
        self.controllers = (dict(states), now)

    def joint_state(self, names, positions, now: float) -> None:
        for index, name in enumerate(names):
            if name in self.config.joints:
                self.samples[name] = (positions[index] if index < len(positions) else float('nan'), now)

    def status(self, now: float) -> tuple[str, str]:
        if self.controllers is None or now - self.controllers[1] > self.config.controller_timeout_s:
            return UNKNOWN, 'controller manager state unavailable'
        inactive = [name for name in self.config.controllers if self.controllers[0].get(name) != 'active']
        if inactive:
            return UNAVAILABLE, 'controllers not active: ' + ', '.join(inactive)
        unseen = [name for name in self.config.joints if name not in self.samples]
        if unseen:
            return UNKNOWN, 'no joint feedback received yet: ' + ', '.join(unseen)
        stale = [name for name in self.config.joints if now - self.samples[name][1] > self.config.joint_timeout_s]
        if stale:
            return UNAVAILABLE, 'joint feedback stale: ' + ', '.join(stale)
        invalid = [name for name in self.config.joints if not isfinite(self.samples[name][0])]
        if invalid:
            return UNAVAILABLE, 'non-finite joint position: ' + ', '.join(invalid)
        return AVAILABLE, 'arm controllers active with fresh finite joint feedback'
