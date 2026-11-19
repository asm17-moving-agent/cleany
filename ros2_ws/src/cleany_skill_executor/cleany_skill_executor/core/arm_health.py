"""Arm execution-layer availability for the System Supervisor; no ROS dependency."""
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
    """Report whether the arm controllers can execute, not whether a motion is safe.

    Permission to drive with the arm in a given state stays with the manipulation
    outcome (`safe_to_drive`) that the Mission Runtime checks before every drive.
    A joint never observed is UNKNOWN; one that stopped reporting is UNAVAILABLE.
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
