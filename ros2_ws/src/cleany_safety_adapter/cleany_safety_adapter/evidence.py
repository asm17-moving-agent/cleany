"""ROS-independent evaluation of independent-safety evidence. Missing evidence is unsafe."""
from dataclasses import dataclass
from math import isfinite


@dataclass
class SafetyReport:
    protection_ready: bool
    estop_engaged: bool
    physical_release_confirmed: bool
    stopped: bool
    detail: str


class SafetyEvidence:
    def __init__(self, *, timeout_s: float, linear_mps: float, angular_radps: float, joint_radps: float,
                 monitored_joints: list[str], estop_configured: bool, protection_configured: bool):
        self.timeout_s = timeout_s
        self.linear_mps, self.angular_radps, self.joint_radps = linear_mps, angular_radps, joint_radps
        self.monitored_joints = list(monitored_joints)
        self.estop_configured, self.protection_configured = estop_configured, protection_configured
        self.released: tuple[bool, float] | None = None
        self.protection: tuple[bool, float] | None = None
        self.base: tuple[bool, float] | None = None
        self.joint_samples: dict[str, tuple[bool, float]] = {}

    def estop_released(self, released: bool, now: float) -> None:
        self.released = (bool(released), now)

    def protection_ready(self, ready: bool, now: float) -> None:
        self.protection = (bool(ready), now)

    def odometry(self, vx: float, vy: float, wz: float, now: float) -> None:
        still = (all(isfinite(v) for v in (vx, vy, wz)) and abs(vx) <= self.linear_mps and
                 abs(vy) <= self.linear_mps and abs(wz) <= self.angular_radps)
        self.base = (still, now)

    def joints(self, names, velocities, now: float) -> None:
        # A joint without a velocity value gives no stop evidence.
        for index, name in enumerate(names):
            value = velocities[index] if index < len(velocities) else float('nan')
            self.joint_samples[name] = (isfinite(value) and abs(value) <= self.joint_radps, now)

    def awaiting_first_evidence(self) -> bool:
        """True until each configured hardware source has reported once.

        Publishing "unsafe" before then would make the Supervisor latch at every boot; a
        silent adapter is instead treated as missing evidence with the boot grace.
        """
        return ((self.estop_configured and self.released is None) or
                (self.protection_configured and self.protection is None))

    def _fresh(self, sample, now):
        return sample is not None and now - sample[1] <= self.timeout_s

    def evaluate(self, now: float) -> SafetyReport:
        reasons = []
        if not self.protection_configured:
            reasons.append('no independent protection evidence source configured')
        elif not self._fresh(self.protection, now):
            reasons.append('protection evidence stale')
        elif not self.protection[0]:
            reasons.append('protection layer reports not ready')
        protection = not reasons
        released = self.estop_configured and self._fresh(self.released, now) and self.released[0]
        if not self.estop_configured:
            reasons.append('no e-stop circuit evidence source configured; treated as engaged')
        elif not self._fresh(self.released, now):
            reasons.append('e-stop evidence stale; treated as engaged')
        elif not self.released[0]:
            reasons.append('e-stop engaged or not physically reset')
        joints = self.monitored_joints or list(self.joint_samples)
        motion = []
        if not self._fresh(self.base, now):
            motion.append('base odometry stale')
        elif not self.base[0]:
            motion.append('base moving')
        if not joints:
            motion.append('no joint velocity evidence')
        for name in joints:
            sample = self.joint_samples.get(name)
            if not self._fresh(sample, now):
                motion.append(f'{name} velocity stale')
            elif not sample[0]:
                motion.append(f'{name} moving')
        reasons += motion
        return SafetyReport(protection, not released, released, not motion,
                            '; '.join(reasons) if reasons else 'protection ready, e-stop released, robot stopped')
