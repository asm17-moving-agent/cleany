"""Bounded asynchronous RGB monitoring; never labels occlusion as proven drop."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TrackingEvidence:
    reference_id: str
    arm: str
    source_snapshot_id: str
    source_object_id: int
    stamp_ns: int
    frame_id: str
    valid: bool
    visible: bool
    reason: str


class CarryGuard:
    def __init__(self, maximum_capture_age_sec: float = 12., maximum_update_age_sec: float = 8.):
        if not all(math.isfinite(v) and v > 0 for v in (maximum_capture_age_sec, maximum_update_age_sec)):
            raise ValueError('Carry tracking age limits must be finite and positive')
        self.capture_age = maximum_capture_age_sec
        self.update_age = maximum_update_age_sec
        self.identity: tuple[str, str, str, int] | None = None
        self.latest: TrackingEvidence | None = None
        self.received: float | None = None
        self.armed = False
        self.fault: str | None = None

    def bind(self, reference_id: str, arm: str, source: str, object_id: int) -> None:
        if not reference_id or arm not in ('left', 'right') or not source or object_id <= 0:
            raise ValueError('Invalid carry tracking identity')
        self.identity = (reference_id, arm, source, object_id)
        self.latest = self.received = self.fault = None
        self.armed = False

    def update(self, evidence: TrackingEvidence, received_sec: float) -> None:
        identity = (evidence.reference_id, evidence.arm, evidence.source_snapshot_id, evidence.source_object_id)
        if identity != self.identity:
            return
        invalid = (not evidence.valid or not evidence.visible or evidence.stamp_ns <= 0
                   or evidence.frame_id != f'{evidence.arm}_wrist_rgb_optical_frame')
        if invalid and self.armed:
            self.fault = f'Grasp anomaly suspected: {evidence.reason or "invalid wrist evidence"}'
        if self.latest is not None and evidence.stamp_ns <= self.latest.stamp_ns and not invalid:
            return  # Duplicate messages cannot refresh a dead camera heartbeat.
        self.latest, self.received = evidence, received_sec

    def arm(self, now_ns: int, now_sec: float) -> None:
        self.armed = True
        self.check(now_ns, now_sec)

    def disarm(self) -> None:
        self.armed = False

    def check(self, now_ns: int, now_sec: float) -> None:
        if not self.armed:
            return
        evidence = self.latest
        reason = self.fault
        if reason is None:
            if evidence is None:
                reason = 'No continuous wrist tracking evidence before carry'
            elif (not evidence.valid or not evidence.visible or evidence.stamp_ns <= 0
                  or evidence.frame_id != f'{evidence.arm}_wrist_rgb_optical_frame'):
                reason = f'Grasp anomaly suspected: {evidence.reason}'
            elif not 0 <= (now_ns-evidence.stamp_ns)/1e9 <= self.capture_age:
                reason = ('Wrist tracking capture is stale or from the future: '
                          f'age={(now_ns-evidence.stamp_ns)/1e9:.3f}s '
                          f'limit={self.capture_age:.3f}s (ROS clock)')
            elif not 0 <= now_sec-self.received <= self.update_age:
                reason = ('Wrist tracking updates stopped: '
                          f'age={now_sec-self.received:.3f}s '
                          f'limit={self.update_age:.3f}s (wall clock)')
        if reason is not None:
            self.fault = reason
            raise RuntimeError(reason)


class ContactLossGuard:
    """Debounce transient compliant-jaw motion without pausing the arm."""

    def __init__(self, grace_seconds: float = .3):
        if not math.isfinite(grace_seconds) or grace_seconds <= 0:
            raise ValueError('Contact loss grace must be finite and positive')
        self.grace_seconds = grace_seconds
        self.missing_since: float | None = None

    def check(self, healthy: bool, now_sec: float) -> None:
        if healthy:
            self.missing_since = None
        elif self.missing_since is None:
            self.missing_since = now_sec
        elif now_sec-self.missing_since >= self.grace_seconds:
            raise RuntimeError('Grasp anomaly suspected: gripper contact lost persistently')
