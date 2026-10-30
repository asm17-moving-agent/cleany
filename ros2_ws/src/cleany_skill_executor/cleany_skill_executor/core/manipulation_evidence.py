"""Conservative gripper-resistance proxy + independently identified RGB tracking.

Raw load is NOT calibrated force/torque. This gate needs an experimentally set
threshold; it never treats motor arrival or visibility alone as physical grasp.
"""
from dataclasses import dataclass
import math

from .gripper_contact import ContactCriteria, ContactState, LiveContactHistory


@dataclass(frozen=True)
class TrackingConfig:
    maximum_age_sec: float = .5
    maximum_skew_sec: float = .2

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in (
                self.maximum_age_sec, self.maximum_skew_sec)):
            raise ValueError('Tracking age and skew limits must be finite and positive')


@dataclass(frozen=True)
class TrackingSample:
    stamp_ns: int
    arm: str
    reference_id: str
    snapshot_id: str
    object_id: int
    valid: bool
    visible: bool
    mask_pixels: int
    image_pixels: int


@dataclass(frozen=True)
class Evidence:
    success: bool
    reason: str
    stamp_ns: int = 0
    contact: bool = False
    tracked: bool = False


class RawContactHistory(LiveContactHistory):
    """Strict bilateral motor rule fused with independently identified wrist RGB."""
    def __init__(self, criteria: ContactCriteria, config: TrackingConfig):
        super().__init__(criteria)
        self.config = config

    def evaluate(self, tracking: TrackingSample | None = None, *, arm: str,
                 wrist_reference_id: str, snapshot_id: str, object_id: int,
                 after_ns: int, now_ns: int) -> Evidence:
        decision = super().evaluate(now_ns=now_ns, after_ns=after_ns)
        if decision.state == ContactState.UNKNOWN:
            return Evidence(False, decision.reason)
        if not tracking:
            return Evidence(False, 'tracking_missing')
        if (not wrist_reference_id or tracking.reference_id != wrist_reference_id
                or tracking.arm != arm or tracking.snapshot_id != snapshot_id
                or tracking.object_id != object_id):
            return Evidence(False, 'tracking_identity_mismatch')
        latest = self.samples[-1].stamp_ns
        if (tracking.stamp_ns <= after_ns
                or not 0 <= now_ns-tracking.stamp_ns <= self.config.maximum_age_sec*1e9):
            return Evidence(False, 'tracking_stale_future_or_replayed')
        if abs(latest-tracking.stamp_ns) > self.config.maximum_skew_sec*1e9:
            return Evidence(False, 'motor_camera_time_skew')
        if not tracking.valid or not 0 < tracking.mask_pixels <= tracking.image_pixels:
            return Evidence(False, 'tracking_invalid')
        return Evidence(True, decision.reason, min(latest, tracking.stamp_ns),
                        decision.state == ContactState.CONTACT, tracking.visible)
