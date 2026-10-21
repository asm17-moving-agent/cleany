"""Conservative gripper-resistance proxy + independently identified RGB tracking.

Raw load is NOT calibrated force/torque. This gate needs an experimentally set
threshold; it never treats motor arrival or visibility alone as physical grasp.
"""
from collections import deque
from dataclasses import dataclass
import math

from .gripper_contact import ContactCriteria, ContactState, LiveContactHistory


@dataclass(frozen=True)
class ContactConfig:
    load_threshold_raw: float = 0.  # Unset; refuse positive evidence.
    closed_position_rad: float = math.nan
    opening_direction: int = 1
    closed_margin_rad: float = .02
    window_sec: float = .15
    maximum_age_sec: float = .5
    maximum_gap_sec: float = .1
    maximum_skew_sec: float = .2
    temperature_limit_c: float = 60.

    def __post_init__(self):
        if (not math.isfinite(self.load_threshold_raw) or not 0 <= self.load_threshold_raw <= 1023
                or self.opening_direction not in (-1,1)):
            raise ValueError('Invalid load threshold/direction')
        for value in (self.closed_margin_rad,self.window_sec,
                      self.maximum_age_sec,self.maximum_gap_sec,self.maximum_skew_sec,self.temperature_limit_c):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Invalid evidence limits')
        if self.window_sec > self.maximum_age_sec:
            raise ValueError('Contact window exceeds allowed evidence age')


@dataclass(frozen=True)
class MotorSample:
    stamp_ns: int
    position: float
    load_raw: float
    torque_enabled: float
    status: float
    temperature: float


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


class ContactHistory:
    def __init__(self, config: ContactConfig):
        self.config = config
        self.samples: deque[MotorSample] = deque(maxlen=200)

    def add(self, sample: MotorSample) -> None:
        if self.samples and sample.stamp_ns <= self.samples[-1].stamp_ns:
            return  # Replay cannot renew the window.
        self.samples.append(sample)

    def evaluate(self, tracking: TrackingSample | None, *, arm: str, wrist_reference_id: str,
                 snapshot_id: str, object_id: int, after_ns: int, now_ns: int) -> Evidence:
        c = self.config
        if c.load_threshold_raw <= 0 or not math.isfinite(c.closed_position_rad):
            return Evidence(False, 'contact_threshold_or_closed_reference_unconfigured')
        if not tracking or not self.samples:
            return Evidence(False, 'motor_or_tracking_missing')
        if (not wrist_reference_id or tracking.reference_id != wrist_reference_id
                or tracking.arm != arm or tracking.snapshot_id != snapshot_id or tracking.object_id != object_id):
            return Evidence(False, 'tracking_identity_mismatch')
        latest = self.samples[-1]
        for stamp in (latest.stamp_ns,tracking.stamp_ns):
            if stamp <= after_ns or not 0 <= now_ns-stamp <= c.maximum_age_sec*1e9:
                return Evidence(False, 'stale_future_or_replayed_evidence')
        if abs(latest.stamp_ns-tracking.stamp_ns) > c.maximum_skew_sec*1e9:
            return Evidence(False, 'motor_camera_time_skew')
        # Include one sample before the window boundary to establish duration.
        boundary = latest.stamp_ns-int(c.window_sec*1e9)
        samples = list(self.samples)
        start = max((i for i,s in enumerate(samples) if s.stamp_ns <= boundary),default=-1)
        if start < 0:
            return Evidence(False, 'contact_window_incomplete')
        samples = samples[start:]
        if len(samples) < 3 or any(b.stamp_ns-a.stamp_ns > c.maximum_gap_sec*1e9 for a,b in zip(samples,samples[1:])):
            return Evidence(False, 'contact_window_has_gap')
        for s in samples:
            if (not all(math.isfinite(v) for v in (s.position,s.load_raw,s.torque_enabled,s.status,s.temperature))
                    or s.torque_enabled != 1 or s.status != 0 or not 0 <= s.temperature < c.temperature_limit_c
                    or not 0 <= s.load_raw <= 2047 or s.load_raw != int(s.load_raw)):
                return Evidence(False, 'invalid_or_faulted_motor_telemetry')
        if not tracking.valid or not 0 < tracking.mask_pixels <= tracking.image_pixels:
            return Evidence(False, 'tracking_invalid')
        contact = (all((int(s.load_raw)&1023) >= c.load_threshold_raw for s in samples)
            and all((s.position-c.closed_position_rad)*c.opening_direction >= c.closed_margin_rad for s in samples))
        return Evidence(True, 'resistance_proxy' if contact else 'no_sustained_resistance',
                        min(latest.stamp_ns,tracking.stamp_ns),contact,tracking.visible)


class RawContactHistory(LiveContactHistory):
    """Strict bilateral motor rule fused with independently identified wrist RGB."""
    def __init__(self, criteria: ContactCriteria, config: ContactConfig):
        super().__init__(criteria)
        self.config = config  # Tracking age/skew limits; legacy load gate is unused.

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
