"""Camera-direction interlock for continuous tracking or legacy stop/look/move."""
from dataclasses import dataclass
import math


@dataclass
class PanGate:
    angle_tolerance: float = .04
    settle_s: float = .2
    source_timeout_s: float = .5
    linear_stopped: float = .01
    angular_stopped: float = .02
    pan_stopped: float = .05
    allow_reverse: bool = True
    allow_lateral: bool = False
    allow_mixed_motion: bool = False
    translation_angle_deg: float = 0.
    pan_min_position_rad: float = -math.pi
    pan_max_position_rad: float = math.pi
    target: float = 0.
    aligned_since: float | None = None
    last_time: float = -math.inf

    def __post_init__(self) -> None:
        if (not all(math.isfinite(v) for v in (self.pan_min_position_rad, self.pan_max_position_rad))
                or not math.isfinite(self.pan_max_position_rad-self.pan_min_position_rad)
                or not self.pan_min_position_rad <= 0. <= self.pan_max_position_rad
                or self.pan_min_position_rad >= self.pan_max_position_rad):
            raise ValueError('Pan position limits must be finite, ordered and include the front pose 0')
        if not math.isfinite(self.translation_angle_deg) or not 0 <= self.translation_angle_deg <= 10:
            raise ValueError('translation_angle_deg must be in [0, 10]')
        if self.allow_lateral and self.translation_angle_deg:
            raise ValueError('Choose lateral or bounded diagonal motion')

    def _position_target(self, bearing: float, pan: float) -> float | None:
        """Choose the nearest exactly equivalent position within joint limits.

        A bounded position servo sees raw angle differences, so canonical
        atan2 angles on either side of pi can otherwise command a full turn.
        No clipping or angle-tolerance relaxation is allowed here.
        """
        period = 2*math.pi
        lower = math.ceil((self.pan_min_position_rad-bearing)/period)
        upper = math.floor((self.pan_max_position_rad-bearing)/period)
        if lower > upper:
            return None
        nearest = min(upper, max(lower, round((pan-bearing)/period)))
        candidates = [bearing+k*period for k in {lower, upper, nearest}]
        candidates = [v for v in candidates if self.pan_min_position_rad <= v <= self.pan_max_position_rad]
        return min(candidates, key=lambda v: (abs(v-pan), abs(v-bearing))) if candidates else None

    def step(self, now: float, command: tuple[float, float, float],
             measured: tuple[float, float, float], pan: float, pan_speed: float,
             joint_stamp: float, odom_stamp: float, depth_stamp: float
             ) -> tuple[float, tuple[float, float, float], str]:
        zero = (0., 0., 0.)
        if now < self.last_time:
            self.aligned_since = None
        self.last_time = now
        if not all(math.isfinite(v) for v in (*command, *measured, pan, pan_speed, now)):
            self.aligned_since = None
            return self.target, zero, 'INVALID_INPUT'
        if any(not 0 <= now-stamp <= self.source_timeout_s for stamp in (joint_stamp, odom_stamp)):
            self.aligned_since = None
            return self.target, zero, 'STALE_FEEDBACK'
        x, y, w = command
        diagonal = (self.translation_angle_deg > 0 and abs(x) > 1e-6 and abs(w) <= 1e-6
                    and abs(y) <= abs(x)*math.tan(math.radians(self.translation_angle_deg))+1e-12)
        if sum(abs(v) > 1e-6 for v in command) > 1 and not diagonal and not self.allow_mixed_motion:
            return self.target, zero, 'MIXED_AXIS'
        if abs(y) > 1e-6 and not self.allow_lateral and not diagonal:
            return self.target, zero, 'LATERAL_NOT_ENABLED'
        if x < -1e-6 and not self.allow_reverse:
            return self.target, zero, 'REVERSE_NOT_ENABLED'
        desired = (math.atan2(y, x) if self.allow_mixed_motion and math.hypot(x, y) > 1e-6 else
                   math.copysign(math.pi/2, y) if abs(y) > 1e-6 and not diagonal else
                   math.pi if x < -1e-6 else 0. if x > 1e-6 else self.target)
        desired_position = self._position_target(desired, pan)
        if desired_position is None:
            self.aligned_since = None
            return self.target, zero, 'PAN_TARGET_OUT_OF_RANGE'
        desired = desired_position
        stopped = math.hypot(*measured[:2]) <= self.linear_stopped and abs(measured[2]) <= self.angular_stopped
        if self.allow_mixed_motion:
            # Track a continuous body-frame travel bearing. Tiny DWB corrections
            # must not restart settling or require a complete chassis stop.
            self.target = desired
            error = math.atan2(math.sin(pan-self.target), math.cos(pan-self.target))
            if abs(error) > self.angle_tolerance:
                self.aligned_since = None
                return self.target, zero, 'TURNING_PAN'
            if self.aligned_since is None:
                self.aligned_since = now
            # Exact-stamped TF already compensates for camera/body motion.
            # Require an observation after alignment, not mechanical standstill
            # or an arbitrary stabilization delay in holonomic tracking mode.
            if not (self.aligned_since <= depth_stamp <= now
                    and now-depth_stamp <= self.source_timeout_s):
                return self.target, zero, 'WAIT_FRESH_DEPTH'
            return self.target, command, 'PASS'
        if desired != self.target:
            self.aligned_since = None
            if not stopped:
                return self.target, zero, 'STOP_BEFORE_PAN'
            self.target = desired
        aligned = abs(pan-self.target) <= self.angle_tolerance and abs(pan_speed) <= self.pan_stopped
        if not aligned:
            self.aligned_since = None
            return self.target, zero, 'TURNING_PAN'
        if self.aligned_since is None:
            if not stopped:
                return self.target, zero, 'WAIT_FOR_STOP'
            self.aligned_since = now
        if now-self.aligned_since < self.settle_s:
            return self.target, zero, 'SETTLING'
        if not (self.aligned_since+self.settle_s <= depth_stamp <= now
                and now-depth_stamp <= self.source_timeout_s):
            return self.target, zero, 'WAIT_FRESH_DEPTH'
        return self.target, command, 'PASS'


@dataclass
class PanTracker:
    """Best-effort observation direction. Never grants permission to drive."""
    positions: PanGate
    target_change_rad: float = math.radians(10.)
    minimum_speed: float = .01
    target: float = 0.

    def __post_init__(self) -> None:
        if (not math.isfinite(self.target_change_rad) or not 0 < self.target_change_rad < math.pi
                or not math.isfinite(self.minimum_speed) or self.minimum_speed <= 0):
            raise ValueError('Invalid pan tracking thresholds')

    def step(self, command: tuple[float, float, float], pan: float) -> tuple[float, str]:
        if not all(math.isfinite(v) for v in (*command, pan)):
            return self.target, 'INVALID_INPUT'
        if math.hypot(*command[:2]) < self.minimum_speed:
            return self.target, 'HOLD_IDLE'
        bearing = math.atan2(command[1], command[0])
        error = math.atan2(math.sin(bearing-self.target), math.cos(bearing-self.target))
        if abs(error) < self.target_change_rad:
            return self.target, 'HOLD_DIRECTION'
        target = self.positions._position_target(bearing, pan)
        if target is None:
            return self.target, 'PAN_TARGET_OUT_OF_RANGE'
        self.target = target
        return self.target, 'TRACKING'
