"""Raw-feedback contact decision adopted from bilateral experiments.

CONTACT means motor resistance before the closed goal. It does not establish
target identity, lifting, retention, calibrated force, or runtime activation.
Values below are supplied by configs/robot/gripper_contact_decision.yaml.
"""
from bisect import bisect_right
from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
from statistics import median
from pathlib import Path
from typing import Mapping, Sequence


class ContactState(str, Enum):
    CONTACT = 'CONTACT'
    EMPTY = 'EMPTY'
    UNKNOWN = 'UNKNOWN'


@dataclass(frozen=True)
class ContactCriteria:
    motor_id: int
    closed_goal_raw: int
    opening_direction: int
    output_limit_raw: int
    window_sec: float
    maximum_gap_sec: float
    maximum_age_sec: float
    residual_threshold_ticks: int
    current_threshold_raw: int
    current_stop_raw: int
    load_threshold_raw: int
    voltage_min_raw: int
    voltage_max_raw: int
    temperature_limit_c: float

    def __post_init__(self) -> None:
        if (not 1 <= self.motor_id <= 253 or not 0 <= self.closed_goal_raw <= 4095
                or self.opening_direction not in (-1, 1)
                or not 1 <= self.output_limit_raw <= 1000
                or not 1 <= self.current_threshold_raw < self.current_stop_raw <= 32767
                or not 1 <= self.load_threshold_raw <= 1000
                or not 0 <= self.voltage_min_raw <= self.voltage_max_raw <= 255):
            raise ValueError('Invalid gripper identity or register limits')
        for value in (self.window_sec, self.maximum_gap_sec, self.maximum_age_sec,
                      self.residual_threshold_ticks,
                      self.temperature_limit_c):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Contact time, position and temperature limits must be positive')


@dataclass(frozen=True)
class ContactSample:
    stamp_ns: int
    motor_id: int
    position_raw: int | None
    goal_position_raw: int | None
    current_raw: int | None  # Decoded absolute magnitude; not the encoded word.
    load_magnitude_raw: int | None
    torque_enabled: int | None
    moving_flag: int | None
    motor_status: int | None
    protocol_error_code: int | None
    torque_limit_readback_raw: int | None
    voltage_raw: int | None
    temperature_c: int | None


@dataclass(frozen=True)
class ContactDecision:
    state: ContactState
    reason: str
    sample_count: int = 0
    window_sec: float = 0.
    minimum_residual_ticks: float | None = None
    position_span_ticks: float | None = None
    median_current_raw: float | None = None
    minimum_load_raw: float | None = None


def load_contact_criteria(path: str | Path) -> dict[str, ContactCriteria]:
    import yaml
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or data.get('schema_version') != 1:
        raise ValueError('Invalid gripper contact configuration')
    result = {arm: ContactCriteria(**data['criteria'], **data['arms'][arm])
              for arm in ('left', 'right')}
    if result['left'].motor_id != 6 or result['right'].motor_id != 12:
        raise ValueError('Contact configuration does not match the physical gripper IDs')
    return result


class LiveContactHistory:
    """Use hardware acquisition timestamps, never broadcaster replay timestamps."""
    def __init__(self, criteria: ContactCriteria):
        self.criteria = criteria
        self.samples: deque[ContactSample] = deque(maxlen=256)
        self.invalid_reason = ''
        self.high_water_stamp_ns = 0

    def invalidate(self, reason: str) -> None:
        self.samples.clear()
        self.invalid_reason = reason

    def observe(self, interfaces: Mapping[str, float]) -> None:
        try:
            stamp = interfaces['feedback_stamp_ns']
            if not math.isfinite(stamp) or stamp <= 0 or stamp != int(stamp):
                raise ValueError('Invalid hardware acquisition timestamp')
            if interfaces['current_limit_raw'] != self.criteria.current_stop_raw:
                self.invalidate('current_protection_configuration_mismatch')
                return
            aliases = {'temperature_c': 'temperature_raw', 'motor_status': 'motor_status_raw'}
            values = {name: interfaces[aliases.get(name, name)]
                      for name in ContactSample.__dataclass_fields__
                      if name not in ('stamp_ns', 'motor_id')}
            if any(not math.isfinite(v) for v in values.values()):
                raise ValueError('Invalid hardware contact telemetry')
            if int(stamp) <= self.high_water_stamp_ns:
                return  # A duplicate source snapshot must not renew evidence.
            sample = ContactSample(int(stamp), self.criteria.motor_id, **values)
            self.high_water_stamp_ns = sample.stamp_ns
            self.samples.append(sample)
            self.invalid_reason = ''
        except (KeyError, TypeError, ValueError, OverflowError):
            self.invalidate('required_contact_telemetry_missing_or_invalid')

    def evaluate(self, *, now_ns: int, after_ns: int = 0) -> ContactDecision:
        if self.invalid_reason:
            return ContactDecision(ContactState.UNKNOWN, self.invalid_reason)
        return evaluate_contact(list(self.samples), self.criteria, now_ns=now_ns, after_ns=after_ns)


def evaluate_contact(samples: Sequence[ContactSample], criteria: ContactCriteria,
                     *, now_ns: int, after_ns: int) -> ContactDecision:
    """Evaluate a trailing window with samples strictly after the close request.

Missing fields stay missing. Disagreement is UNKNOWN rather than evidence of
an empty gripper. The caller supplies samples from one monotonic clock domain.
"""
    def unknown(reason: str) -> ContactDecision:
        return ContactDecision(ContactState.UNKNOWN, reason)

    c = criteria
    if not samples:
        return unknown('feedback_missing')
    if any(not isinstance(s.stamp_ns, int) for s in samples):
        return unknown('invalid_feedback_timestamp')
    if any(b.stamp_ns <= a.stamp_ns for a, b in zip(samples, samples[1:])):
        return unknown('replayed_or_out_of_order_feedback')
    latest = samples[-1].stamp_ns
    if not 0 <= now_ns-latest <= round(c.maximum_age_sec*1e9):
        return unknown('stale_or_future_feedback')
    boundary = latest-round(c.window_sec*1e9)
    start = bisect_right([s.stamp_ns for s in samples], boundary)-1
    if start < 0:
        return unknown('contact_window_incomplete')
    w = samples[start:]
    if len(w) < 3 or w[0].stamp_ns <= after_ns:
        return unknown('contact_window_incomplete_or_before_request')
    if any(b.stamp_ns-a.stamp_ns >= round(c.maximum_gap_sec*1e9)
           for a, b in zip(w, w[1:])):
        return unknown('feedback_gap')
    for s in w:
        values = (s.motor_id, s.position_raw, s.goal_position_raw, s.current_raw,
                  s.load_magnitude_raw, s.torque_enabled, s.moving_flag,
                  s.motor_status, s.protocol_error_code, s.torque_limit_readback_raw,
                  s.voltage_raw, s.temperature_c)
        if any(not isinstance(v, (int, float)) or isinstance(v, bool)
               or not math.isfinite(v) or v != int(v) for v in values):
            return unknown('missing_or_malformed_register')
        if (not 0 <= s.position_raw <= 4095 or not 0 <= s.current_raw <= 32767
                or not 0 <= s.load_magnitude_raw <= 1000):
            return unknown('invalid_decoded_register_range')
        if s.motor_id != c.motor_id:
            return unknown('wrong_gripper')
        if s.goal_position_raw != c.closed_goal_raw or s.torque_limit_readback_raw != c.output_limit_raw:
            return unknown('closed_goal_or_output_profile_mismatch')
        if s.torque_enabled != 1:
            return unknown('torque_not_enabled')
        if s.motor_status != 0 or s.protocol_error_code != 0:
            return unknown('motor_or_packet_fault')
        if s.current_raw >= c.current_stop_raw:
            return unknown('current_protection_limit')
        if (not c.voltage_min_raw <= s.voltage_raw <= c.voltage_max_raw
                or not 0 <= s.temperature_c < c.temperature_limit_c):
            return unknown('voltage_or_temperature_invalid')
        if s.moving_flag != 0:
            return unknown('moving_flag_not_zero')
    positions = [s.position_raw for s in w]
    # Diagnostic only: compliant objects can let a loaded jaw close further.
    span = max(positions)-min(positions)
    residuals = [(s.position_raw-c.closed_goal_raw)*c.opening_direction for s in w]
    current = median(s.current_raw for s in w)
    load = min(s.load_magnitude_raw for s in w)
    state, reason = ContactState.UNKNOWN, 'inconsistent_contact_signals'
    if (min(residuals) > c.residual_threshold_ticks
            and current >= c.current_threshold_raw and load >= c.load_threshold_raw):
        state, reason = ContactState.CONTACT, 'loaded_before_closed_goal'
    elif (max(abs(r) for r in residuals) <= c.residual_threshold_ticks
          and current < c.current_threshold_raw
          and max(s.load_magnitude_raw for s in w) < c.load_threshold_raw):
        state, reason = ContactState.EMPTY, 'empty_closed'
    return ContactDecision(state, reason, len(w), (w[-1].stamp_ns-w[0].stamp_ns)/1e9,
                           min(residuals), span, float(current), load)
