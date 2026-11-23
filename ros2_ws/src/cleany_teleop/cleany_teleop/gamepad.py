"""Map left-stick translation and right-stick yaw to manual control."""

import math
from dataclasses import dataclass

CONDITIONED_AXIS_COUNT = 10
SETTLE_SECONDS = 0.060


class _AxisSnapback:
    def __init__(self):
        self.direction = 0
        self.settling_since = None

    def apply(self, value, direction, sampled_at, stick_released):
        changed = direction != self.direction
        released = stick_released or (changed and self.direction != 0)
        if released or (changed and self.settling_since is not None):
            self.settling_since = sampled_at
        self.direction = direction
        if self.settling_since is not None:
            if sampled_at - self.settling_since < SETTLE_SECONDS:
                return 0.0
            self.settling_since = None
        return value if direction else 0.0


class StickSnapbackFilter:
    """Detect each axis crossing neutral or reversing at the receiver timestamp."""

    def __init__(self, center_band=0.0):
        self.center_band = center_band
        self.reset()

    def reset(self):
        self.axes = (_AxisSnapback(), _AxisSnapback())
        self.output = (0.0, 0.0)
        self.released = False

    def apply(self, axes, sampled_at):
        directions = tuple((v > self.center_band) - (v < -self.center_band) for v in axes)
        previous_directions = tuple(guard.direction for guard in self.axes)
        # Suppress the orthogonal rebound when two samples straddle neutral.
        stick_released = any(previous_directions) and not any(
            old != 0 and old == new for old, new in zip(previous_directions, directions)
        )
        previous = self.output
        self.output = tuple(
            guard.apply(value, direction, sampled_at, stick_released)
            for guard, value, direction in zip(self.axes, axes, directions)
        )
        self.released = any(abs(old) > self.center_band and new == 0
                            for old, new in zip(previous, self.output))
        return self.output


def physical_axes(axes):
    """Return physical axes in logical order, rejecting truncated extended reports."""
    if len(axes) == CONDITIONED_AXIS_COUNT:
        return (axes[6], axes[7], axes[2], axes[8], axes[9], axes[5])
    if len(axes) > 6:
        return ()
    return axes


@dataclass(frozen=True)
class Mapping:
    forward_axis: int = 1
    left_axis: int = 0
    slow_trigger_axis: int = 2
    fast_trigger_axis: int = 5
    trigger_threshold: float = 0.5
    slow_mps: float = 0.1
    fast_mps: float = 0.5
    turn_axis: int = 3
    deadzone: float = 0.08
    speed: float = 0.5
    max_age: float = 0.15
    update_period: float = 0.07

    def __post_init__(self):
        indices = (self.forward_axis, self.left_axis, self.slow_trigger_axis,
                   self.fast_trigger_axis, self.turn_axis)
        if min(indices) < 0 or self.turn_axis in indices[:-1]:
            raise ValueError("회전 축은 다른 조종 축과 구분해야 하며 축 번호는 음수일 수 없습니다")
        if (self.slow_trigger_axis == self.fast_trigger_axis or not 0 < self.trigger_threshold <= 1
                or not 0 < self.slow_mps <= self.fast_mps or not math.isfinite(self.fast_mps)):
            raise ValueError("트리거 축, 임계값 또는 속도 설정 오류")
        if not (0 <= self.deadzone < 1 and 0.1 <= self.speed <= 10):
            raise ValueError("게임패드 속도 또는 데드존 설정 오류")
        if not 0 < self.update_period < self.max_age < 0.25:
            raise ValueError("게임패드 갱신 주기와 입력 만료 설정 오류")


class GamepadControl:
    """Acquire or renew control from fresh input after observing initial neutral."""

    def __init__(self, mapping: Mapping, profile: dict):
        self.mapping = mapping
        radius = profile["geometry"]["wheel_radius_m"]
        limits = profile["limits"]
        bounds = [radius, limits["linear_x_mps"], limits["linear_y_mps"], limits["wheel_rad_s"]]
        if not all(math.isfinite(v) and v > 0 for v in bounds):
            raise ValueError("베이스 프로필의 휠 반지름과 속도 제한을 확인하세요")
        # The phone controller expects wheel speed in radians per second.
        self.max_translation_mps = min(limits["linear_x_mps"], limits["linear_y_mps"],
                                       radius * min(limits["wheel_rad_s"], 10.0))
        self.slow_speed = min(mapping.slow_mps, self.max_translation_mps) / radius
        self.fast_speed = min(mapping.fast_mps, self.max_translation_mps) / radius
        self.command_speed = mapping.speed
        self.stick = StickSnapbackFilter()
        self.turn_stick = StickSnapbackFilter()
        self.last_motion = (0.0, 0.0, 0.0)
        self.active = False
        self.startup_neutral_seen = False
        self.resume_after = -math.inf
        self.forward_only = False
        self.last_sent = -math.inf
        self.deadline = -math.inf
        self.last_stamp = -math.inf

    def interrupt(self, now):
        active = self.active
        self.stick.reset()
        self.turn_stick.reset()
        self.last_motion = (0.0, 0.0, 0.0)
        self.active = False
        self.forward_only = False
        self.resume_after = now
        self.deadline = -math.inf
        return ("stop", None) if active else None

    def timeout(self, now):
        if self.deadline != -math.inf and now >= self.deadline:
            return self.interrupt(self.deadline)
        return None

    def sample(self, axes, buttons, stamp, age, now, *, can_press=True, can_update=True):
        m = self.mapping
        if not len(axes) or not math.isfinite(age) or not math.isfinite(stamp) or age < 0:
            return self.interrupt(now)
        if age >= m.max_age or stamp <= self.last_stamp:
            return self.timeout(now)
        self.last_stamp = stamp
        expired = self.timeout(now)
        if expired or now - age <= self.resume_after:
            return expired
        self.deadline = now + m.max_age - age
        try:
            forward, left = axes[m.forward_axis], axes[m.left_axis]
            raw = physical_axes(axes)
            raw_forward, raw_left = raw[m.forward_axis], raw[m.left_axis]
            lt, rt = axes[m.slow_trigger_axis], axes[m.fast_trigger_axis]
            turn, raw_turn = axes[m.turn_axis], raw[m.turn_axis]
        except IndexError:
            return self.interrupt(now)
        if not (0 <= lt <= 1 and 0 <= rt <= 1):
            return self.interrupt(now)
        if len(buttons) < 6 or any(b not in (0, 1) for b in buttons) or not all(
            math.isfinite(v) and abs(v) <= 1 for v in (forward, left, turn, raw_forward, raw_left, raw_turn)
        ):
            return self.interrupt(now)
        radius = math.hypot(forward, left)
        if radius <= m.deadzone:
            forward = left = 0.0
        elif radius > 1.0:
            forward, left = forward / radius, left / radius
        if abs(turn) <= m.deadzone:
            turn = 0.0
        if math.hypot(raw_forward, raw_left) <= m.deadzone and abs(raw_turn) <= m.deadzone:
            self.startup_neutral_seen = True
            self.forward_only = False
        if not self.startup_neutral_seen:
            return None
        if self.forward_only:
            left = 0.0
        if len(axes) != CONDITIONED_AXIS_COUNT:
            forward, left = self.stick.apply((forward, left), stamp)
            turn, _ = self.turn_stick.apply((turn, 0.0), stamp)
        if forward == left == turn == 0.0:
            self.last_motion = (0.0, 0.0, 0.0)
            if self.active:
                self.active = False
                return ("stop", None)
            return None
        if not self.active:
            if not can_press:
                return None
            self.active = True
            # Negotiate lateral movement before sending a velocity from the next report.
            self.last_sent = -math.inf
            self.command_speed = m.speed
            return ("press", [0.0, 0.0, 0.0])
        if not can_update:
            return None
        translation_speed = m.speed
        if lt >= m.trigger_threshold:
            translation_speed = self.slow_speed
        elif rt >= m.trigger_threshold:
            translation_speed = self.fast_speed
        speed = max(translation_speed if forward or left else 0.0, m.speed if turn else 0.0)
        motion = (forward, left, turn)
        axis_released = any(old and not new for old, new in zip(self.last_motion, motion))
        if (not axis_released and speed == self.command_speed
                and now - self.last_sent < m.update_period):
            return None
        self.command_speed = speed
        self.last_motion = motion
        self.last_sent = now
        return ("update", [forward * (translation_speed / speed), left * (translation_speed / speed),
                           turn * (m.speed / speed)])
