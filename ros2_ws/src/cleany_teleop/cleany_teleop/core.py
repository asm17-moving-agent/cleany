"""Host-only helpers for the MCU protocol-v2 commissioning tool."""

from __future__ import annotations

import math
from dataclasses import dataclass


def newer(sequence: int, previous: int) -> bool:
    return 0 < ((sequence - previous) & 0xFFFFFFFF) < 0x80000000


def wheel_targets(direction: str, speed: float) -> tuple[float, ...]:
    # Logical wheel order FL, FR, RL, RR; no PCB/encoder sign corrections.
    signs = {
        "forward": (1, 1, 1, 1),
        "backward": (-1, -1, -1, -1),
        "left": (-1, 1, 1, -1),
        "right": (1, -1, -1, 1),
        "ccw": (-1, 1, -1, 1),
        "cw": (1, -1, 1, -1),
    }
    if not math.isfinite(speed) or not 0 < speed <= 10:
        raise ValueError("speed must be finite and in (0, 10] rad/s")
    return tuple(float(sign * speed) for sign in signs[direction])


def joystick_targets(vector, speed: float) -> tuple[float, ...]:
    """Normalized forward/left/CCW intent; preserve ratios at the wheel limit."""
    if not isinstance(vector, (list, tuple)) or len(vector) != 3:
        raise ValueError("잘못된 조이스틱 입력")
    if any(
        isinstance(v, bool)
        or not isinstance(v, (int, float))
        or not math.isfinite(v)
        or abs(v) > 1
        for v in vector
    ):
        raise ValueError("조이스틱 입력은 -1에서 1 사이여야 합니다")
    if not math.isfinite(speed) or not 0 < speed <= 10:
        raise ValueError("잘못된 속도")
    forward, left, ccw = vector
    wheels = (
        forward - left - ccw,
        forward + left + ccw,
        forward + left - ccw,
        forward - left + ccw,
    )
    scale = float(speed) / max(1.0, *(abs(v) for v in wheels))
    return tuple(float(v * scale) for v in wheels)


@dataclass
class FeedbackClock:
    boot_id: int | None = None
    state_sequence: int = 0
    timestamp_us: int = 0
    received_at: float = 0.0
    command_sequence: int | None = None

    def observe(self, state, now: float) -> bool:
        if state.protocol_version != 2:
            raise RuntimeError("MCU protocol version is not 2")
        if self.boot_id is not None:
            if state.boot_id != self.boot_id:
                raise RuntimeError("MCU rebooted; motion will not resume")
            if not newer(state.sequence, self.state_sequence):
                return False
            if state.timestamp_us <= self.timestamp_us:
                raise RuntimeError("MCU timestamp moved backwards")
        self.boot_id = state.boot_id
        self.state_sequence = state.sequence
        self.timestamp_us = state.timestamp_us
        self.received_at = now
        return True

    def require_fresh(self, now: float) -> None:
        if self.boot_id is None or now - self.received_at > 0.10:
            raise RuntimeError("wheel feedback missing or older than 100 ms")

    def next_sequence(self, last_accepted: int) -> int:
        if self.command_sequence is None or newer(last_accepted, self.command_sequence):
            self.command_sequence = last_accepted
        self.command_sequence = (self.command_sequence + 1) & 0xFFFFFFFF
        return self.command_sequence

    def deadline(self, now: float, watchdog_ms: int) -> int:
        self.require_fresh(now)
        lifetime_us = min(150_000, watchdog_ms * 1000 - 50_000)
        if lifetime_us < 20_000:
            raise RuntimeError("MCU watchdog too short for this tool")
        # MCU monotonic time, never wall/ROS epoch. Transport delay makes the
        # estimate conservative; every refreshed command still expires on MCU.
        return self.timestamp_us + int((now - self.received_at) * 1e6) + lifetime_us
