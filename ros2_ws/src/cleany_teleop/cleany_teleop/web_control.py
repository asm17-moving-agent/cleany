"""Single-owner, expiring web-control intent mapped to MCU protocol v2."""

from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import time

from cleany_teleop.core import FeedbackClock, newer, wheel_targets, joystick_targets


@dataclass
class Command:
    mode: int
    sequence: int
    valid_until_us: int = 0
    velocity_rad_s: tuple[float, ...] = (0.0,) * 4


class Control:
    STOP, ENABLE, VELOCITY = 0, 1, 2
    MAX_WEB_VELOCITY = 10.0

    def __init__(self, publish, now=time.monotonic, event_sink=None):
        self.publish, self.now = publish, now
        self.clock = FeedbackClock()
        self.state = None
        self.phase = "idle"
        self.paused = False
        self.message = "ESP32 상태를 기다리는 중"
        self.owner = None
        self.requests = {}
        self.lease_until = 0.0
        self.started_at = 0.0
        self.pending = None
        self.phase_until = 0.0
        self.targets = (0.0,) * 4
        self.publisher_count = 1
        self.subscriber_count = 0
        self.stop_confirmed = False
        self.stop_started_at = None
        self.stop_ack_ms = None
        self.events = deque(maxlen=24)
        self.event_sink = event_sink
        self.event_sink_error = None
        self.last_stop_reason = None
        self.last_fault_reason = None

    def event(self, kind, **details):
        event = dict(at=round(self.now(), 3), kind=kind, **details)
        self.events.append(event)
        if self.event_sink is not None:
            try:
                self.event_sink(dict(event))
            except Exception as error:
                self.event_sink_error = str(error)

    def packet(self, mode, targets=(0.0,) * 4):
        sequence = self.clock.next_sequence(
            self.state.last_command_sequence if self.state else 0
        )
        deadline = (
            0
            if mode == self.STOP
            else self.clock.deadline(self.now(), self.state.watchdog_ms)
        )
        return Command(mode, sequence, deadline, targets)

    def feedback(self, state):
        if self.clock.boot_id is not None and state.boot_id != self.clock.boot_id:
            self.stop("ESP32가 재부팅되었습니다. 버튼을 놓고 다시 눌러주세요.")
            self.clock = FeedbackClock()
        try:
            if self.clock.observe(state, self.now()):
                self.state = state
                if self.message == "ESP32 상태를 기다리는 중":
                    self.message = "연결 완료 · 조이스틱을 움직이면 조종합니다"
        except RuntimeError as error:
            self.stop(str(error))

    def accept_request(self, client, sequence):
        if not isinstance(client, str) or not 1 <= len(client) <= 80:
            raise ValueError("잘못된 조종 세션")
        if type(sequence) is not int or not 0 <= sequence <= 2**53 - 1:
            raise ValueError("잘못된 요청 번호")
        if sequence <= self.requests.get(client, -1):
            raise ValueError("이전 요청은 처리하지 않습니다")
        if client not in self.requests and len(self.requests) >= 1024:
            raise ValueError("세션이 너무 많습니다. 서버를 다시 시작해주세요.")
        self.requests[client] = sequence

    def ready(self):
        self.clock.require_fresh(self.now())
        if self.publisher_count != 1 or self.subscriber_count < 1:
            raise ValueError("다른 제어기가 실행 중이거나 ESP32가 연결되지 않았습니다")

    def request(self, payload):
        client, sequence = payload.get("client"), payload.get("sequence")
        self.accept_request(client, sequence)
        action = payload.get("action")
        if action == "pause":
            self.paused = True
            self.stop(
                "지도 초기화 중 · 새 조종기 연결을 기다립니다", source="maintenance"
            )
            return
        if action == "stop":
            reason = payload.get("reason")
            self.stop(
                "정지 요청"
                + (
                    " · " + reason
                    if isinstance(reason, str) and len(reason) <= 160
                    else ""
                ),
                source="user",
            )
            return
        if self.paused:
            raise ValueError("지도 초기화 중에는 조종할 수 없습니다")
        if action == "heartbeat":
            if self.now() >= self.lease_until and self.owner == client:
                self.stop("버튼 신호가 끊겨 정지했습니다")
            if self.owner != client or self.phase not in (
                "stopping_before_enable",
                "enabling",
                "moving",
            ):
                raise ValueError("조종이 종료되었습니다. 버튼을 놓고 다시 눌러주세요.")
            self.lease_until = self.now() + 0.25
            return
        if action not in ("press", "update"):
            raise ValueError("알 수 없는 명령")
        self.ready()
        if action == "press" and (self.phase != "idle" or self.owner is not None):
            raise ValueError("이미 조종 중입니다. 먼저 버튼을 놓아주세요.")
        if action == "update":
            if self.owner == client and self.now() >= self.lease_until:
                self.stop("버튼 신호가 끊겨 정지했습니다")
            if self.owner != client or self.phase not in (
                "stopping_before_enable",
                "enabling",
                "moving",
            ):
                raise ValueError("조종이 종료되었습니다. 버튼을 놓고 다시 눌러주세요.")
        direction = payload.get("direction")
        if "vector" not in payload and direction not in (
            "forward",
            "backward",
            "left",
            "right",
            "ccw",
            "cw",
        ):
            raise ValueError("잘못된 이동 방향")
        speed = payload.get("speed")
        if isinstance(speed, bool) or not isinstance(speed, (int, float)):
            raise ValueError("잘못된 속도")
        targets = (
            joystick_targets(payload["vector"], speed)
            if "vector" in payload
            else wheel_targets(direction, speed)
        )
        if speed > min(self.MAX_WEB_VELOCITY, self.state.max_velocity_rad_s):
            raise ValueError(
                f"웹 조종 속도는 MCU 한도 및 {self.MAX_WEB_VELOCITY:g} rad/s 이하입니다"
            )
        self.targets = targets
        if action == "update":
            self.lease_until = self.now() + 0.25
            return
        self.owner = client
        self.lease_until = self.now() + 0.25
        self.started_at = self.now()
        self.pending = self.packet(self.STOP)
        self.phase = "stopping_before_enable"
        self.phase_until = self.now() + 0.15
        self.message = "이동 준비 중"
        self.stop_confirmed = False
        self.publish(self.pending)
        self.event("press", client=client, sequence=sequence)

    def stopped(self):
        return (
            self.state is not None
            and not self.state.enabled
            and all(v == 0 for v in self.state.target_rad_s)
            and all(v == 0 for v in self.state.commanded_rad_s)
            and all(v == 0 for v in self.state.pwm_percent)
        )

    def ack(self):
        return (
            self.pending is not None
            and self.state is not None
            and (
                self.state.last_command_sequence == self.pending.sequence
                or newer(self.state.last_command_sequence, self.pending.sequence)
            )
        )

    def stop(self, reason, source="safety"):
        self.last_stop_reason = reason
        if source == "safety":
            self.last_fault_reason = reason
        self.event(
            "stop",
            reason=reason,
            client=self.owner,
            source=source,
            phase=self.phase,
            feedback_age_ms=round((self.now() - self.clock.received_at) * 1000, 1),
            lease_remaining_ms=round((self.lease_until - self.now()) * 1000, 1),
            command_age_ms=int(self.state.command_age_ms) if self.state else None,
        )
        self.owner = None
        self.targets = (0.0,) * 4
        self.message = reason
        self.stop_confirmed = False
        if self.phase != "stopping":
            self.stop_started_at = self.now()
            self.stop_ack_ms = None
            self.pending = self.packet(self.STOP)
            self.phase_until = self.now() + 0.6
        self.phase = "stopping"
        self.publish(self.pending)

    def tick(self):
        now = self.now()
        if self.phase == "idle":
            return
        if self.phase == "stopping":
            self.publish(self.pending)
            if (
                self.state is not None
                and now - self.clock.received_at <= 0.1
                and self.ack()
                and self.stopped()
            ):
                self.stop_confirmed = True
                self.stop_ack_ms = round((now - self.stop_started_at) * 1000, 1)
                self.phase = "idle"
                self.message += " · 출력 0 확인"
            elif now >= self.phase_until:
                self.phase = "idle"
                self.message += " · 정지 응답 미확인"
            return
        try:
            self.ready()
            if now >= self.lease_until:
                raise ValueError("버튼 신호가 끊겨 정지했습니다")
            if self.phase == "stopping_before_enable":
                if self.ack() and self.stopped():
                    self.pending = self.packet(self.ENABLE)
                    self.phase = "enabling"
                    self.event("enabling")
                    self.phase_until = now + 0.1
                elif now >= self.phase_until:
                    raise ValueError("이동 전 정지 응답을 확인하지 못했습니다")
                self.publish(self.pending)
            elif self.phase == "enabling":
                if self.state.enabled and self.ack():
                    self.phase = "moving"
                    self.event("moving")
                    self.message = "조종 중 · 입력 해제 시 정지 요청"
                    self.publish(self.packet(self.VELOCITY, self.targets))
                elif now >= self.phase_until:
                    raise ValueError("ENABLE 응답을 확인하지 못했습니다")
                else:
                    self.publish(self.pending)
            else:
                if (
                    not self.state.enabled
                    or self.state.fault_bits
                    or self.state.command_age_ms > 150
                ):
                    raise ValueError("ESP32가 명령을 받지 못해 정지했습니다")
                self.publish(self.packet(self.VELOCITY, self.targets))
        except (ValueError, RuntimeError) as error:
            self.stop(str(error))

    def status(self):
        state = self.state
        fresh = state is not None and self.now() - self.clock.received_at <= 0.1
        return {
            "connected": fresh,
            "phase": self.phase,
            "message": self.message,
            "feedback_age_ms": round((self.now() - self.clock.received_at) * 1000, 1)
            if state
            else None,
            "boot_id": int(state.boot_id) if state else None,
            "ready": fresh
            and self.publisher_count == 1
            and self.subscriber_count > 0
            and self.phase == "idle"
            and not self.paused,
            "paused": self.paused,
            "stop_confirmed": self.stop_confirmed,
            "stop_ack_ms": self.stop_ack_ms,
            "enabled": bool(state.enabled) if state else False,
            "fault_bits": int(state.fault_bits) if state else 0,
            "target": list(self.targets),
            "web_max_velocity": min(
                self.MAX_WEB_VELOCITY, float(state.max_velocity_rad_s)
            )
            if state
            else self.MAX_WEB_VELOCITY,
            "mcu_max_velocity": float(state.max_velocity_rad_s) if state else None,
            "velocity": [float(v) for v in state.velocity_rad_s]
            if state
            else [0.0] * 4,
            "pwm": [int(v) for v in state.pwm_percent] if state else [0] * 4,
            "command_age_ms": int(state.command_age_ms) if state else None,
            "publisher_count": self.publisher_count,
            "subscriber_count": self.subscriber_count,
            "lease_remaining_ms": max(
                0, round((self.lease_until - self.now()) * 1000, 1)
            )
            if self.owner
            else 0,
            "last_stop_reason": self.last_stop_reason,
            "last_fault_reason": self.last_fault_reason,
            "events": list(self.events),
            "event_sink_error": self.event_sink_error,
        }
