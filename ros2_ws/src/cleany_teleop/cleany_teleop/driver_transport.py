"""Adapt guarded wheel intent to the sole base driver's Twist/permission port."""

from __future__ import annotations

from dataclasses import dataclass
import traceback
import uuid

from .core import newer
from .web_control import Control


@dataclass(frozen=True)
class Geometry:
    radius: float
    wheelbase: float
    separation: float


def body_velocity(
    wheels: tuple[float, ...], geometry: Geometry
) -> tuple[float, float, float]:
    fl, fr, rl, rr = wheels
    radius = geometry.radius
    arm = (geometry.wheelbase + geometry.separation) / 2
    return (
        radius * (fl + fr + rl + rr) / 4,
        radius * (-fl + fr + rl - rr) / 4,
        radius * (-fl + fr - rl + rr) / (4 * arm),
    )


class DriverTransport:
    """Correlate permission RPC replies with fresh MCU feedback, never RPC alone."""

    def __init__(self, request, velocity, geometry: Geometry):
        self.request, self.velocity, self.geometry = request, velocity, geometry
        self.client = "phone-" + uuid.uuid4().hex
        self.sequence = 0
        self.key = self.future = None
        self.permission_status = None
        self.last_permission_error = None

    def publish(self, command):
        if command.mode == Control.VELOCITY:
            self.velocity(body_velocity(command.velocity_rad_s, self.geometry))
            return
        key = command.mode, command.sequence
        if key == self.key:
            return
        self.key = key
        self.sequence += 1
        self.permission_status = dict(
            stage="rpc_pending",
            client=self.client,
            request_sequence=self.sequence,
            mode=command.mode,
        )
        try:
            self.future = self.request(
                self.client, self.sequence, command.mode == Control.ENABLE
            )
        except Exception as error:
            self.future = None
            self.fail_permission("rpc_failed", str(error), error)

    def acknowledged(self, command, state) -> bool:
        if (
            command is None
            or self.key != (command.mode, command.sequence)
            or self.future is None
            or not self.future.done()
            or self.permission_status["stage"]
            in ("rpc_failed", "rpc_rejected", "rpc_empty")
        ):
            return False
        try:
            reply = self.future.result()
        except Exception as error:
            self.fail_permission("rpc_failed", str(error), error)
            return False
        if reply is None:
            self.fail_permission(
                "rpc_empty", "Base permission service returned no response"
            )
            return False
        if not reply.accepted:
            self.fail_permission("rpc_rejected", reply.reason)
            return False
        self.permission_status.update(
            boot_id=reply.boot_id, command_sequence=reply.command_sequence
        )
        if state is None:
            self.permission_status["stage"] = "mcu_feedback_missing"
            return False
        if reply.boot_id != state.boot_id:
            self.permission_status["stage"] = "mcu_boot_mismatch"
            return False
        acknowledged = state.last_command_sequence == reply.command_sequence or newer(
            state.last_command_sequence, reply.command_sequence
        )
        self.permission_status["stage"] = (
            "acknowledged" if acknowledged else "mcu_ack_pending"
        )
        return acknowledged

    def fail_permission(self, code, reason, error: Exception | None = None):
        self.permission_status.update(
            stage=code,
            code=code,
            reason=reason,
            exception_type=type(error).__name__ if error else None,
            traceback="".join(
                traceback.format_exception(type(error), error, error.__traceback__)
            )
            if error
            else None,
        )
        self.last_permission_error = dict(self.permission_status)


class DriverControl(Control):
    def __init__(self, transport, **kwargs):
        self.transport = transport
        super().__init__(transport.publish, **kwargs)
        self.reported_permission_error = None

    def ack(self):
        result = self.transport.acknowledged(self.pending, self.state)
        error = self.transport.last_permission_error
        if error is not None and error != self.reported_permission_error:
            self.event("permission_failure", **error)
            self.reported_permission_error = dict(error)
        return result

    def tick(self):
        # RPC diagnostics must remain observable even when MCU feedback is absent.
        self.ack()
        super().tick()

    def stop(self, reason, source="safety"):
        if self.transport.permission_status:
            self.event("permission_at_stop", **self.transport.permission_status)
        super().stop(reason, source)

    def status(self):
        status = super().status()
        status["permission"] = dict(self.transport.permission_status or {})
        status["last_permission_error"] = dict(
            self.transport.last_permission_error or {}
        )
        if self.state is not None:
            status["target"] = [float(v) for v in self.state.target_rad_s]
        return status
