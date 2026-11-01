"""Adapt guarded wheel intent to the sole base driver's Twist/permission port."""

from __future__ import annotations

from dataclasses import dataclass
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

    def publish(self, command):
        if command.mode == Control.VELOCITY:
            self.velocity(body_velocity(command.velocity_rad_s, self.geometry))
            return
        key = command.mode, command.sequence
        if key == self.key:
            return
        self.key = key
        self.sequence += 1
        self.future = self.request(
            self.client, self.sequence, command.mode == Control.ENABLE
        )

    def acknowledged(self, command, state) -> bool:
        if (
            command is None
            or state is None
            or self.key != (command.mode, command.sequence)
            or self.future is None
            or not self.future.done()
        ):
            return False
        try:
            reply = self.future.result()
            return (
                reply is not None
                and reply.accepted
                and reply.boot_id == state.boot_id
                and (
                    state.last_command_sequence == reply.command_sequence
                    or newer(state.last_command_sequence, reply.command_sequence)
                )
            )
        except Exception:
            return False


class DriverControl(Control):
    def __init__(self, transport, **kwargs):
        self.transport = transport
        super().__init__(transport.publish, **kwargs)

    def ack(self):
        return self.transport.acknowledged(self.pending, self.state)

    def status(self):
        status = super().status()
        if self.state is not None:
            status["target"] = [float(v) for v in self.state.target_rad_s]
        return status
