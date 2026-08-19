from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import mujoco
from rclpy.time import Time


@dataclass(frozen=True)
class MujocoSimulationContext:
    """MuJoCo handles shared with simulation step observers.

    Consumers must treat both handles as read-only.  They remain native MuJoCo
    objects because sensor adapters need to pass them to MuJoCo APIs directly.
    """

    model: mujoco.MjModel
    data: mujoco.MjData


class StepObserver(Protocol):
    """Receives one callback after each group of MuJoCo physics steps."""

    def after_step(
        self,
        context: MujocoSimulationContext,
        stamp: Time,
    ) -> None:
        """Observe the latest simulation state without mutating it."""
        ...
