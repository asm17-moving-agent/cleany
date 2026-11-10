from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import mujoco
from rclpy.time import Time


@dataclass(frozen=True)
class MujocoSimulationContext:
    """시뮬레이션 step 관측기와 공유하는 MuJoCo handle입니다.

    소비자는 두 handle을 읽기 전용으로 다뤄야 합니다. 센서 어댑터가 MuJoCo API에
    직접 전달해야 하므로 native MuJoCo 객체로 유지합니다.
    """

    model: mujoco.MjModel
    data: mujoco.MjData


class StepObserver(Protocol):
    """MuJoCo 물리 step 묶음마다 콜백을 한 번 받습니다."""

    def after_step(
        self,
        context: MujocoSimulationContext,
        stamp: Time,
    ) -> None:
        """최신 시뮬레이션 상태를 변경하지 않고 관측합니다."""
        ...
