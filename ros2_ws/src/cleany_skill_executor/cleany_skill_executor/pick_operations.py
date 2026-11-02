"""Checkpoint data shared by legacy sorting and the single-object BT backend."""
from dataclasses import dataclass
from typing import Any


@dataclass
class GraspProgress:
    approach_start: Any = None
    approach_start_joints: Any = None
    gripper_start: float = 0.0
    close_position: float = 0.0
    confirmed: bool = False
    count_retreat: bool = False
    contact_tcp_z: float = 0.0
    minimum_object_z: float | None = None
