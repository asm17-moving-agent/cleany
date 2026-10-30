"""Reuse a successful measured FK result for an identical URDF chain state."""
from __future__ import annotations

from collections.abc import Sequence
import math
import xml.etree.ElementTree as ET

from .cartesian import CartesianPose
from .urdf_fk import UrdfChain


class MeasuredFkCache:
    """Keep one result; fresh hardware checks remain the caller's responsibility.

    Only exact finite positions of joints between the requested frame and link
    identify a pose. Unsupported chains bypass caching rather than guessing
    dependencies. Model, frame, link and communication recovery changes discard
    the entry. This stores geometry, never collision or arrival decisions.
    """

    def __init__(self) -> None:
        self._context: tuple[str, str, str, int] | None = None
        self._names: tuple[str, ...] | None = None
        self._positions: tuple[float, ...] | None = None
        self._pose: CartesianPose | None = None

    def key(self, description: str, frame: str, link: str, names: Sequence[str],
            positions: Sequence[float], recovery: int) -> tuple | None:
        context = (description, frame, link, recovery)
        if context != self._context:
            self._context = context
            self._names, self._positions, self._pose = None, None, None
            try:
                chain = UrdfChain(description, frame, link)
                self._names = tuple(j.name for j in chain.joints if j.kind != 'fixed')
            except (ValueError, ET.ParseError, AttributeError):
                return None
        if (self._names is None or len(names) != len(positions)
                or len(set(names)) != len(names)):
            self._positions, self._pose = None, None
            return None
        values = dict(zip(names, positions))
        if any(name not in values or not math.isfinite(values[name]) for name in self._names):
            self._positions, self._pose = None, None
            return None
        joint_positions = tuple(float(values[name]) for name in self._names)
        if joint_positions != self._positions:
            self._positions, self._pose = joint_positions, None
        return context, joint_positions

    def get(self, key: tuple | None) -> CartesianPose | None:
        if key is not None and key == (self._context, self._positions):
            return self._pose
        return None

    def put(self, key: tuple | None, pose: CartesianPose) -> None:
        if key is not None and key == (self._context, self._positions):
            self._pose = pose
