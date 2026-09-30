from __future__ import annotations

from typing import Protocol, Sequence

from cleany_perception.core.models import (
    Detection2D,
    ObjectMask,
    RgbArray,
    RigidTransform,
)


class DetectorPort(Protocol):
    def detect(
        self,
        rgb: RgbArray,
        query: str,
    ) -> Sequence[Detection2D]:
        """Return validated pixel-space detections for one RGB image."""


class SegmenterPort(Protocol):
    def segment(
        self,
        rgb: RgbArray,
        detections: Sequence[Detection2D],
    ) -> Sequence[ObjectMask]:
        """Return one mask for every detection, preserving input order."""


class TransformPort(Protocol):
    def lookup(
        self,
        target_frame: str,
        source_frame: str,
        stamp_ns: int,
    ) -> RigidTransform:
        """Return target-from-source transform at the capture timestamp."""
