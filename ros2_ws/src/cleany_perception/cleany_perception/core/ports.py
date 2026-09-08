from __future__ import annotations

from typing import Protocol, Sequence

from cleany_perception.core.models import (
    Detection2D,
    MaskArray,
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


class ReferenceTrackerPort(Protocol):
    def track(self, reference_rgb: RgbArray, detection: Detection2D,
              current_rgb: RgbArray) -> MaskArray:
        """Track one original identity; no new semantic or mask-IoU confidence."""


class SequenceReferenceTrackerPort(ReferenceTrackerPort, Protocol):
    def track_sequence(self, frames: Sequence[RgbArray], detection: Detection2D,
                       *, reference_mask: MaskArray | None = None) -> MaskArray:
        """Track through ordered sensor frames, returning the final frame mask."""


class StreamingSessionPort(Protocol):
    def track(self, rgb: RgbArray) -> MaskArray:
        """Append one selected sensor frame while retaining target memory."""

    def close(self) -> None:
        """Release this target's inference state."""


class StreamingReferenceTrackerPort(SequenceReferenceTrackerPort, Protocol):
    def start_stream(self, rgb: RgbArray, mask: MaskArray,
                     *, memory_frames: int = 32, cpu_threads: int = 4) -> StreamingSessionPort:
        """Seed a new target exactly once; never reseed from subsequent masks."""


class TransformPort(Protocol):
    def lookup(
        self,
        target_frame: str,
        source_frame: str,
        stamp_ns: int,
    ) -> RigidTransform:
        """Return target-from-source transform at the capture timestamp."""
