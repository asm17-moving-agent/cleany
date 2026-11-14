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
        """RGB 영상 하나의 검증된 픽셀 공간 검출을 반환합니다."""


class SegmenterPort(Protocol):
    def segment(
        self,
        rgb: RgbArray,
        detections: Sequence[Detection2D],
    ) -> Sequence[ObjectMask]:
        """입력 순서를 유지하며 검출마다 mask 하나를 반환합니다."""


class ReferenceTrackerPort(Protocol):
    def track(self, reference_rgb: RgbArray, detection: Detection2D,
              current_rgb: RgbArray) -> MaskArray:
        """Track one original identity; no new semantic or mask-IoU confidence."""


class TransformPort(Protocol):
    def lookup(
        self,
        target_frame: str,
        source_frame: str,
        stamp_ns: int,
    ) -> RigidTransform:
        """촬영 시각의 target-from-source 변환을 반환합니다."""
