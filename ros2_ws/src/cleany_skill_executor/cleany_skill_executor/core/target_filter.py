"""ROS-independent target screening and ordering for validate_pick."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import re
from typing import Protocol, Sequence


DEFAULT_DENYLIST = (
    'keyboard',
    'cloth',
    'fabric',
    'towel',
    'cable',
    'wire',
    'cord',
)


class DetectionLike(Protocol):
    object_id: int
    label: str
    median_depth_m: float
    estimated_width_m: float
    estimated_height_m: float


class RejectionReason(str, Enum):
    OUTSIDE_WORKSPACE = 'outside_workspace'
    DENYLIST = 'denylist'
    INVALID_DEPTH = 'invalid_depth'
    INVALID_SIZE = 'invalid_size'
    TOO_LARGE = 'too_large'


@dataclass(frozen=True, slots=True)
class TargetDecision:
    object_id: int
    label: str
    eligible: bool
    reason: RejectionReason | None
    detail: str
    depth_m: float | None
    smaller_extent_m: float | None


@dataclass(frozen=True, slots=True)
class TargetSelection:
    selected_index: int | None
    decisions: tuple[TargetDecision, ...]
    eligible_indices: tuple[int, ...] = ()


def _normalized_words(value: str) -> tuple[str, ...]:
    return tuple(re.findall(r'[a-z0-9]+', value.casefold()))


def _denylist_match(label: str, denylist: Sequence[str]) -> str | None:
    padded = f" {' '.join(_normalized_words(label))} "
    for entry in denylist:
        normalized = ' '.join(_normalized_words(entry))
        if normalized and f' {normalized} ' in padded:
            return entry
    return None


def select_closest_eligible(
    detections: Sequence[DetectionLike],
    *,
    denylist: Sequence[str] = DEFAULT_DENYLIST,
    maximum_smaller_extent_m: float = 0.095,
    workspace_roi_pixels: Sequence[float] = (),
) -> TargetSelection:
    """Keep every decision and order eligible targets by depth, then object ID."""
    if not math.isfinite(maximum_smaller_extent_m) or maximum_smaller_extent_m <= 0:
        raise ValueError('maximum_smaller_extent_m must be finite and positive')
    roi = tuple(workspace_roi_pixels)
    if roi and (len(roi) != 4 or not all(math.isfinite(v) for v in roi)
                or not 0 <= roi[0] < roi[2] or not 0 <= roi[1] < roi[3]):
        raise ValueError("workspace_roi_pixels must be empty or [xmin, ymin, xmax, ymax]")
    decisions: list[TargetDecision] = []
    eligible: list[tuple[float, int, int]] = []
    for index, detection in enumerate(detections):
        matched = _denylist_match(str(detection.label), denylist)
        depth = float(detection.median_depth_m)
        width = float(detection.estimated_width_m)
        height = float(detection.estimated_height_m)
        finite_depth = math.isfinite(depth) and depth > 0.0
        finite_size = (
            math.isfinite(width)
            and math.isfinite(height)
            and width > 0.0
            and height > 0.0
        )
        smaller = min(width, height) if finite_size else None
        reason: RejectionReason | None = None
        detail = 'eligible'
        box = tuple(float(getattr(detection, key, math.nan))
                    for key in ("x_min", "y_min", "x_max", "y_max")) if roi else ()
        inside = not roi or (all(math.isfinite(v) for v in box)
                    and roi[0] <= box[0] < box[2] <= roi[2]
                    and roi[1] <= box[1] < box[3] <= roi[3])
        if not inside:
            reason = RejectionReason.OUTSIDE_WORKSPACE
            detail = "bbox is not fully inside configured workspace ROI"
        elif matched is not None:
            reason = RejectionReason.DENYLIST
            detail = f'denylist:{matched}'
        elif not finite_depth:
            reason = RejectionReason.INVALID_DEPTH
            detail = 'no valid central aligned depth'
        elif not finite_size:
            reason = RejectionReason.INVALID_SIZE
            detail = 'support-plane footprint size is unavailable'
        elif smaller is not None and smaller > maximum_smaller_extent_m:
            reason = RejectionReason.TOO_LARGE
            detail = (
                f'smaller footprint extent {smaller:.3f} m exceeds '
                f'{maximum_smaller_extent_m:.3f} m'
            )
        decision = TargetDecision(
            object_id=int(detection.object_id),
            label=str(detection.label),
            eligible=reason is None,
            reason=reason,
            detail=detail,
            depth_m=depth if finite_depth else None,
            smaller_extent_m=smaller,
        )
        decisions.append(decision)
        if decision.eligible:
            eligible.append((depth, int(detection.object_id), index))
    indices = tuple(index for _, _, index in sorted(eligible))
    return TargetSelection(indices[0] if indices else None, tuple(decisions), indices)
