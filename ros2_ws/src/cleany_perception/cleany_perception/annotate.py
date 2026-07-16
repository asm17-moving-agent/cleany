"""Draw detection overlays for visual debugging.

Optional visualization only: takes an RGB image and Detection candidates and
returns a copy with boxes + labels (and, when present, semi-transparent
instance masks) drawn. Kept out of the detection core so it never affects the
published Detection2DArray (AGENTS.md sections 3-4).

cv2 is imported lazily so importing this module (and the detection pipeline)
never requires OpenCV unless annotation is actually turned on.
"""

from __future__ import annotations

from typing import Iterable

import numpy as np

from cleany_perception.detector import Detection

_BOX_COLOR = (0, 255, 0)  # green in RGB channel order
_THICKNESS = 2
_MASK_ALPHA = 0.4


def draw_detections(image: np.ndarray, detections: Iterable[Detection]) -> np.ndarray:
    """Return a copy of the RGB image with each detection's box + label drawn."""
    import cv2

    canvas = np.ascontiguousarray(image, dtype=np.uint8).copy()
    for det in detections:
        if det.mask is not None and det.mask.shape == canvas.shape[:2]:
            # Force bool: an int 0/1 mask would silently become (wrong) fancy
            # indexing instead of a pixel selection.
            mask = np.asarray(det.mask, dtype=bool)
            region = canvas[mask].astype(np.float32)
            canvas[mask] = (
                (1.0 - _MASK_ALPHA) * region + _MASK_ALPHA * np.array(_BOX_COLOR)
            ).astype(np.uint8)
        p1 = (int(det.x1), int(det.y1))
        p2 = (int(det.x2), int(det.y2))
        cv2.rectangle(canvas, p1, p2, _BOX_COLOR, _THICKNESS)
        label = f'{det.label} {det.score:.2f}'
        cv2.putText(
            canvas, label, (p1[0], max(0, p1[1] - 5)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, _BOX_COLOR, 1, cv2.LINE_AA,
        )
    return canvas
