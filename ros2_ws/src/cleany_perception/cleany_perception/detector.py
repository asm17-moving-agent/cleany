"""Pure object-detection logic for Cleany perception.

ROS-independent detection core so it can be unit tested without a running ROS
graph (AGENTS.md section 4). It takes an image (ndarray) and returns detection
candidates; it does not decide robot behaviour (that is the Planner's job).

- `Detection`: a single detection candidate (label, score, pixel bbox, and an
  optional full-frame instance mask when a segmentation model is used).
- `parse_boxes()`: pure conversion of raw box/mask arrays into `Detection`s;
  unit tested without ultralytics.
- `YoloDetector`: wraps an ultralytics YOLO model. `ultralytics` is imported
  lazily so this module (and `parse_boxes`) imports without it installed.

Model choice (YOLO11 seg, pretrained COCO) is an experiment-branch decision and
is not confirmed in the KB. Weights/conf/classes are configurable, never
hardcoded as project truth (AGENTS.md section 4).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Mapping, Sequence

import numpy as np


@dataclass(frozen=True)
class Detection:
    """One detection candidate. Bbox is pixel corners (x1,y1)-(x2,y2).

    `mask` is a full-frame boolean instance mask (same HxW as the source
    image) when the model provides segmentation, else None. It is excluded
    from equality so Detection comparisons stay well-defined (ndarray `==`
    is elementwise).
    """

    label: str
    score: float
    x1: float
    y1: float
    x2: float
    y2: float
    mask: np.ndarray | None = field(default=None, compare=False)


def parse_boxes(
    boxes_xyxy: Iterable[Sequence[float]],
    scores: Iterable[float],
    class_ids: Iterable[float],
    class_names: Mapping[int, str] | Sequence[str],
    masks: Iterable[np.ndarray] | None = None,
) -> list[Detection]:
    """Turn raw model outputs into Detection candidates (pure).

    `class_names` maps class id -> label (ultralytics `model.names` is a dict).
    Unknown ids fall back to their stringified id rather than raising, so an
    unexpected class never crashes perception.

    `masks`, when given, must align 1:1 with the boxes; each entry becomes the
    Detection's boolean instance mask. None keeps every mask empty (detection-
    only models).
    """
    mask_list = list(masks) if masks is not None else None
    detections: list[Detection] = []
    for i, ((x1, y1, x2, y2), score, class_id) in enumerate(
        zip(boxes_xyxy, scores, class_ids)
    ):
        cid = int(class_id)
        try:
            label = class_names[cid]
        except (KeyError, IndexError):
            label = str(cid)
        mask = None
        if mask_list is not None and i < len(mask_list):
            mask = np.asarray(mask_list[i], dtype=bool)
        detections.append(
            Detection(
                label=str(label),
                score=float(score),
                x1=float(x1),
                y1=float(y1),
                x2=float(x2),
                y2=float(y2),
                mask=mask,
            )
        )
    return detections


class YoloDetector:
    """ultralytics YOLO wrapper. Loads the model lazily on first detect()."""

    def __init__(
        self,
        weights: str = 'yolo11n-seg.pt',
        conf: float = 0.25,
        classes: Sequence[int] | None = None,
        device: str = '',
    ) -> None:
        self._weights = weights
        self._conf = conf
        self._classes = list(classes) if classes is not None else None
        # device: '' lets ultralytics auto-pick (CUDA if usable, else CPU);
        # 'cpu', 'cuda:0'/'0', or 'mps' force a specific target. Kept
        # configurable so the same code runs on any host (AGENTS.md section 4).
        self._device = device
        self._model = None

    def load(self) -> None:
        """Load the model now (optional warm-up so the first detect() is fast)."""
        self._ensure_model()

    def _ensure_model(self) -> None:
        if self._model is None:
            from ultralytics import YOLO

            self._model = YOLO(self._weights)

    def detect(self, image) -> list[Detection]:
        """Run detection on an RGB ndarray and return Detection candidates."""
        self._ensure_model()
        predict_kwargs = {
            'source': image,
            'conf': self._conf,
            'classes': self._classes,
            'verbose': False,
            # Segmentation masks at the source image resolution instead of the
            # model's padded input size, so masks align with bbox pixels and
            # the depth image without extra rescaling. Ignored by detect-only
            # models.
            'retina_masks': True,
        }
        if self._device != '':
            predict_kwargs['device'] = self._device
        results = self._model.predict(**predict_kwargs)
        if not results:
            return []
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return []
        xyxy = boxes.xyxy.cpu().numpy()
        scores = boxes.conf.cpu().numpy()
        class_ids = boxes.cls.cpu().numpy()
        masks = None
        if results[0].masks is not None:
            masks = results[0].masks.data.cpu().numpy() > 0.5
        return parse_boxes(xyxy, scores, class_ids, self._model.names, masks)
