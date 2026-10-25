"""RGB-only scale/orientation fallback for the public, untuned head detector."""
from dataclasses import replace
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image

from cleany_perception.core.models import BoundingBox2D


class HeadDetector:
    """Retry empty full frames on four quadrants at four right-angle rotations.

    No target pose or simulation state enters this adapter. A fallback mask must
    be wholly inside its tile. Boxes and masks return to original sensor pixels
    before RGB-D reconstruction and semantic classification. Nonempty original
    detections are preserved; this bounded fallback does not guarantee recall.
    """

    def __init__(self, detector, evidence: Path | None = None):
        self.detector = detector
        self.evidence = evidence
        if evidence is not None:
            evidence.mkdir(parents=True, exist_ok=True)

    def prepare(self):
        self.detector.prepare()

    def detect(self, rgb, query):
        if self.evidence is None:
            return self._detect(rgb, query)
        identifier = str(time.time_ns())
        Image.fromarray(rgb).save(self.evidence/f'{identifier}.png')
        started = time.monotonic()
        report = {'query': query, 'input_shape': list(rgb.shape)}
        try:
            detections = self._detect(rgb, query)
            report['detections'] = [dict(label=d.label, confidence=d.confidence,
                bbox=vars(d.bbox), mask_pixels=int(d.segmentation_mask.sum())
                if d.segmentation_mask is not None else None) for d in detections]
            return detections
        except Exception as error:
            report['error'] = repr(error)
            raise
        finally:
            report['elapsed_s'] = time.monotonic()-started
            (self.evidence/f'{identifier}.json').write_text(json.dumps(report, indent=2))

    def _detect(self, rgb, query):
        detections = self.detector.detect(rgb, query)
        if detections:
            return detections
        height, width = rgb.shape[:2]
        restored = []
        for y0, y1 in ((0, height//2), (height//2, height)):
            for x0, x1 in ((0, width//2), (width//2, width)):
                tile = rgb[y0:y1, x0:x1]
                h, w = tile.shape[:2]
                if not h or not w:
                    continue
                for turns in range(4):
                    for detection in self.detector.detect(
                            np.ascontiguousarray(np.rot90(tile, turns)), query):
                        mask = detection.segmentation_mask
                        if mask is None:
                            continue
                        mask = np.rot90(mask, -turns)
                        if mask.shape != (h, w):
                            raise ValueError('Head fallback mask differs from sensor tile')
                        if not mask.any() or any(edge.any() for edge in (
                                mask[0], mask[-1], mask[:, 0], mask[:, -1])):
                            continue
                        b = detection.bbox
                        if turns == 0:
                            coords = b.x_min, b.y_min, b.x_max, b.y_max
                        elif turns == 1:
                            coords = w-b.y_max, b.x_min, w-b.y_min, b.x_max
                        elif turns == 2:
                            coords = w-b.x_max, h-b.y_max, w-b.x_min, h-b.y_min
                        else:
                            coords = b.y_min, h-b.x_max, b.y_max, h-b.x_min
                        full_mask = np.zeros((height, width), dtype=bool)
                        full_mask[y0:y1, x0:x1] = mask
                        full_mask.flags.writeable = False
                        a, b, c, d = coords
                        restored.append(replace(detection,
                            bbox=BoundingBox2D(a+x0, b+y0, c+x0, d+y0),
                            segmentation_mask=full_mask))
        # Rotated passes can rediscover one instance. Keep its original score;
        # never inflate confidence by counting repeated image transformations.
        kept = []
        for candidate in sorted(restored, key=lambda item: -item.confidence):
            if any(candidate.label == other.label and _mask_iou(
                    candidate.segmentation_mask, other.segmentation_mask) >= .5
                    for other in kept):
                continue
            kept.append(candidate)
        return tuple(kept[:10])


def _mask_iou(first, second):
    return np.count_nonzero(first & second) / np.count_nonzero(first | second)
