"""Frame-local camera redaction with fixed YOLOE prompts, YuNet and DB text detection."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from .model_runtime import load_model_profile
from .privacy_text import TextEngine
from .privacy_history import ScreenHistory

# Neutral opaque slate in BGR. A constant fill cannot retain source detail.
FILL = (86, 76, 66)
PERSON_FILL = (127, 124, 100)
CONTEXT = {'robot arm', 'robot gripper', 'wheeled robot', 'box', 'cup', 'monitor stand'}
BODY = {'person', 'hand', 'bare arm'}
# Generic flat objects are also common grasp targets. DB covers their lettering separately.
MIN_CONFIDENCE = {'book': .30, 'piece of paper': .20, 'cell phone': .14,
                  'laptop': .14, 'tablet computer': .14, 'name tag': .12}


@dataclass(frozen=True)
class RedactedFrame:
    image: np.ndarray
    mask: np.ndarray
    counts: dict[str, int]


@dataclass(frozen=True)
class CameraViews:
    preview: RedactedFrame
    archive: RedactedFrame


def cover_box(mask: np.ndarray, box, margin: float) -> None:
    """Cover continuous xyxy coordinates including a margin at the frame edge."""
    x1, y1, x2, y2 = map(float, box)
    if not np.isfinite([x1, y1, x2, y2]).all() or x2 <= x1 or y2 <= y1:
        raise ValueError('Invalid detector bounds')
    dx, dy = max(3, (x2 - x1) * margin), max(3, (y2 - y1) * margin)
    h, w = mask.shape
    x1, x2 = np.clip([np.floor(x1 - dx), np.ceil(x2 + dx)], 0, w).astype(int)
    y1, y2 = np.clip([np.floor(y1 - dy), np.ceil(y2 + dy)], 0, h).astype(int)
    mask[y1:y2, x1:x2] = 255


def cover_person_head(mask: np.ndarray, box, silhouette: np.ndarray | None) -> None:
    """Cover the upper silhouette for side and rear views without a detected face."""
    x1, y1, x2, y2 = box
    height, width = y2 - y1, x2 - x1
    # A wide, horizontal silhouette has no reliable vertical head position.
    depth = height if width > 1.35 * height else min(height, max(.32 * height, .65 * width))
    if silhouette is None or not np.any(silhouette):
        cover_box(mask, (x1, y1, x2, y1 + depth), .12)
        return
    if silhouette.shape != mask.shape:
        raise ValueError('Invalid silhouette size')
    end = int(np.clip(np.ceil(y1 + depth), 0, mask.shape[0]))
    np.copyto(mask[:end], 255, where=silhouette[:end] != 0)


def opaque_frame(bgr: np.ndarray, mask: np.ndarray) -> np.ndarray:
    if mask.shape != bgr.shape[:2] or mask.dtype != np.uint8:
        raise ValueError('Mask does not match its source frame')
    output = bgr.copy()
    output[mask != 0] = FILL
    return output


class CameraRedactor:
    """Redact a single BGR frame. Any failed detector invalidates the entire output."""

    def __init__(self, weights: Path, face: Path, text: Path, profile: Path, *, device: str):
        from ultralytics import YOLO

        for asset in (weights, face, text):
            if not asset.is_file():
                raise FileNotFoundError(asset)
        self.classes = load_model_profile(profile)['yoloe_classes']
        self.history = ScreenHistory()
        self.device = device
        self.segmenter = YOLO(str(weights), task='segment')
        self.face = cv2.FaceDetectorYN.create(str(face), '', (320, 320), 0.6)
        self.text_gpu = TextEngine(text, device) if text.suffix == '.plan' else None
        self.text = None if self.text_gpu else cv2.dnn.readNetFromONNX(str(text))
        self._mean = np.array([.485, .456, .406], np.float32)[None, :, None, None]
        self._std = np.array([.229, .224, .225], np.float32)[None, :, None, None]

    def _objects(self, bgr, mask, heads, counts, stamp) -> np.ndarray:
        result = self.segmenter.predict(
            bgr, device=self.device, conf=.016, imgsz=640, rect=False,
            retina_masks=True, verbose=False, max_det=100,
        )[0]
        if [result.names[i] for i in range(len(result.names))] != self.classes:
            raise ValueError('Privacy engine classes do not match the configured profile')
        boxes = result.boxes.cpu().numpy()
        silhouettes = result.masks.data.cpu().numpy() if result.masks is not None else ()
        if len(boxes) == 100:
            raise ValueError('Privacy detector reached its instance limit')
        body = np.zeros_like(mask)
        surfaces = np.zeros_like(mask)
        for i, (box, category, score) in enumerate(zip(boxes.xyxy, boxes.cls, boxes.conf)):
            name = self.classes[int(category)]
            if name in CONTEXT:
                continue
            threshold = MIN_CONFIDENCE.get(name, .065)
            if name == 'computer monitor' and score < threshold:
                # Inactive displays have weak semantic scores and a large dark panel.
                x1, y1, x2, y2 = np.maximum(box.astype(int), 0)
                patch = bgr[y1:y2, x1:x2]
                if patch.size and np.mean(np.max(patch, axis=2) < 112) > .40:
                    threshold = .016
            if score < threshold:
                continue
            counts[name] = counts.get(name, 0) + 1
            if name == 'face':
                # Include hair and ears around the face detector's tighter box.
                cover_box(heads, box, .45)
            elif name == 'person':
                silhouette = silhouettes[i] if i < len(silhouettes) else None
                cover_person_head(heads, box, silhouette)
            if i < len(silhouettes):
                pixels = silhouettes[i]
                if pixels.shape != mask.shape:
                    raise ValueError('Invalid silhouette size')
                if name in BODY and np.any(pixels):
                    body[pixels != 0] = 255
                    continue
                if name in {'computer monitor', 'laptop', 'tablet computer', 'cell phone'}:
                    contours, _ = cv2.findContours(pixels.astype(np.uint8), cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)
                    if contours:
                        hull = cv2.convexHull(np.concatenate(contours))
                        area = (box[2] - box[0]) * (box[3] - box[1])
                        if cv2.contourArea(hull) >= .55 * area:
                            cv2.fillConvexPoly(surfaces, hull, 255)
                            continue
            if name in {'computer monitor', 'laptop', 'tablet computer', 'cell phone'}:
                cover_box(surfaces, box, .035)
            elif name in BODY or name == 'face':
                cover_box(body, box, .09)
            else:
                cover_box(mask, box, .035)
        radius = max(3, round(min(mask.shape) * .012))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (radius * 2 + 1,) * 2)
        surfaces = self.history.apply(bgr, surfaces, stamp)
        cv2.bitwise_or(mask, cv2.dilate(surfaces, kernel), dst=mask)
        cv2.dilate(heads, kernel, dst=heads)
        return cv2.dilate(body, kernel)

    def _faces(self, view: np.ndarray, mask: np.ndarray) -> int:
        h, w = view.shape[:2]
        self.face.setInputSize((w, h))
        _, faces = self.face.detect(view)
        if faces is None:
            return 0
        scale = np.array([mask.shape[1] / w, mask.shape[0] / h])
        for x, y, width, height in faces[:, :4]:
            center = np.rint((x + width / 2, y + height / 2) * scale).astype(int)
            axes = np.ceil((.8 * width, .82 * height) * scale).astype(int)
            cv2.ellipse(mask, tuple(center), tuple(axes), 0, 0, 360, 255, -1)
        return len(faces)

    def _letters(self, view: np.ndarray, mask: np.ndarray) -> int:
        # DB returns a text probability map. No character recognition or storage.
        h, w = view.shape[:2]
        shape = (max(32, round(w / 32) * 32), max(32, round(h / 32) * 32))
        blob = cv2.dnn.blobFromImage(view, 1 / 255., shape)
        blob = (blob - self._mean) / self._std
        if self.text_gpu:
            scores = self.text_gpu.predict(blob)
        else:
            self.text.setInput(blob)
            scores = self.text.forward()[0, 0]
        if scores.shape != shape[::-1] or not np.isfinite(scores).all():
            raise ValueError('Invalid text detector output')
        contours, _ = cv2.findContours((scores > .20).astype('uint8'), cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        scale = np.array([mask.shape[1] / shape[0], mask.shape[0] / shape[1]])
        centers = []
        for contour in contours:
            x, y, width, height = cv2.boundingRect(contour)
            if width * height >= 5:
                # DB learns shrunken text cores. Restore the outer glyphs in both axes.
                pad = max(3., min(width, height) * .95)
                cover_box(mask, np.array([x-pad, y-pad, x+width+pad, y+height+pad]) * np.tile(scale, 2), 0.)
                centers.append(((x + width / 2) * w / shape[0], (y + height / 2) * h / shape[1]))
        if len(centers) >= 2:
            # Cover a whole outlined card/document when two text rows lie within it.
            edges = cv2.Canny(view, 40, 110)
            borders, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            for border in borders:
                area = cv2.contourArea(border)
                if not .005 * w * h <= area <= .40 * w * h:
                    continue
                quad = cv2.approxPolyDP(border, .025 * cv2.arcLength(border, True), True)
                if len(quad) != 4 or not cv2.isContourConvex(quad):
                    continue
                if sum(cv2.pointPolygonTest(quad, center, False) >= 0 for center in centers) >= 2:
                    points = np.rint(quad * [mask.shape[1] / w, mask.shape[0] / h]).astype(np.int32)
                    cv2.fillConvexPoly(mask, points, 255)
        return len(centers)

    def redact(self, bgr: np.ndarray, stamp: float | None = None) -> RedactedFrame | None:
        """Return the full-body policy used by existing snapshot and archive consumers."""
        views = self.redact_views(bgr, stamp)
        return views.archive if views is not None else None

    def redact_views(self, bgr: np.ndarray, stamp: float | None = None) -> CameraViews | None:
        """Produce both outputs from the same detections; withhold both during warm-up."""
        if bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3 or not bgr.size:
            raise ValueError('Expected an 8-bit BGR frame')
        mask = np.zeros(bgr.shape[:2], np.uint8)
        counts: dict[str, int] = {}
        emit = stamp is None or self.history.continuous(stamp)
        heads = np.zeros_like(mask)
        people = self._objects(bgr, mask, heads, counts, stamp)
        ratio = min(1., 640 / max(mask.shape))
        view = cv2.resize(bgr, None, fx=ratio, fy=ratio, interpolation=cv2.INTER_AREA) if ratio < 1 else bgr
        counts['face_yunet'] = self._faces(view, heads)
        counts['text_region'] = self._letters(view, mask)
        if not emit:
            return None
        preview = opaque_frame(bgr, mask)
        preview[heads != 0] = PERSON_FILL
        preview_mask = mask | heads
        archive = preview.copy()
        archive[people != 0] = PERSON_FILL
        return CameraViews(RedactedFrame(preview, preview_mask, counts),
                           RedactedFrame(archive, preview_mask | people, counts))
