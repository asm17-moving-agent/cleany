"""Image-space stability and duplicate-view gates; no robot control."""
from dataclasses import dataclass, field

import cv2
import numpy as np


def view_signature(found: dict, board) -> np.ndarray:
    """Map a fixed set of board points to pixels, even under partial occlusion.

    A homography is only a viewpoint heuristic here, not a calibrated pose.
    """
    source = board.chessboardCorners[found['ids'].flatten(), :2]
    target = found['corners'].reshape(-1, 2)
    matrix, _ = cv2.findHomography(source, target, 0)
    if matrix is None:
        raise ValueError('No viewpoint homography')
    signature = cv2.perspectiveTransform(
        board.chessboardCorners[:, :2].copy().reshape(-1, 1, 2), matrix).reshape(-1, 2)
    if not np.isfinite(signature).all():
        raise ValueError('Invalid viewpoint signature')
    return signature


@dataclass
class AutoCaptureGate:
    hold_seconds: float
    stable_rms_px: float
    novel_rms_px: float
    max_gap_seconds: float
    saved: list[np.ndarray] = field(default_factory=list)
    anchor: np.ndarray | None = None
    since: float = 0.0
    last_time: float | None = None

    @staticmethod
    def distance(a: np.ndarray, b: np.ndarray) -> float:
        return float(np.sqrt(np.mean(np.sum((a - b) ** 2, axis=1))))

    def update(self, signature: np.ndarray | None, now: float) -> tuple[bool, str]:
        interrupted = self.last_time is not None and (
            now - self.last_time > self.max_gap_seconds or now < self.last_time)
        self.last_time = now
        if signature is None:
            self.anchor = None
            return False, 'REPOSITION: need spread-out board corners'
        if interrupted or self.anchor is None or self.distance(signature, self.anchor) > self.stable_rms_px:
            self.anchor = signature.copy()
            self.since = now
        if any(self.distance(signature, old) < self.novel_rms_px for old in self.saved):
            return False, 'CHANGE VIEW: this viewpoint was already saved'
        remaining = self.hold_seconds - (now - self.since)
        if remaining > 0:
            return False, f'HOLD STILL: {remaining:.1f}s'
        return True, 'AUTO SAVING'

    def commit(self, signature: np.ndarray) -> None:
        self.saved.append(signature.copy())
        self.anchor = None
