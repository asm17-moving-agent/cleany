"""Bridge brief display detection gaps after measuring the camera motion between frames."""
import cv2
import numpy as np


class ScreenHistory:
    def __init__(self, hold_seconds=.48):
        self.hold_seconds = hold_seconds
        self.clear()

    def clear(self):
        self.previous = None
        self.remaining = None
        self.stamp = None

    def continuous(self, stamp):
        return self.stamp is not None and 0 < stamp - self.stamp < self.hold_seconds

    @staticmethod
    def _motion(previous, current):
        points = cv2.goodFeaturesToTrack(previous, 160, .015, 5)
        if points is None or len(points) < 8:
            return None
        moved, status, _ = cv2.calcOpticalFlowPyrLK(previous, current, points, None)
        if moved is None:
            return None
        ok = status.ravel() != 0
        if ok.sum() < 8:
            return None
        transform, inliers = cv2.estimateAffinePartial2D(points[ok], moved[ok],
                                                        ransacReprojThreshold=2.)
        if transform is None or inliers.sum() < max(8, .6 * ok.sum()):
            return None
        # Reject cuts and large camera movements even when a few corners match.
        if not .85 < np.linalg.norm(transform[0, :2]) < 1.15:
            return None
        aligned = cv2.warpAffine(previous, transform, current.shape[::-1],
                                 borderMode=cv2.BORDER_REPLICATE)
        if np.mean(cv2.absdiff(aligned, current)) > 25:
            return None
        return transform

    def apply(self, bgr, surfaces, stamp):
        if stamp is None:
            self.previous = self.stamp = self.remaining = None
            return surfaces
        h, w = surfaces.shape
        reduction = min(1., 320 / max(h, w))
        size = (max(1, round(w * reduction)), max(1, round(h * reduction)))
        small = cv2.resize(bgr, size, interpolation=cv2.INTER_AREA)
        grey = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        observed = cv2.resize(surfaces, size, interpolation=cv2.INTER_AREA) != 0
        remaining = np.zeros(grey.shape, np.float32)
        if (self.previous is not None and self.previous.shape == grey.shape
                and self.continuous(stamp)):
            delta = stamp - self.stamp
            motion = self._motion(self.previous, grey)
            if motion is not None:
                remaining = cv2.warpAffine(self.remaining, motion, size, flags=cv2.INTER_NEAREST)
                np.maximum(remaining - delta, 0, out=remaining)
        remaining[observed] = self.hold_seconds
        self.previous, self.remaining, self.stamp = grey, remaining, stamp
        held = cv2.resize((remaining > 0).astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST)
        return surfaces | (held * 255)
