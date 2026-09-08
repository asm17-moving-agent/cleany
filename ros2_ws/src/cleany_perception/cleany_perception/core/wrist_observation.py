"""RGB-only projection/association guards. Never infer metric depth from RGB."""
from dataclasses import dataclass, fields
from itertools import product
import numpy as np

from cleany_perception.core.models import BoundingBox2D, RigidTransform


@dataclass(frozen=True)
class WristObservationConfig:
    capture_timeout_seconds: float = 3.0
    maximum_frame_age_seconds: float = 0.75
    maximum_head_prior_age_seconds: float = 60.0
    reference_ttl_seconds: float = 120.0
    minimum_detection_confidence: float = 0.25
    minimum_segmentation_score: float = 0.8
    minimum_detection_iou: float = 0.2
    near_plane_m: float = 0.005
    minimum_box_edge_pixels: float = 8.0
    minimum_visible_fraction: float = 0.6
    minimum_mask_pixels: int = 100
    maximum_mask_fraction: float = 0.7
    minimum_mask_in_box_fraction: float = 0.65
    history_period_seconds: float = 2.0
    history_maximum_frames: int = 32
    tracking_support_frames: int = 4
    streaming_memory_frames: int = 32
    streaming_cpu_threads: int = 4
    tracking_check_timeout_seconds: float = 12.0
    maximum_tracking_result_age_seconds: float = 6.0

    def __post_init__(self) -> None:
        for field in fields(self):
            value = getattr(self, field.name)
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'Invalid wrist limit: {field.name}')
            if any(part in field.name for part in ('confidence', 'score', 'iou', 'fraction')) and value > 1:
                raise ValueError(f'Wrist fraction exceeds one: {field.name}')
        if not 1 <= self.tracking_support_frames <= 14 or self.history_maximum_frames > 64:
            raise ValueError('Wrist frame history exceeds bounded inference limits')
        if not 16 <= self.streaming_memory_frames <= 64:
            raise ValueError('Wrist streaming memory must be 16..64 frames')
        if not isinstance(self.streaming_cpu_threads, int) or not 1 <= self.streaming_cpu_threads <= 8:
            raise ValueError('Wrist streaming CPU threads must be 1..8')


def project_box(center, rotation, size, transform: RigidTransform, k, width: int, height: int,
                config: WristObservationConfig = WristObservationConfig()) -> BoundingBox2D:
    center, rotation, size, k = map(np.asarray, (center, rotation, size, k))
    if (center.shape != (3,) or rotation.shape != (3, 3) or size.shape != (3,)
            or k.shape != (3, 3) or np.any(size <= 0) or width <= 0 or height <= 0
            or not all(np.isfinite(a).all() for a in (center, rotation, size, k))):
        raise ValueError('Invalid expected object geometry')
    if k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('Invalid wrist camera focal length')
    corners = np.asarray(list(product((-0.5, 0.5), repeat=3))) * size
    points = (corners @ rotation.T + center) @ transform.rotation.T + transform.translation
    if not np.isfinite(points).all() or np.any(points[:, 2] <= config.near_plane_m):
        raise ValueError('Expected object crosses wrist camera near plane')
    pixels = points @ k.T
    pixels = pixels[:, :2] / pixels[:, 2:]
    low, high = pixels.min(axis=0), pixels.max(axis=0)
    clipped_low, clipped_high = np.maximum(low, 0), np.minimum(high, (width-1, height-1))
    if (np.any(clipped_high-clipped_low < config.minimum_box_edge_pixels)
            or np.prod(clipped_high-clipped_low)/np.prod(high-low) < config.minimum_visible_fraction):
        raise ValueError('Expected object is outside wrist image')
    return BoundingBox2D(*clipped_low, *clipped_high)


def bbox_iou(a: BoundingBox2D, b: BoundingBox2D) -> float:
    intersection = max(0., min(a.x_max,b.x_max)-max(a.x_min,b.x_min)) * max(
        0., min(a.y_max,b.y_max)-max(a.y_min,b.y_min))
    area = lambda x: (x.x_max-x.x_min)*(x.y_max-x.y_min)
    return intersection / (area(a)+area(b)-intersection)


def verify_mask(mask, box: BoundingBox2D,
                config: WristObservationConfig = WristObservationConfig()) -> None:
    mask = np.asarray(mask)
    if mask.ndim != 2 or mask.dtype != np.bool_:
        raise ValueError('Invalid wrist mask')
    y, x = np.nonzero(mask)
    if len(x) < config.minimum_mask_pixels or len(x) > mask.size * config.maximum_mask_fraction:
        raise ValueError('Wrist target mask has insufficient or excessive area')
    inside = (x >= box.x_min) & (x <= box.x_max) & (y >= box.y_min) & (y <= box.y_max)
    if inside.mean() < config.minimum_mask_in_box_fraction:
        raise ValueError('Wrist mask is inconsistent with expected object projection')
