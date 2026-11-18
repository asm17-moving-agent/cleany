"""Separate a well-observed grasp body from sparse, detached mask fragments."""
from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial import cKDTree

from cleany_grasping.core.models import PointCloud


@dataclass(frozen=True)
class GraspTarget(PointCloud):
    """Observed body for prediction, plus detached returns kept for collision checks."""

    detached: PointCloud | None = None

    def obstacle_context(self, context: PointCloud) -> PointCloud:
        """Add detached returns omitted by the context sampler, without reweighting existing points."""
        if self.detached is None:
            return context
        points = self.detached.points
        distances, _ = cKDTree(context.points).query(points)
        missing = distances > 0.0
        if not missing.any():
            return context
        return PointCloud(np.vstack((context.points, points[missing])),
                          np.vstack((context.colors, self.detached.colors[missing])))


@dataclass(frozen=True)
class BodyFilter:
    # The diagonal of the collection profile's 5 mm voxel is 8.7 mm.
    radius_m: float = 0.009
    minimum_fraction: float = 0.90

    def __post_init__(self) -> None:
        if not math.isfinite(self.radius_m) or self.radius_m < 0:
            raise ValueError('Body connectivity radius must be finite and nonnegative')
        if not 0.5 < self.minimum_fraction <= 1.0:
            raise ValueError('Minimum body fraction must be in (0.5, 1]')

    def target(self, cloud: PointCloud) -> GraspTarget:
        if isinstance(cloud, GraspTarget):
            return cloud
        body = dominant_body_mask(cloud.points, self.radius_m, self.minimum_fraction)
        if body.all():
            return GraspTarget(cloud.points, cloud.colors)
        detached = PointCloud(cloud.points[~body], cloud.colors[~body])
        return GraspTarget(cloud.points[body], cloud.colors[body], detached)


def dominant_body_mask(points: np.ndarray, radius_m: float, minimum_fraction: float) -> np.ndarray:
    """Keep the original mask unless one connected body is sufficiently dominant and isolated.

    Components use 3-D Euclidean distances. A gap of at least two sampling
    radii is required so a single missing depth sample cannot split a jaw span.
    """
    whole = np.ones(len(points), dtype=bool)
    if radius_m == 0 or len(points) < 4:
        return whole

    tree = cKDTree(points)
    labels = np.full(len(points), -1, dtype=np.intp)
    counts = []
    for start in range(len(points)):
        if labels[start] >= 0:
            continue
        label = len(counts)
        labels[start] = label
        pending = [start]
        size = 1
        while pending and size < len(points):
            neighbors = np.asarray(tree.query_ball_point(points[pending.pop()], radius_m))
            fresh = neighbors[labels[neighbors] < 0]
            labels[fresh] = label
            pending.extend(fresh.tolist())
            size += len(fresh)
        if size == len(points):
            return whole
        counts.append(size)

    largest = int(np.argmax(counts))
    if counts[largest] < max(3, minimum_fraction * len(points)):
        return whole
    body = labels == largest
    gap, _ = cKDTree(points[body]).query(points[~body], k=1)
    return body if gap.min() >= 2 * radius_m else whole
