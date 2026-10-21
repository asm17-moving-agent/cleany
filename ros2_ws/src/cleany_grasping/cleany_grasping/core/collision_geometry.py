"""Observed convex footprint extruded to a perceived support plane.

This encloses the supplied observations, not unknown hidden surfaces. It is
not an object mesh recovered from the simulator and does not model cavities.
"""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ConvexPrism:
    vertices: np.ndarray
    triangles: np.ndarray


def observed_convex_prism(points: np.ndarray, center: np.ndarray,
                          rotation: np.ndarray, height: float,
                          maximum_vertices: int = 512) -> ConvexPrism:
    """Return vertices in the OBB's local frame and outward-wound faces."""
    points, center, rotation = map(lambda x: np.asarray(x, dtype=float), (points, center, rotation))
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 3
            or center.shape != (3,) or rotation.shape != (3, 3)
            or not all(np.isfinite(x).all() for x in (points, center, rotation))
            or not np.isfinite(height) or height <= 0 or maximum_vertices < 6
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
            or abs(np.linalg.det(rotation)-1) > 1e-5):
        raise ValueError('Invalid observed collision geometry inputs')
    local = (points-center) @ rotation
    planar = sorted(set(map(tuple, local[:, :2])))
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    def chain(values):
        result = []
        for point in values:
            while len(result) >= 2 and cross(result[-2], result[-1], point) <= 0:
                result.pop()
            result.append(point)
        return result
    hull = chain(planar)[:-1] + chain(reversed(planar))[:-1]
    if len(hull) < 3 or 2*len(hull) > maximum_vertices:
        raise ValueError('Degenerate or oversized observed collision footprint')
    bottom, top = min(-height/2, float(local[:, 2].min())), max(height/2, float(local[:, 2].max()))
    area = sum(a[0]*b[1]-a[1]*b[0] for a, b in zip(hull, hull[1:]+hull[:1])) / 2
    if area <= 1e-10:
        raise ValueError('Degenerate observed collision footprint area')
    count = len(hull)
    vertices = np.array([(x, y, z) for z in (bottom, top) for x, y in hull])
    triangles = []
    for index in range(1, count-1):
        triangles.extend(((0, index+1, index), (count, count+index, count+index+1)))
    for index in range(count):
        following = (index+1) % count
        triangles.extend(((index, following, count+following),
                          (index, count+following, count+index)))
    return ConvexPrism(vertices, np.array(triangles, dtype=int))
