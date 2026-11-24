"""Upper-surface measurements for shallow parallel-jaw contacts."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from cleany_grasping.core.models import PointCloud


@dataclass(frozen=True)
class FaceContact:
    center: np.ndarray
    face_width_m: float
    opening_m: float


@dataclass(frozen=True)
class TopFace:
    points: np.ndarray
    normal: np.ndarray
    origin: np.ndarray
    tolerance_m: float

    @classmethod
    def fit(cls, body: PointCloud, context: PointCloud, up: np.ndarray, tolerance_m: float) -> 'TopFace | None':
        """Accept a broad, populated upper plane rather than a tangent to a curved body."""
        points = body.points
        if not tolerance_m or len(points) < 24:
            return None
        heights = points @ up
        top = np.percentile(heights, 90)
        # Start at an upper quantile so a sparse set of flying pixels cannot set the plane.
        level = np.percentile(heights, 75)
        selected = np.abs(heights - level) <= tolerance_m
        for _ in range(3):
            if np.count_nonzero(selected) < max(18, .30 * len(points)):
                return None
            origin = points[selected].mean(axis=0)
            _, values, basis = np.linalg.svd(points[selected] - origin, full_matrices=False)
            normal = basis[-1] * np.sign(basis[-1] @ up)
            if normal @ up < .94 or values[1] < 3 * tolerance_m * np.sqrt(selected.sum()):
                return None
            distances = (points - origin) @ normal
            selected = np.abs(distances) <= tolerance_m
        if (np.count_nonzero(selected) < max(18, .30 * len(points))
                or np.percentile(distances, 90) > 2 * tolerance_m
                or top - level > 6 * tolerance_m):
            return None
        # A curved crown can approximate a plane in a narrow band. Require the
        # observed flat patch to span a majority of the body in both tangents.
        for tangent in basis[:2]:
            full = np.ptp(np.percentile(points @ tangent, (5, 95)))
            patch = np.ptp(np.percentile(points[selected] @ tangent, (5, 95)))
            if patch < .65 * full:
                return None
        surface = points[selected]
        if context.viewpoint is not None:
            surface = _extend_occluding_edge(surface, body.colors[selected], normal, origin,
                                             context, tolerance_m)
        return cls(surface, normal, origin, tolerance_m)

    def contact(self, target: np.ndarray, site: np.ndarray, rotation: np.ndarray,
                half_span_m: float, pad_offset_m: float, bite_m: float,
                tip_reach_m: float, margin_m: float, trim: float) -> FaceContact | None:
        """Measure the local face and clear the occupied volume above the lowest fingertip."""
        approach, closing, lateral = rotation.T
        incidence = -float(approach @ self.normal)
        if incidence <= .5:
            return None
        face_site = site - approach * ((site - self.origin) @ self.normal) / (approach @ self.normal)
        relative = self.points - face_site
        local = relative[np.abs(relative @ lateral) <= half_span_m + 1e-12]
        if len(local) < 8:
            return None
        low, high = np.percentile(local @ closing, (trim, 100 - trim))
        face_width = float(high - low)
        across = local @ closing
        interior = np.abs(across - (low + high)/2) < face_width/4
        if (face_width <= 2 * self.tolerance_m or low >= 0 or high <= 0
                or not np.any(interior)):
            return None
        # Both opposing contacts must lie in the same shallow bite.
        if abs(float(closing @ self.normal)) * face_width > 2 * (bite_m + self.tolerance_m):
            return None
        tangent = closing - self.normal * (closing @ self.normal)
        middle = face_site + tangent * ((low + high) / (2 * (tangent @ closing)))
        center = middle + approach * (bite_m / incidence - pad_offset_m)
        offsets = target - center
        # Include the entire finger span swept on entry. A wider lip or wall
        # can require more clearance than the width measured on the upper face.
        swept = offsets[(np.abs(offsets @ lateral) <= half_span_m + 1e-12)
                        & (offsets @ approach <= tip_reach_m)]
        if not len(swept):
            return None
        clearance = 2 * float(np.max(np.abs(swept @ closing)))
        return FaceContact(center, face_width, max(face_width, clearance) + margin_m)

def _extend_occluding_edge(surface: np.ndarray, colors: np.ndarray, normal: np.ndarray,
                           origin: np.ndarray, context: PointCloud, tolerance: float) -> np.ndarray:
    """Intersect far-rim pixel rays with the face where depth drops but face colour continues."""
    view = context.viewpoint
    assert view is not None
    height = float((view - origin) @ normal)
    if height <= tolerance:
        return surface
    rays = context.points - view
    drop = -rays @ normal
    usable = drop > height + tolerance
    if not np.any(usable):
        return surface
    projected = view + rays[usable] * (height / drop[usable])[:, None]
    tree = cKDTree(surface)
    spacing = float(np.median(tree.query(surface, k=6)[0][:, -1])) / 2
    if spacing <= 0:
        return surface
    distances, nearest = tree.query(projected, k=3)
    dist = distances[:, 0]
    delta = projected - surface[nearest[:, 0]]
    away = origin - view
    away -= (away @ normal) * normal
    if np.linalg.norm(away) <= tolerance:
        return surface
    facing_away = delta @ (away / np.linalg.norm(away)) > .25 * dist
    nearby = (dist > spacing / 2) & (dist <= 4 * spacing) & facing_away
    local_color = np.median(colors[nearest], axis=1)
    difference = np.max(np.abs(context.colors[usable] - local_color), axis=1)
    # Without foreground/background contrast the depth gap cannot identify the rim.
    background = (dist > 4 * spacing) & (dist <= 8 * spacing) & facing_away
    if not background.any():
        return surface
    contrast = float(np.median(difference[background]))
    if contrast < 12 / 255:
        return surface
    edge = projected[nearby & (difference <= min(12 / 255, .40 * contrast))]
    if len(edge) < 3:
        return surface
    # Isolated colour matches on the table cannot extend a measured edge.
    supported = cKDTree(edge).query(edge, k=3)[0][:, -1] <= 2 * spacing
    return np.vstack((surface, edge[supported]))
