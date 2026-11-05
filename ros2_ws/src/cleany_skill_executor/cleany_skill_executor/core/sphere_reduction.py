"""Conservative reduction of per-link collision spheres, without deleting coverage."""
from __future__ import annotations

import numpy as np


def reduce_spheres(centers, radii, maximum: int) -> tuple[np.ndarray, np.ndarray]:
    """Merge the cheapest pair until the link budget is met.

    Each replacement ball encloses both input balls. By induction the output
    covers the entire input union, including the interiors, not just centers.
    Additional occupied volume can reject more proposals; final mesh validation
    must still be used. Never merge geometry across different moving links.
    """
    if type(maximum) is not int or maximum < 1:
        raise ValueError('Sphere budget must be a positive integer')
    centers = np.asarray(centers, dtype=float).copy()
    radii = np.asarray(radii, dtype=float).copy()
    if (centers.ndim != 2 or centers.shape[1] != 3 or len(centers) == 0
            or radii.shape != (len(centers),) or not np.isfinite(centers).all()
            or not np.isfinite(radii).all() or np.any(radii <= 0)):
        raise ValueError('Sphere geometry must contain finite centers and positive radii')
    while len(radii) > maximum:
        left, right = np.triu_indices(len(radii), 1)
        delta = centers[right] - centers[left]
        distance = np.linalg.norm(delta, axis=1)
        radius = np.maximum(np.maximum(radii[left], radii[right]),
                            (distance + radii[left] + radii[right]) / 2)
        # Prefer removal of contained/redundant balls, then the least added volume.
        cost = radius ** 3 - radii[left] ** 3 - radii[right] ** 3
        selected = int(np.argmin(cost))
        a, b = int(left[selected]), int(right[selected])
        d = distance[selected]
        if radii[a] >= d + radii[b]:
            center, bound = centers[a], radii[a]
        elif radii[b] >= d + radii[a]:
            center, bound = centers[b], radii[b]
        else:
            bound = radius[selected]
            center = centers[a] + delta[selected] * ((bound - radii[a]) / d)
            # Round outward so repeated merges cannot shave the original union.
            bound = max(np.linalg.norm(center - centers[a]) + radii[a],
                        np.linalg.norm(center - centers[b]) + radii[b])
            bound = np.nextafter(bound, np.inf)
        centers[a], radii[a] = center, bound
        centers, radii = np.delete(centers, b, axis=0), np.delete(radii, b)
    return centers, radii


def sphere_cover_error(original_centers, original_radii, centers, radii) -> float:
    """Worst containment residual; nonpositive means every old ball is enclosed."""
    distances = np.linalg.norm(np.asarray(original_centers)[:, None, :]
                               - np.asarray(centers)[None, :, :], axis=2)
    residual = distances + np.asarray(original_radii)[:, None] - np.asarray(radii)[None, :]
    return float(np.max(np.min(residual, axis=1)))
