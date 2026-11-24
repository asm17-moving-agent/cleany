"""Estimate local closing-axis spans from observed circular cross sections."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class RoundSection:
    center: np.ndarray
    normal: np.ndarray
    radius_m: float
    half_height_m: float

    def closing_span(self, contact: np.ndarray, closing: np.ndarray) -> float | None:
        """Return the chord in meters when both ends lie in the fitted slab."""
        offset = contact - self.center
        height = float(offset @ self.normal)
        slope = float(closing @ self.normal)
        radial = offset - height * self.normal
        direction = closing - slope * self.normal
        a = float(direction @ direction)
        radial_squared = float(radial @ radial)
        radius_squared = self.radius_m ** 2
        if a <= 1e-9 or radial_squared >= radius_squared:
            return None
        b = float(radial @ direction)
        discriminant = b * b + a * (radius_squared - radial_squared)
        half_chord = math.sqrt(discriminant) / a
        midpoint = -b / a
        if abs(height + slope * midpoint) + abs(slope) * half_chord > self.half_height_m:
            return None
        return 2.0 * half_chord


def fit_round_section(
    points: np.ndarray, center: np.ndarray, normal: np.ndarray,
    tangent_axes: np.ndarray, half_height_m: float, tolerance_m: float,
) -> RoundSection | None:
    """Fit a circle in a contact-height slab with enough observed arc support."""
    relative = points - center
    band = relative[np.abs(relative @ normal) <= half_height_m] @ tangent_axes
    if len(band) < 12:
        return None
    mean = band.mean(axis=0)
    xy = band - mean
    coefficients, _, rank, _ = np.linalg.lstsq(
        np.column_stack((2.0 * xy, np.ones(len(xy)))), np.sum(xy * xy, axis=1), rcond=None)
    offset = coefficients[:2]
    radius_sq = float(coefficients[2] + offset @ offset)
    if rank != 3 or radius_sq <= 0.0:
        return None
    radius = math.sqrt(radius_sq)
    radial = xy - offset
    errors = np.abs(np.linalg.norm(radial, axis=1) - radius)
    if np.percentile(errors, 90.0) > tolerance_m:
        return None
    # A small arc can fit many circles with very different hidden widths.
    observed = radial[errors <= tolerance_m]
    angles = np.sort(np.arctan2(observed[:, 1], observed[:, 0]))
    largest_gap = np.max(np.diff(np.append(angles, angles[0] + math.tau)))
    if math.tau - largest_gap < math.pi / 2.0:
        return None
    return RoundSection(center + tangent_axes @ (mean + offset), normal, radius, half_height_m)
