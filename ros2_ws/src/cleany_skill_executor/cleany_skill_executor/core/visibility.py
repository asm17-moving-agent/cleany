"""Camera-to-observed-OBB envelope for pregrasp self-occlusion checks."""
from __future__ import annotations

from dataclasses import dataclass
from itertools import product
import math

from cleany_skill_executor.core.grasp_selection import quaternion_axis


@dataclass(frozen=True, slots=True)
class VisibilityCone:
    camera: tuple[float, float, float]
    target: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    radius_m: float
    sides: int


def enclosing_visibility_cone(
    camera: tuple[float, float, float],
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
    *,
    padding_m: float = 0.003,
    sides: int = 16,
) -> VisibilityCone:
    """Enclose all eight OBB corner rays, without object ground truth.

    The disc is perpendicular to the camera-center ray. Perspective projection
    accounts for the near corners appearing larger. Circumscribing the polygon
    keeps its flat sides outside the requested circular envelope.
    """
    if (len(camera) != 3 or len(center) != 3 or len(size) != 3 or len(orientation) != 4
            or not all(math.isfinite(v) for v in (*camera, *center, *size, *orientation, padding_m))
            or min(size) <= 0. or padding_m < 0. or not 3 <= sides <= 128
            or not math.isclose(sum(v*v for v in orientation), 1., abs_tol=1e-4)):
        raise ValueError('visibility geometry must be finite, positive and normalized')
    normal = tuple(a-b for a, b in zip(camera, center))
    distance = math.sqrt(sum(v*v for v in normal))
    if distance <= 1e-6:
        raise ValueError('camera must be outside the observed target')
    normal = tuple(v/distance for v in normal)
    axes = [quaternion_axis(orientation, axis)
            for axis in ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))]
    radius = 0.
    for signs in product((-1., 1.), repeat=3):
        vertex = tuple(center[k] + sum(signs[i]*size[i]*axes[i][k]/2. for i in range(3))
                       for k in range(3))
        ray = tuple(v-c for v, c in zip(vertex, camera))
        depth = -sum(v*n for v, n in zip(ray, normal))
        if depth <= 1e-6:
            raise ValueError('observed target crosses the camera plane')
        projected = tuple(c+v*distance/depth-t for c, v, t in zip(camera, ray, center))
        radius = max(radius, math.sqrt(sum(v*v for v in projected)))
    # Quaternion rotating local +Z to the disc normal toward the camera.
    if normal[2] < -1. + 1e-12:
        rotation = (1., 0., 0., 0.)
    else:
        raw = (-normal[1], normal[0], 0., 1.+normal[2])
        norm = math.sqrt(sum(v*v for v in raw))
        rotation = tuple(v/norm for v in raw)
    return VisibilityCone(camera, center, rotation,
                          (radius+padding_m)/math.cos(math.pi/sides), sides)
