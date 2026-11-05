"""Derive a solid torso with its existing top opening from CAD triangles."""

from collections import Counter, defaultdict

import numpy as np
from optimize_arm_collision_meshes import Mesh


def largest_top_opening(mesh: Mesh) -> np.ndarray:
    """Extract the largest rectangular hole in a horizontal top face."""
    top = mesh.vertices[:, 2].max()
    faces = mesh.faces[np.all(mesh.vertices[mesh.faces, 2] == top, axis=1)]
    edges = Counter(
        tuple(sorted(edge)) for face in faces for edge in zip(face, np.roll(face, -1))
    )
    neighbors = defaultdict(list)
    for (a, b), count in edges.items():
        if count == 1:
            neighbors[a].append(b)
            neighbors[b].append(a)
        elif count != 2:
            raise ValueError("The top face has nonmanifold edges")
    if not neighbors or any(len(values) != 2 for values in neighbors.values()):
        raise ValueError("The top face needs closed boundary loops")
    unused = set(neighbors)
    loops = []
    while unused:
        start = current = min(unused)
        previous = None
        loop = []
        while True:
            loop.append(current)
            unused.remove(current)
            following = next(v for v in neighbors[current] if v != previous)
            previous, current = current, following
            if current == start:
                break
            if current not in unused:
                raise ValueError("The top boundary intersects itself")
        xy = mesh.vertices[loop, :2]
        area = (
            abs(
                float(
                    np.sum(
                        xy[:, 0] * np.roll(xy[:, 1], -1)
                        - xy[:, 1] * np.roll(xy[:, 0], -1)
                    )
                )
            )
            / 2
        )
        loops.append((area, xy))
    if len(loops) < 2:
        raise ValueError("The CAD top face has no opening")
    # The outer loop has the largest area; the next is the access opening.
    _, xy = sorted(loops, key=lambda entry: -entry[0])[1]
    bounds = np.stack((xy.min(axis=0), xy.max(axis=0)))
    edge = np.roll(xy, -1, axis=0) - xy
    area = (
        abs(
            float(
                np.sum(
                    xy[:, 0] * np.roll(xy[:, 1], -1) - xy[:, 1] * np.roll(xy[:, 0], -1)
                )
            )
        )
        / 2
    )
    if not np.all(np.any(edge == 0, axis=1)) or not np.isclose(
        area, np.prod(bounds[1] - bounds[0]), rtol=1e-12, atol=1e-16
    ):
        raise ValueError("The largest CAD opening must be rectangular")
    return bounds


def _intersects_open_pocket(
    triangles: np.ndarray, lower: np.ndarray, upper: np.ndarray
) -> bool:
    """Clip candidate triangles, rejecting surfaces crossing the pocket interior."""
    guard = 64 * np.finfo(float).eps * max(1.0, float(np.abs(triangles).max()))
    candidates = triangles[
        np.all(triangles.max(axis=1) > lower + guard, axis=1)
        & np.all(triangles.min(axis=1) < upper - guard, axis=1)
    ]
    for triangle in candidates:
        polygon = list(triangle)
        for axis in range(3):
            for bound, sign in ((lower[axis], 1), (upper[axis], -1)):
                clipped = []
                for a, b in zip(polygon, polygon[1:] + polygon[:1]):
                    inside_a = sign * (a[axis] - bound) >= 0
                    inside_b = sign * (b[axis] - bound) >= 0
                    if inside_a:
                        clipped.append(a)
                    if inside_a != inside_b:
                        point = a + (b - a) * ((bound - a[axis]) / (b[axis] - a[axis]))
                        point[axis] = bound
                        clipped.append(point)
                polygon = clipped
                if not polygon:
                    break
            if not polygon:
                break
        if polygon:
            center = np.mean(polygon, axis=0)
            if np.all(center > lower + guard) and np.all(center < upper - guard):
                return True
    return False


def body_boxes(
    meshes: list[Mesh], opening_meshes: list[Mesh], floor_meshes: list[Mesh]
) -> tuple[list[np.ndarray], dict]:
    """Cover the torso with five boxes, leaving the CAD opening empty above a rail.

    The floor height is the top of the named existing CAD rail. It is a collision
    enclosure boundary, not an inferred physical bin bottom or usable bin depth.
    """
    points = np.vstack([mesh.vertices for mesh in meshes])
    lower, upper = points.min(axis=0), points.max(axis=0)
    openings = np.asarray([largest_top_opening(mesh) for mesh in opening_meshes])
    opening = np.stack((openings[:, 0].max(axis=0), openings[:, 1].min(axis=0)))
    floor_z = max(float(mesh.vertices[:, 2].max()) for mesh in floor_meshes)
    cavity_lower = np.r_[opening[0], floor_z]
    cavity_upper = np.r_[opening[1], upper[2]]
    if (
        np.any(cavity_lower <= lower)
        or np.any(cavity_upper[:2] >= upper[:2])
        or np.any(cavity_upper <= cavity_lower)
    ):
        raise ValueError("The CAD opening and floor must lie inside the torso")
    # Extend the test above the top so a lid crossing the opening is rejected too.
    check_upper = cavity_upper.copy()
    check_upper[2] = np.nextafter(upper[2] + np.max(upper - lower), np.inf)
    for mesh in meshes:
        if _intersects_open_pocket(
            mesh.vertices[mesh.faces], cavity_lower, check_upper
        ):
            raise ValueError("Original CAD geometry crosses the requested body opening")
    x0, y0, z0 = lower
    x1, y1, z1 = upper
    xa, ya, za = cavity_lower
    xb, yb, _ = cavity_upper
    boxes = [
        np.array([[x0, y0, z0], [x1, y1, za]]),
        np.array([[x0, y0, za], [xa, y1, z1]]),
        np.array([[xb, y0, za], [x1, y1, z1]]),
        np.array([[xa, y0, za], [xb, ya, z1]]),
        np.array([[xa, yb, za], [xb, y1, z1]]),
    ]
    return boxes, {
        "bounds_m": [lower.tolist(), upper.tolist()],
        "opening_bounds_xy_m": opening.tolist(),
        "pocket_bounds_m": [cavity_lower.tolist(), cavity_upper.tolist()],
        "source_triangles": sum(len(mesh.faces) for mesh in meshes),
        "source_pocket_intersection": False,
    }
