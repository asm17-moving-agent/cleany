"""Conservative robot mesh approximations in metres, independent of ROS/CUDA."""
from __future__ import annotations

import numpy as np
from scipy.spatial import ConvexHull, HalfspaceIntersection


def covering_spheres(vertices, count: int, padding: float, faces=None) -> tuple[np.ndarray, np.ndarray]:
    """Enclose connected mesh components in recursively partitioned cells.

    A cell is clipped through its hull edges before splitting along its longest
    dimension. Each output ball contains all cell vertices, hence its entire
    convex volume. Separate components never acquire a connecting hull. Count
    is a per-mesh target with at least one ball per connected component.
    """
    points = np.asarray(vertices, dtype=np.float64)
    if (points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all()
            or type(count) is not int or count < 1
            or not np.isfinite(padding) or padding < 0):
        raise ValueError('Invalid sphere enclosure settings')
    groups = [points]
    if faces is not None:
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import connected_components

        triangles = np.asarray(faces)
        if (triangles.ndim != 2 or triangles.shape[1] != 3
                or not np.issubdtype(triangles.dtype, np.integer)
                or np.any(triangles < 0) or np.any(triangles >= len(points))):
            raise ValueError('Invalid sphere mesh faces')
        edges = triangles[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2)
        graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])),
                           shape=(len(points), len(points)))
        _, labels = connected_components(graph, directed=False)
        used = np.unique(triangles)
        groups = [points[used[labels[used] == label]] for label in np.unique(labels[used])]
    origin = points.mean(axis=0)
    _, _, basis = np.linalg.svd(points - origin, full_matrices=False)

    def cell(vertices):
        hull = ConvexHull(vertices)
        vertices = vertices[hull.vertices]
        center = (vertices.min(axis=0) + vertices.max(axis=0)) / 2
        radius = np.linalg.norm(vertices - center, axis=1).max()
        return vertices, center, radius

    cells = [cell((group - origin) @ basis.T) for group in groups]
    while len(cells) < count:
        index = max(range(len(cells)), key=lambda i: cells[i][2])
        vertices, _, _ = cells.pop(index)
        axis = int(np.argmax(np.ptp(vertices, axis=0)))
        cut = (vertices[:, axis].min() + vertices[:, axis].max()) / 2
        hull = ConvexHull(vertices)
        edges = np.unique(np.sort(hull.simplices[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1), axis=0)
        start, end = vertices[edges[:, 0]], vertices[edges[:, 1]]
        crossing = (np.minimum(start[:, axis], end[:, axis]) < cut) & (cut < np.maximum(start[:, axis], end[:, axis]))
        start, end = start[crossing], end[crossing]
        delta = end - start
        intersections = start + ((cut - start[:, axis]) / delta[:, axis])[:, None] * delta
        cells.extend((cell(np.vstack((vertices[vertices[:, axis] <= cut], intersections))),
                      cell(np.vstack((vertices[vertices[:, axis] >= cut], intersections)))))
    centers = np.asarray([item[1] for item in cells]) @ basis + origin
    radii = np.asarray([item[2] for item in cells]) + padding + 1e-10
    return centers, radii


def enclosing_polytope(vertices, max_expansion: float) -> np.ndarray:
    """Reduce hull facets with outward support planes and a distance bound.

    The returned polytope contains the input hull. Its maximum expansion is
    certified by distances to known interior points along rays; convexity
    extends the vertex certificate to every output point. Zero disables this
    reduction. Failures raise instead of accepting an unverified enclosure.
    """
    points = np.asarray(vertices, dtype=np.float64)
    if (points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all()
            or not np.isfinite(max_expansion) or max_expansion < 0):
        raise ValueError('Invalid hull enclosure settings')
    source = ConvexHull(points)
    points = points[source.vertices]
    if max_expansion == 0 or len(points) <= 8:
        return points.copy()
    source = ConvexHull(points)
    center = points.mean(axis=0)
    normals = np.vstack((np.eye(3), -np.eye(3)))
    margin = min(1e-7, max_expansion / 100)
    planes = np.column_stack((normals, -(points @ normals.T).max(axis=0) - margin))
    slack = -(source.equations[:, :3] @ center + source.equations[:, 3])
    selected = set()
    for _ in range(len(source.equations) + 1):
        result = HalfspaceIntersection(planes, center).intersections
        delta = result - center
        projection = delta @ source.equations[:, :3].T
        ratios = np.full(projection.shape, np.inf)
        np.divide(slack[None, :], projection, out=ratios, where=projection > 0)
        limiting = np.argmin(ratios, axis=1)
        fraction = np.minimum(1., ratios[np.arange(len(result)), limiting])
        bounds = np.linalg.norm(delta, axis=1) * (1. - fraction)
        worst = int(np.argmax(bounds))
        if bounds[worst] <= max_expansion * .99:
            reduced = ConvexHull(result)
            violation = np.max(points @ reduced.equations[:, :3].T + reduced.equations[:, 3])
            if violation > 1e-12:
                raise ValueError('GPU enclosure lost source geometry')
            # A complex hull can already be cheaper than the enclosure.
            return result[reduced.vertices] if len(reduced.vertices) < len(points) else points.copy()
        face = int(limiting[worst])
        if face in selected:
            raise ValueError('GPU enclosure cannot meet expansion bound')
        selected.add(face)
        addition = source.equations[face].copy()
        addition[3] = -(points @ addition[:3]).max() - margin
        planes = np.vstack((planes, addition))
    raise ValueError('GPU enclosure iteration limit exceeded')
