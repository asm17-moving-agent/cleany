"""Reduce CAD inspection meshes with a bidirectional surface-distance check.

The bound concerns triangle surfaces, not conservative solid containment. No
collision padding or hardware calibration is inferred from it.
"""

from __future__ import annotations

from collections import Counter

import numpy as np
from optimize_arm_collision_meshes import Mesh


def polydata(mesh: Mesh):
    import vtk
    from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray

    points = vtk.vtkPoints()
    points.SetData(numpy_to_vtk(np.ascontiguousarray(mesh.vertices), deep=True))
    cells = vtk.vtkCellArray()
    cells.SetData(
        numpy_to_vtkIdTypeArray(
            np.arange(len(mesh.faces) + 1, dtype=np.int64) * 3, deep=True
        ),
        numpy_to_vtkIdTypeArray(
            np.asarray(mesh.faces, dtype=np.int64).ravel(), deep=True
        ),
    )
    result = vtk.vtkPolyData()
    result.SetPoints(points)
    result.SetPolys(cells)
    return result


def _point_triangle_distance(points: np.ndarray, triangles: np.ndarray) -> np.ndarray:
    """Return distances to paired triangles, including degenerate input faces."""
    a, b, c = triangles.transpose(1, 0, 2)
    ab, ac, ap = b - a, c - a, points - a
    normal = np.cross(ab, ac)
    length2 = np.einsum("ij,ij->i", normal, normal)
    offset = np.einsum("ij,ij->i", ap, normal)
    factor = np.divide(offset, length2, out=np.zeros_like(offset), where=length2 > 0)
    projected = points - factor[:, None] * normal
    inside = length2 > 0
    distances = []
    for start, end in ((a, b), (b, c), (c, a)):
        edge = end - start
        delta = points - start
        denominator = np.einsum("ij,ij->i", edge, edge)
        numerator = np.einsum("ij,ij->i", delta, edge)
        t = np.divide(
            numerator, denominator, out=np.zeros_like(numerator), where=denominator > 0
        )
        nearest = start + np.clip(t, 0, 1)[:, None] * edge
        distances.append(np.linalg.norm(points - nearest, axis=1))
        inside &= np.einsum("ij,ij->i", np.cross(edge, projected - start), normal) >= 0
    plane_distance = np.linalg.norm(points - projected, axis=1)
    return np.where(inside, plane_distance, np.min(distances, axis=0))


def surface_bound(
    source: Mesh, target: Mesh, limit_m: float, *, max_cells: int = 2_000_000
) -> dict:
    """Bound the distance of every source triangle point to the target surface.

    Distance to one closed triangle is convex. The largest distance of the
    three source vertices to that triangle bounds their complete triangle.
    Unproven regions are bisected; a work limit fails closed. This is not a
    vertex-only or random-sample Hausdorff estimate. A numerical guard is added
    to the computed distance before accepting a region.
    """
    import vtk

    if not np.isfinite(limit_m) or limit_m <= 0:
        raise ValueError("Surface bound must be positive and finite")
    target_poly = polydata(target)
    locator = vtk.vtkStaticCellLocator()
    locator.SetDataSet(target_poly)
    locator.BuildLocator()
    target_triangles = target.vertices[target.faces]
    pending = [source.vertices[source.faces]]
    closest, cell, sub, distance2 = (
        [0.0] * 3,
        vtk.reference(0),
        vtk.reference(0),
        vtk.reference(0.0),
    )
    guard = 1e-10 * max(
        1.0, float(np.abs(target.vertices).max()), float(np.abs(source.vertices).max())
    )
    checked, maximum = 0, 0.0
    while pending:
        batch = pending.pop()
        if len(batch) > 2048:
            pending.extend(np.array_split(batch, (len(batch) + 2047) // 2048))
            continue
        checked += len(batch)
        if checked > max_cells:
            raise ValueError("Surface certificate work limit exceeded")
        centers = batch.mean(axis=1)
        ids = []
        for point in centers:
            locator.FindClosestPoint(point, closest, cell, sub, distance2)
            if float(distance2) > limit_m**2:
                raise ValueError("Surface deviation exceeds the requested bound")
            ids.append(int(cell))
        selected = target_triangles[ids]
        bounds = (
            np.max(
                [
                    _point_triangle_distance(batch[:, index], selected)
                    for index in range(3)
                ],
                axis=0,
            )
            + guard
        )
        accepted = bounds <= limit_m
        if accepted.any():
            maximum = max(maximum, float(bounds[accepted].max()))
        remaining = batch[~accepted]
        if not len(remaining):
            continue
        # Bisect the longest edge; both children exactly cover the parent.
        lengths = np.linalg.norm(remaining - np.roll(remaining, -1, axis=1), axis=2)
        for edge in range(3):
            triangles = remaining[np.argmax(lengths, axis=1) == edge]
            if not len(triangles):
                continue
            a, b, c = np.roll(triangles, -edge, axis=1).transpose(1, 0, 2)
            middle = (a + b) * 0.5
            if np.any(np.all(middle == a, axis=1) | np.all(middle == b, axis=1)):
                raise ValueError("Surface certificate reached numerical resolution")
            pending.extend(
                (np.stack((a, middle, c), axis=1), np.stack((middle, b, c), axis=1))
            )
    return {"upper_bound_m": maximum, "checked_subtriangles": checked}


def _topology(faces: np.ndarray) -> tuple[int, Counter]:
    edges = np.sort(faces[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1)
    unique, counts = np.unique(edges, axis=0, return_counts=True)
    abnormal = Counter(
        {tuple(edge): int(count) for edge, count in zip(unique, counts) if count != 2}
    )
    euler = len(np.unique(faces)) - len(unique) + len(faces)
    return euler, abnormal


def _face_key(face: np.ndarray) -> tuple:
    a, b, c = (int(value) for value in face)
    return (a, b, c) if a <= b and a <= c else ((b, c, a) if b <= c else (c, a, b))


def _reduce_patch(
    source: Mesh, max_surface_error_m: float, proposal_error: float
) -> tuple[Mesh, dict]:
    import vtk
    from vtk.util.numpy_support import vtk_to_numpy

    if not np.isfinite(max_surface_error_m) or not 0 < max_surface_error_m <= 0.001:
        raise ValueError("Inspection surface error must be in (0, 0.001] metres")
    decimator = vtk.vtkDecimatePro()
    decimator.SetInputData(polydata(source))
    decimator.SetTargetReduction(0.9)
    decimator.PreserveTopologyOn()
    decimator.SplittingOff()
    decimator.BoundaryVertexDeletionOff()
    decimator.AccumulateErrorOn()
    decimator.SetErrorIsAbsolute(1)
    # VTK's proposal metric is not treated as a surface-distance certificate.
    decimator.SetAbsoluteError(proposal_error)
    decimator.Update()
    output = decimator.GetOutput()
    vertices = vtk_to_numpy(output.GetPoints().GetData()).copy()
    faces = vtk_to_numpy(output.GetPolys().GetConnectivityArray()).copy().reshape(-1, 3)
    # Check the float32 values that GLB will store, not only VTK's in-memory output.
    vertices = vertices.astype(np.float32).astype(np.float64)
    candidate = Mesh(vertices, faces)
    report = {
        "source_triangles": len(source.faces),
        "triangles": len(source.faces),
        "reduced": False,
    }
    if len(faces) >= len(source.faces):
        return source, report
    try:
        original_ids = {
            tuple(vertex): index for index, vertex in enumerate(source.vertices)
        }
        mapped = np.array(
            [original_ids[tuple(vertex)] for vertex in vertices], dtype=np.int64
        )
        mapped_faces = mapped[faces]
        if _topology(source.faces) != _topology(mapped_faces):
            raise ValueError(
                "Reduction changes Euler characteristic or boundary/nonmanifold edges"
            )
        original_keys = {_face_key(face) for face in source.faces}
        reduced_keys = {_face_key(face) for face in mapped_faces}
        changed_source = np.array(
            [face for face in source.faces if _face_key(face) not in reduced_keys],
            dtype=np.int64,
        ).reshape(-1, 3)
        changed_target = np.array(
            [face for face in mapped_faces if _face_key(face) not in original_keys],
            dtype=np.int64,
        ).reshape(-1, 3)
        forward = surface_bound(
            Mesh(source.vertices, changed_source), candidate, max_surface_error_m
        )
        backward = surface_bound(
            Mesh(source.vertices, changed_target), source, max_surface_error_m
        )
    except (ValueError, KeyError) as error:
        report["fallback_reason"] = str(error)
        return source, report
    report.update(
        reduced=True,
        triangles=len(faces),
        source_to_reduced=forward,
        reduced_to_source=backward,
        boundary_and_nonmanifold_edges_preserved=True,
    )
    return candidate, report


def reduce_mesh(source: Mesh, max_surface_error_m: float) -> tuple[Mesh, dict]:
    """Retry stricter proposals and retain the source if none can be certified."""
    if not np.isfinite(max_surface_error_m) or not 0 < max_surface_error_m <= 0.001:
        raise ValueError("Inspection surface error must be in (0, 0.001] metres")
    attempts = []
    for divisor in (10_000, 1_000_000, 100_000_000, 10_000_000_000):
        error = max_surface_error_m / divisor
        output, report = _reduce_patch(source, max_surface_error_m, error)
        attempts.append(
            {
                "proposal_error": error,
                "result": report.get("fallback_reason", "accepted"),
            }
        )
        if "fallback_reason" not in report:
            report["attempts"] = attempts
            return output, report
    report["attempts"] = attempts
    return source, report
