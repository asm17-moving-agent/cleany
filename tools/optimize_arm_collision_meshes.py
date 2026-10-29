"""Build reduced collision meshes with per-component geometry certificates.

Enclosure components use supporting halfspaces and include every source triangle.
Their expansion bound is relative to the original component CONVEX HULL: holes
and concavities inside that hull are filled. It is not a CAD surface error bound.
Selected concave components use surface decimation with a two-sided surface
distance bound instead. Original STL assets, visuals and physics are preserved.
Plane partitions retain large openings before enclosing the resulting patches.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import struct
import subprocess

from collision_surface_geometry import directed_bound

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import ConvexHull, HalfspaceIntersection, QhullError


STL_DTYPE = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])


@dataclass(frozen=True)
class Mesh:
    vertices: np.ndarray
    faces: np.ndarray


def read_stl(path: Path) -> Mesh:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f'Truncated binary STL: {path}')
    count = struct.unpack_from('<I', data, 80)[0]
    if not count or len(data) != 84 + 50 * count:
        raise ValueError(f'Expected a nonempty binary STL: {path}')
    triangles = np.frombuffer(data, dtype=STL_DTYPE, offset=84)['vertices'].astype(np.float64)
    if not np.isfinite(triangles).all():
        raise ValueError(f'Nonfinite STL: {path}')
    vertices, indices = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    return Mesh(vertices, indices.reshape(-1, 3))


def components(mesh: Mesh) -> list[Mesh]:
    edges = mesh.faces[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2)
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])),
                       shape=(len(mesh.vertices), len(mesh.vertices)))
    count, labels = connected_components(graph, directed=False)
    result = []
    for label in range(count):
        indices = np.flatnonzero(labels == label)
        mapping = np.full(len(mesh.vertices), -1, dtype=np.int64)
        mapping[indices] = np.arange(len(indices))
        faces = mapping[mesh.faces[labels[mesh.faces[:, 0]] == label]]
        if len(indices) < 4 or (faces < 0).any():
            raise ValueError('Disconnected or degenerate source component')
        result.append(Mesh(mesh.vertices[indices], faces))
    return result


def maximum_plane_violation(points: np.ndarray, equations: np.ndarray) -> float:
    return max(float(np.max(chunk @ equations[:, :3].T + equations[:, 3]))
               for chunk in np.array_split(points, max(1, (len(points) + 511) // 512)))


def clip_surface(mesh: Mesh, axis: int, offset: float, sign: int) -> Mesh:
    """Clip triangles to a halfspace, sharing exact edge intersections.

    Coordinates are metres. The open cut is capped by the later convex hull;
    clipping itself preserves every portion of the original triangle surface.
    """
    if axis not in (0, 1, 2) or sign not in (-1, 1) or not np.isfinite(offset):
        raise ValueError('Invalid partition plane')
    inside = sign * (mesh.vertices[:, axis] - offset) >= 0
    mask = inside[mesh.faces]
    faces = mesh.faces[mask.all(axis=1)].tolist()
    vertices = mesh.vertices.tolist()
    intersections: dict[tuple[int, int], int] = {}
    for triangle in mesh.faces[mask.any(axis=1) & ~mask.all(axis=1)]:
        polygon = []
        for a, b in zip(triangle, np.roll(triangle, -1)):
            if inside[a]:
                polygon.append(int(a))
            if inside[a] != inside[b]:
                edge = tuple(sorted((int(a), int(b))))
                if edge not in intersections:
                    start, end = mesh.vertices[list(edge)]
                    if start[axis] == offset:
                        index = edge[0]
                    elif end[axis] == offset:
                        index = edge[1]
                    else:
                        point = start + (end - start) * ((offset - start[axis]) / (end[axis] - start[axis]))
                        point[axis] = offset
                        index = len(vertices)
                        vertices.append(point.tolist())
                    intersections[edge] = index
                polygon.append(intersections[edge])
        polygon = list(dict.fromkeys(polygon))
        for i in range(1, len(polygon) - 1):
            faces.append([polygon[0], polygon[i], polygon[i + 1]])
    if not faces:
        raise ValueError('Partition produced an empty surface')
    used, inverse = np.unique(faces, return_inverse=True)
    return Mesh(np.asarray(vertices)[used], inverse.reshape(-1, 3))


def partition_surface(mesh: Mesh, planes: list[dict]) -> list[Mesh]:
    """Separate disconnected prongs after each configured cut plane."""
    parts = [mesh]
    for plane in planes:
        axis, offset = plane['axis'], plane['offset_m']
        if type(axis) is not int or axis not in (0, 1, 2) or not np.isfinite(offset):
            raise ValueError('Invalid partition plane')
        divided = []
        for part in parts:
            if not part.vertices[:, axis].min() < offset < part.vertices[:, axis].max():
                divided.append(part)
                continue
            for sign in (-1, 1):
                divided.extend(components(clip_surface(part, axis, offset, sign)))
        parts = divided
    return parts


def source_pieces(source: Mesh, settings: dict, filename: str) -> list[tuple[int, Mesh]]:
    partitions = settings.get('partition_components', {}).get(filename, {})
    return [(index, piece) for index, part in enumerate(components(source))
            for piece in partition_surface(part, partitions.get(str(index), []))]


def radial_bounds(points: np.ndarray, source: ConvexHull, center: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Distance to a known point in the original hull, hence an upper bound.

    Distance to a convex set is convex. Bounding it at every output vertex also
    bounds every point in the output polytope, including its face interiors.
    """
    normals, offsets = source.equations[:, :3], source.equations[:, 3]
    slack = -(normals @ center + offsets)
    delta = points - center
    projection = delta @ normals.T
    ratios = np.full(projection.shape, np.inf)
    np.divide(slack[None, :], projection, out=ratios, where=projection > 0)
    limiting = np.argmin(ratios, axis=1)
    fraction = np.minimum(1.0, ratios[np.arange(len(points)), limiting])
    return np.linalg.norm(delta, axis=1) * (1.0 - fraction), limiting


def enclosing_hull(mesh: Mesh, *, max_expansion: float, margin: float,
                   max_iterations: int, planes_per_iteration: int) -> tuple[Mesh, dict]:
    if not 0 < margin < max_expansion or max_iterations < 1 or planes_per_iteration < 1:
        raise ValueError('Invalid enclosure settings')
    source = ConvexHull(mesh.vertices)
    center = mesh.vertices.mean(axis=0)
    normals = np.vstack((np.eye(3), -np.eye(3)))
    offsets = -(mesh.vertices @ normals.T).max(axis=0) - margin
    planes = np.column_stack((normals, offsets))
    selected: set[int] = set()
    for _ in range(max_iterations):
        vertices = HalfspaceIntersection(planes, center).intersections
        bounds, limiting = radial_bounds(vertices, source, center)
        if bounds.max() <= max_expansion * 0.99:
            break
        candidates = []
        for index in np.argsort(-bounds):
            face = int(limiting[index])
            if bounds[index] <= max_expansion * 0.99:
                break
            if face not in selected:
                selected.add(face)
                candidates.append(face)
            if len(candidates) >= planes_per_iteration:
                break
        if not candidates:
            raise ValueError('Enclosure cannot meet the requested expansion bound')
        additions = source.equations[candidates].copy()
        additions[:, 3] = -(mesh.vertices @ additions[:, :3].T).max(axis=0) - margin
        planes = np.vstack((planes, additions))
    else:
        raise ValueError('Enclosure iteration limit exceeded')

    # Certify the actual float32 coordinates written into STL, not the ideal
    # double-precision intersection. The small outward margin covers rounding.
    vertices = vertices.astype(np.float32).astype(np.float64)
    output = ConvexHull(vertices)
    violation = maximum_plane_violation(mesh.vertices, output.equations)
    bound = float(radial_bounds(vertices, source, center)[0].max())
    if violation > 0 or bound > max_expansion:
        raise ValueError(f'Rounded enclosure failed: {violation=}, {bound=}')
    faces = output.simplices.copy()
    triangles = vertices[faces]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    reverse = np.einsum('ij,ij->i', normals, output.equations[:, :3]) < 0
    faces[reverse] = faces[reverse][:, [0, 2, 1]]
    used, inverse = np.unique(faces, return_inverse=True)
    result = Mesh(vertices[used], inverse.reshape(-1, 3))
    return result, {
        'source_vertices': len(mesh.vertices), 'source_triangles': len(mesh.faces),
        'vertices': len(result.vertices), 'triangles': len(result.faces),
        'maximum_source_plane_violation_m': violation,
        'max_expansion_from_component_hull_m': bound,
        'source_convex_hull_volume_m3': float(source.volume),
        'output_volume_m3': float(output.volume),
    }


def stl_bytes(mesh: Mesh) -> bytes:
    triangles = mesh.vertices[mesh.faces]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    if (lengths <= 0).any():
        raise ValueError('Degenerate output triangle')
    records = np.zeros(len(triangles), dtype=STL_DTYPE)
    records['vertices'] = triangles
    records['normal'] = normals / lengths[:, None]
    return b'Cleany enclosing collision mesh; original CAD preserved'.ljust(80, b' ') + struct.pack('<I', len(records)) + records.tobytes()


def searched_enclosure(mesh: Mesh, settings: dict, variant: str, scale: float) -> tuple[Mesh, dict]:
    """Replay a reviewed seed orientation and optional supporting-plane pruning.

    Keep the source partition unchanged. Certify native STL-unit float32
    coordinates, including containment of complete source triangles. Selected
    variants are config data so a sphere-incompatible patch can keep its legacy
    enclosure without changing the sphere model.
    """
    if variant not in ('xyz', 'xyz_pruned', 'principal_axes', 'principal_axes_pruned'):
        raise ValueError('Unknown enclosure search variant: ' + variant)
    source = ConvexHull(mesh.vertices)
    center = mesh.vertices.mean(0)
    basis = np.eye(3)
    if variant.startswith('principal_axes'):
        centered = mesh.vertices-center
        basis = np.linalg.eigh(centered.T @ centered)[1].T
    normals = np.vstack((basis, -basis))
    planes = np.column_stack((normals, -(mesh.vertices @ normals.T).max(0)-settings['enclosure_margin_m']))
    selected = set()
    limit = settings['max_hull_expansion_m']
    for _ in range(settings['max_iterations']):
        vertices = HalfspaceIntersection(planes, center).intersections
        bounds, limiting = radial_bounds(vertices, source, center)
        if bounds.max() <= limit*.99:
            break
        for index in np.argsort(-bounds):
            face = int(limiting[index])
            if face not in selected:
                selected.add(face)
                plane = source.equations[face].copy()
                plane[3] = -(mesh.vertices @ plane[:3]).max()-settings['enclosure_margin_m']
                planes = np.vstack((planes, plane))
                break
        else:
            raise ValueError('No unused enclosure supporting plane')
    else:
        raise ValueError('Enclosure search iteration limit exceeded')

    def evaluate(halfspaces: np.ndarray, bound_limit: float) -> tuple[Mesh, dict]:
        with np.errstate(divide='raise', invalid='raise'):
            points = HalfspaceIntersection(halfspaces, center).intersections
        if not np.isfinite(points).all():
            raise ValueError('Nonfinite enclosure intersection')
        points = (points/scale).astype(np.float32).astype(float)*scale
        hull = ConvexHull(points)
        violation = maximum_plane_violation(mesh.vertices, hull.equations)
        bound = float(radial_bounds(points, source, center)[0].max())
        if violation > 0 or bound > bound_limit:
            raise ValueError('Serialized enclosure violates source geometry bounds')
        faces = hull.simplices.copy()
        triangles = points[faces]
        normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
        reverse = np.einsum('ij,ij->i', normals, hull.equations[:, :3]) < 0
        faces[reverse] = faces[reverse][:, [0, 2, 1]]
        used, inverse = np.unique(faces, return_inverse=True)
        result = Mesh(points[used], inverse.reshape(-1, 3))
        return result, {'source_vertices': len(mesh.vertices), 'source_triangles': len(mesh.faces),
            'vertices': len(result.vertices), 'triangles': len(result.faces),
            'maximum_source_plane_violation_m': violation,
            'max_expansion_from_component_hull_m': bound,
            'source_convex_hull_volume_m3': float(source.volume), 'output_volume_m3': float(hull.volume),
            'enclosure_variant': variant}

    best, report = evaluate(planes, limit)
    if not variant.endswith('_pruned'):
        return best, report
    best, report = evaluate(planes, limit*.99)
    while True:
        changed = False
        for index in reversed(range(len(planes))):
            if len(planes) <= 4:
                break
            trial = np.delete(planes, index, axis=0)
            try:
                candidate, check = evaluate(trial, limit*.99)
            except (ValueError, QhullError, FloatingPointError):
                continue
            if len(candidate.faces) <= len(best.faces):
                best, report, planes, changed = candidate, check, trial, True
        if not changed:
            return best, report


def surface_decimation(source: Mesh, settings: dict, image: str) -> tuple[Mesh, dict]:
    worker = Path(__file__).with_name('decimate_collision_surface.py').resolve()
    request = {'vertices': source.vertices.tolist(), 'faces': source.faces.tolist(),
               'target_triangles': settings['target_triangles']}
    process = subprocess.run(['docker', 'run', '--rm', '--interactive', '--network', 'none',
        '--pull=never', '--env', 'OMP_NUM_THREADS=1', '--volume', f'{worker}:/worker.py:ro',
        '--entrypoint', 'python3', image, '/worker.py'], input=json.dumps(request),
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
    result = json.loads(process.stdout)
    output = Mesh(np.asarray(result['vertices'], dtype=float), np.asarray(result['faces'], dtype=np.int64))
    def topology(mesh):
        edges = np.sort(mesh.faces[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1)
        unique, counts = np.unique(edges, axis=0, return_counts=True)
        if not np.all(counts == 2) or len(components(mesh)) != 1:
            raise ValueError('Surface decimation changed closed component topology')
        return len(mesh.vertices) - len(unique) + len(mesh.faces)
    if topology(source) != topology(output):
        raise ValueError('Surface decimation changed Euler characteristic')
    limit = settings['max_surface_distance_m']
    certificate = {'source_to_reduced': directed_bound(source, output, limit),
                   'reduced_to_source': directed_bound(output, source, limit)}
    return output, {'method': 'surface_quadric_decimation',
        'source_vertices': len(source.vertices), 'source_triangles': len(source.faces),
        'vertices': len(output.vertices), 'triangles': len(output.faces),
        'open3d_version': result['open3d_version'], 'surface_certificate': certificate}


def optimize(source: Mesh, settings: dict, filename: str = '') -> tuple[Mesh, list[dict]]:
    vertices, faces, reports = [], [], []
    offset = 0
    pieces = source_pieces(source, settings, filename)
    variants = settings.get('enclosure_variants', {}).get(filename, ['current']*len(pieces))
    if len(variants) != len(pieces):
        raise ValueError('Enclosure variants do not match the source partitions')
    for piece_index, (index, part) in enumerate(pieces):
        surface = settings.get('surface_components', {}).get(filename, {}).get(str(index))
        if surface:
            result, report = surface_decimation(part, surface, settings['surface_simplifier_image'])
        elif variants[piece_index] != 'current':
            result, report = searched_enclosure(part, settings, variants[piece_index],
                                                settings.get('mesh_scales', {}).get(filename, 1.))
        else:
            result, report = enclosing_hull(part,
                max_expansion=settings['max_hull_expansion_m'], margin=settings['enclosure_margin_m'],
                max_iterations=settings['max_iterations'], planes_per_iteration=settings['planes_per_iteration'])
        if not surface:
            # Native-unit STL rounding can change nearly coplanar triangles,
            # especially for millimetre CAD. Retriangulate the actual stored
            # vertices so each exported component is a convex solid, not just
            # a point cloud whose convex hull meets the enclosure certificate.
            scale = settings.get('mesh_scales', {}).get(filename, 1.)
            rounded = (result.vertices/scale).astype(np.float32).astype(float)*scale
            hull = ConvexHull(rounded)
            hull_faces = hull.simplices.copy()
            triangles = rounded[hull_faces]
            normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
            reverse = np.einsum('ij,ij->i', normals, hull.equations[:, :3]) < 0
            hull_faces[reverse] = hull_faces[reverse][:, [0, 2, 1]]
            used, inverse = np.unique(hull_faces, return_inverse=True)
            result = Mesh(rounded[used], inverse.reshape(-1, 3))
            violation = maximum_plane_violation(part.vertices, hull.equations)
            bound = float(radial_bounds(result.vertices, ConvexHull(part.vertices), part.vertices.mean(0))[0].max())
            if violation > 0 or bound > settings['max_hull_expansion_m']:
                raise ValueError('Native STL rounding violates enclosure bounds')
            report.update(vertices=len(result.vertices), triangles=len(result.faces),
                          maximum_source_plane_violation_m=violation,
                          max_expansion_from_component_hull_m=bound, output_volume_m3=float(hull.volume))
        vertices.append(result.vertices)
        faces.append(result.faces + offset)
        offset += len(result.vertices)
        report['source_component'] = index
        reports.append(report)
    return Mesh(np.vstack(vertices), np.vstack(faces)), reports


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=root / 'configs/robot/arm_collision_meshes.json')
    parser.add_argument('--output', type=Path, help='Override output directory for a candidate')
    parser.add_argument('--write', action='store_true', help='Write generated assets; default only checks reproducibility')
    args = parser.parse_args()
    settings = json.loads(args.config.read_text())
    package = root / settings['description_package']
    destination = args.output or package / settings['output_directory']
    if destination.resolve() == (package / 'meshes').resolve():
        raise ValueError('Output directory must preserve the original CAD meshes')
    report = {'schema_version': 3, 'method': 'partitioned_component_enclosure_or_surface_decimation',
              'bound_reference': 'Enclosure: convex hull of each original component or clipped surface patch, in metres. Surface decimation: bidirectional original CAD surface distance.',
              'settings': settings, 'meshes': {}}
    pending = []
    for filename in settings['source_meshes']:
        path = package / 'meshes' / filename
        source = read_stl(path)
        scale = settings.get('mesh_scales', {}).get(filename, 1.)
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError('Mesh scale must be positive and finite')
        output, parts = optimize(Mesh(source.vertices * scale, source.faces), settings, filename)
        output = Mesh(output.vertices / scale, output.faces)
        data = stl_bytes(output)
        report['meshes'][filename] = {
            'source_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'output_sha256': hashlib.sha256(data).hexdigest(),
            'source_vertices': len(source.vertices), 'source_triangles': len(source.faces),
            'vertices': len(output.vertices), 'triangles': len(output.faces), 'components': parts,
        }
        pending.append((destination / filename, data))
        print(filename, f'{len(source.vertices)} -> {len(output.vertices)} vertices;',
              f'{len(source.faces)} -> {len(output.faces)} triangles', flush=True)
    pending.append((destination / 'arm_meshes.json', (json.dumps(report, indent=2) + '\n').encode()))
    for path, data in pending:
        if args.write:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
        elif path.read_bytes() != data:
            raise ValueError(f'Generated asset differs: {path}')


if __name__ == '__main__':
    main()
