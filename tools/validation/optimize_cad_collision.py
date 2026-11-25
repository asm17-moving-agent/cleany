#!/usr/bin/env python3
"""Generate an offline CAD collision candidate with bounded geometry changes."""
from __future__ import annotations

import argparse
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import sys
from tempfile import NamedTemporaryFile
import xml.etree.ElementTree as ET

import fast_simplification
import numpy as np
from scipy.spatial import ConvexHull, QhullError, cKDTree
from scipy.spatial.transform import Rotation
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from collision_bores import close_bores
from collision_surface_geometry import TriangleIndex, directed_bound, distances
from meshopt_decimation import simplify as meshopt_simplify
from optimize_arm_collision_meshes import Mesh, components, enclosing_hull, maximum_plane_violation, read_stl


PACKAGE = 'package://cleany_description/'
Sphere = tuple[float, float, float, float]


@dataclass(frozen=True)
class MeshInput:
    path: Path
    destination: Path
    enclosure: bool
    surface_limit: float
    solid_limit: float
    bore_radius: float
    spheres: tuple[Sphere, ...] = ()
    minimum_triangle_fraction: float = .25
    meshoptimizer_library: Path | None = None


def read_mesh(path: Path) -> Mesh:
    if path.suffix == '.stl':
        return read_stl(path)
    geometry = trimesh.load(path, process=False)
    if isinstance(geometry, trimesh.Scene):
        geometry = geometry.to_geometry()
    return Mesh(np.asarray(geometry.vertices, dtype=float), np.asarray(geometry.faces, dtype=np.int64))


def clean(mesh: Mesh) -> Mesh:
    """Remove duplicate positions, duplicate triangles and zero-area faces."""
    points, indexes = np.unique(mesh.vertices, axis=0, return_inverse=True)
    faces = indexes[mesh.faces]
    a, b, c = points[faces].transpose(1, 0, 2)
    faces = faces[np.linalg.norm(np.cross(b-a, c-a), axis=1) > 0.]
    _, unique = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    faces = faces[np.sort(unique)]
    used, mapped = np.unique(faces, return_inverse=True)
    return Mesh(points[used], mapped.reshape(-1, 3))


def topology(mesh: Mesh) -> tuple[int, int, int]:
    edges = np.sort(mesh.faces[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1)
    unique, counts = np.unique(edges, axis=0, return_counts=True)
    return (len(mesh.vertices)-len(unique)+len(mesh.faces), int(np.sum(counts == 1)), int(np.sum(counts > 2)))


def oriented_hull(points: np.ndarray) -> Mesh:
    hull = ConvexHull(points)
    faces = hull.simplices.copy()
    a, b, c = points[faces].transpose(1, 0, 2)
    reverse = np.einsum('ij,ij->i', np.cross(b-a, c-a), hull.equations[:, :3]) < 0
    faces[reverse] = faces[reverse][:, [0, 2, 1]]
    used, inverse = np.unique(faces, return_inverse=True)
    return Mesh(points[used], inverse.reshape(-1, 3))


def useful(source: Mesh, candidate: Mesh) -> bool:
    return (len(candidate.faces) < len(source.faces) and len(candidate.vertices) <= len(source.vertices)
            and len(candidate.faces) >= 4 and len(candidate.vertices) >= 4)


def reject_sample_mismatch(source: Mesh, candidate: Mesh, limit: float) -> None:
    """Quickly reject large changes before certifying both complete surfaces."""
    samples = np.vstack((source.vertices, source.vertices[source.faces].mean(axis=1)))
    triangles = candidate.vertices[candidate.faces]
    tree = cKDTree(triangles.mean(axis=1))
    exact = None
    for start in range(0, len(samples), 2048):
        points = samples[start:start+2048]
        _, index = tree.query(points, k=min(12, len(triangles)))
        upper = distances(points, triangles[np.asarray(index).reshape(len(points), -1)])
        uncertain = upper > limit
        if uncertain.any():
            if exact is None:
                exact = TriangleIndex(triangles)
            if np.any(exact.query(points[uncertain], upper[uncertain]) > limit):
                raise ValueError('Sample exceeds the surface distance limit')


def certificate(source: Mesh, candidate: Mesh, limit: float) -> dict:
    reject_sample_mismatch(source, candidate, limit)
    reject_sample_mismatch(candidate, source, limit)
    return {'input_to_output': directed_bound(source, candidate, limit, max_depth=16),
            'output_to_input': directed_bound(candidate, source, limit, max_depth=16)}


def project_surface_vertices(points: np.ndarray, spheres: tuple[Sphere, ...]) -> np.ndarray:
    """Keep new surface vertices in the previously fitted approximation."""
    if not spheres:
        return points
    balls = np.asarray(spheres)
    delta = points[:, None, :]-balls[None, :, :3]
    radius = np.linalg.norm(delta, axis=-1)
    nearest = (radius-balls[:, 3]).argmin(axis=1)
    rows = np.arange(len(points))
    distance = radius[rows, nearest]
    outside = distance > balls[nearest, 3]
    owners = nearest[outside]
    inset = np.minimum(balls[owners, 3]*.0001, 1e-7)
    projected = points.copy()
    projected[outside] = balls[owners, :3] + delta[rows[outside], owners] * (
        (balls[owners, 3]-inset)/distance[outside])[:, None]
    return projected


def decimate(source: Mesh, limit: float, spheres: tuple[Sphere, ...],
             minimum_triangle_fraction: float, meshoptimizer_library: Path | None = None) -> tuple[Mesh, dict]:
    """Remove surface detail by proximity bounds, retaining components and connectivity."""
    best, details = source, {'method': 'unchanged'}
    euler, border, non_manifold = topology(source)
    if non_manifold or len(source.faces) < 20:
        return best, details
    if not border:
        try:
            convex = oriented_hull(source.vertices)
            directions = ConvexHull(convex.vertices).equations
            if useful(best, convex) and maximum_plane_violation(source.vertices, directions) <= 1e-12:
                check = certificate(source, convex, limit)
                best, details = convex, {'method': 'bounded_convex_surface', 'certificate': check,
                                         'closed_handle_count': max(0, (2-euler)//2)}
        except (QhullError, ValueError):
            pass
    size = float(np.ptp(source.vertices, axis=0).max())
    center = source.vertices.mean(axis=0)
    if size <= 0:
        return best, details
    points = np.ascontiguousarray((source.vertices-center)/size)
    faces = np.ascontiguousarray(source.faces, dtype=np.int32)
    fractions = {minimum_triangle_fraction, .125, .25, .50, .75}
    for fraction in sorted(f for f in fractions if f >= minimum_triangle_fraction):
        target = max(12, int(len(faces)*fraction))
        vertices, indices = fast_simplification.simplify(points, faces, target_count=target,
                                                        preserve_border=bool(border))
        if len(indices) >= len(best.faces) or len(indices) < 4:
            continue
        points_in_metres = project_surface_vertices(vertices*size+center, spheres)
        candidate = clean(Mesh(points_in_metres.astype(np.float32).astype(float), indices))
        if not useful(best, candidate) or topology(candidate) != (euler, border, non_manifold):
            continue
        try:
            if len(components(candidate)) != 1:
                continue
            check = certificate(source, candidate, limit)
        except ValueError:
            continue
        best, details = candidate, {'method': 'bounded_surface_decimation', 'certificate': check}
        break
    if meshoptimizer_library is not None:
        alternate = meshopt_surface(source, best, limit, spheres, meshoptimizer_library)
        if alternate is not None:
            return alternate
    return best, details


def meshopt_surface(source: Mesh, previous: Mesh, limit: float, spheres: tuple[Sphere, ...],
                    library: Path) -> tuple[Mesh, dict] | None:
    target = max(8, len(previous.faces)//12)
    signature = topology(source)
    max_gap = max(1e-5, sphere_gap(source,spheres) or 0.)
    best, details = previous, None
    for update in (False, True):
        for fraction in (.75, .50, .35, .24, .16, .10, .065, .04):
            error = limit*fraction
            draft = meshopt_simplify(source,library,target,error,update)
            if len(draft.faces) >= len(best.faces) or len(draft.faces) < 4:
                continue
            used, indexes = np.unique(draft.faces,return_inverse=True)
            points = project_surface_vertices(draft.vertices[used],spheres).astype(np.float32).astype(float)
            candidate = clean(Mesh(points,indexes.reshape(-1,3)))
            if (not useful(best,candidate) or topology(candidate) != signature
                    or (sphere_gap(candidate,spheres) or 0.) > max_gap):
                continue
            try:
                if len(components(candidate)) != 1:
                    continue
                check = certificate(source,candidate,limit)
            except ValueError:
                continue
            best, details = candidate, {'method': 'bounded_meshoptimizer', 'certificate': check,
                                        'proposed_error_m': error, 'updated_positions': update}
            break
    return (best, details) if details is not None else None


def pack(pieces: list[Mesh]) -> Mesh:
    faces, count = [], 0
    for piece in pieces:
        faces.append(piece.faces+count)
        count += len(piece.vertices)
    return Mesh(np.vstack([piece.vertices for piece in pieces]), np.vstack(faces))


def counts(mesh: Mesh) -> dict[str, int]:
    return {'vertices': len(mesh.vertices), 'triangles': len(mesh.faces)}


def fitted_geometry(item: MeshInput, original: Mesh, factor: float) -> tuple[Mesh, list[dict]]:
    welded = clean(original)
    revised, reports = [], []
    for piece in components(welded):
        candidate = piece
        report = {'method': 'unchanged'}
        if item.enclosure and len(piece.vertices) > 8:
            try:
                outer, evidence = enclosing_hull(piece, max_expansion=item.solid_limit*factor,
                    margin=3e-7, max_iterations=512, planes_per_iteration=1)
                if useful(piece, outer) and topology(outer) == (2, 0, 0):
                    candidate = outer
                    report = {'method': 'conservative_enclosure', 'certificate': evidence}
            except (QhullError, ValueError):
                pass
        elif not item.enclosure:
            surface, bores = close_bores(piece, item.bore_radius, .006) if item.bore_radius else (piece, [])
            candidate, report = decimate(surface, item.surface_limit*factor, item.spheres,
                                         item.minimum_triangle_fraction, item.meshoptimizer_library)
            if bores:
                report['closed_bores'] = bores
        revised.append(candidate)
        reports.append(dict(report, before=counts(piece), after=counts(candidate)))
    packed = pack(revised)
    if not useful(original, packed):
        return original, reports
    if not item.enclosure and len(welded.faces) < len(original.faces):
        a, b, c = original.vertices[original.faces].transpose(1, 0, 2)
        degenerate = original.faces[np.linalg.norm(np.cross(b-a,c-a),axis=1)==0.]
        if len(degenerate):
            # Duplicate nondegenerate faces share the exact remaining surface.
            used, faces = np.unique(degenerate,return_inverse=True)
            removed = Mesh(original.vertices[used],faces.reshape(-1,3))
            target = welded if any(report.get('closed_bores') for report in reports) else packed
            verify = directed_bound(removed,target,item.surface_limit,max_depth=16)
            reports.append({'method': 'degenerate_surface_cleanup', 'certificate': verify})
    return packed, reports


def sphere_gap(mesh: Mesh, spheres: tuple[Sphere, ...]) -> float | None:
    """Repeat the CAD primitive coverage check on vertices and face centres."""
    if not spheres:
        return None
    balls = np.asarray(spheres)
    samples = np.vstack((mesh.vertices,mesh.vertices[mesh.faces].mean(axis=1)))
    maximum = 0.
    for start in range(0, len(samples), 2048):
        chunk = samples[start:start+2048]
        distances_to_centers = np.linalg.norm(chunk[:, None, :]-balls[None, :, :3], axis=-1)
        outside = (distances_to_centers-balls[:, 3]).min(axis=1)
        maximum = max(maximum, float(outside.max()))
    return maximum


def optimize_one(item: MeshInput) -> dict:
    original = read_mesh(item.path)
    if item.path.suffix == '.stl':
        return {'before': counts(original), 'after': counts(original),
                'source_sha256': hashlib.sha256(item.path.read_bytes()).hexdigest(),
                'components': [{'method': 'unchanged', 'reason': 'Retain the original gripper drive rod'}]}
    before_gap = sphere_gap(original,item.spheres)
    limit = max(1e-5,before_gap or 0.)
    result, reports, after_gap = original, [], before_gap
    try:
        for factor in (1., .5, .25):
            candidate, checks = fitted_geometry(item,original,factor)
            gap = sphere_gap(candidate,item.spheres)
            if gap is None or gap <= limit:
                result, reports, after_gap = candidate, checks, gap
                break
        else:
            reports = [{'method': 'unchanged', 'reason': 'Retain the cached GPU sphere coverage'}]
    except ValueError as error:
        reports = [{'method': 'unchanged', 'reason': str(error)}]
    record = {'before': counts(original), 'after': counts(result),
              'source_sha256': hashlib.sha256(item.path.read_bytes()).hexdigest(), 'components': reports}
    if before_gap is not None:
        record['gpu_sphere_sample_gap_m'] = {'before': before_gap, 'after': after_gap}
    if result is not original:
        body = trimesh.Trimesh(vertices=result.vertices, faces=result.faces, process=False)
        binary = trimesh.exchange.gltf.export_glb(body.scene(), include_normals=False)
        sha256 = hashlib.sha256(binary).hexdigest()
        path = item.destination / (sha256 + '.glb')
        with NamedTemporaryFile(suffix='.glb', dir=item.destination, delete=False) as temporary:
            temporary.write(binary)
        staged = Path(temporary.name)
        try:
            reopened = read_mesh(staged)
            if (not np.array_equal(reopened.vertices, result.vertices)
                    or not np.array_equal(reopened.faces, result.faces)):
                raise ValueError(f'GLB serialization altered certified coordinates or triangles: {path}')
            staged.replace(path)
        finally:
            staged.unlink(missing_ok=True)
        record['result'] = path.name
        record['sha256'] = sha256
    return record


def scope(name: str) -> str:
    return 'self' if name.startswith('cad_self__') else 'world' if name.startswith('cad_world__') else 'both'


def local_spheres(origin: ET.Element | None, cached: list[dict]) -> tuple[Sphere, ...]:
    if not cached:
        return ()
    attributes = origin.attrib if origin is not None else {}
    shift = np.fromstring(attributes.get('xyz','0 0 0'),sep=' ')
    rotation = Rotation.from_euler('xyz',np.fromstring(attributes.get('rpy','0 0 0'),sep=' '))
    return tuple((*map(float,rotation.inv().apply(np.asarray(item['center'])-shift)),item['radius'])
                 for item in cached)


def group(root: ET.Element, package_path: Path, output: Path, surface: float, solid: float,
          contact: float, bore_radius: float, minimum_triangle_fraction: float,
          meshoptimizer_library: Path | None):
    cad = json.loads(root.attrib['cleany_cad_calibration'])
    uri = cad['gpu_primitives_uri']
    if not uri.startswith(PACKAGE):
        raise ValueError('Expected the installed CAD primitive cache URI')
    primitives = (package_path/uri.removeprefix(PACKAGE)).read_bytes()
    if hashlib.sha256(primitives).hexdigest() != cad['gpu_primitives_sha256']:
        raise ValueError('CAD primitive cache hash differs from the URDF')
    spheres = json.loads(primitives)['links']
    uses = defaultdict(list)
    for link in root.findall('link'):
        name = link.get('name')
        for element in link.findall('collision'):
            mesh = element.find('geometry/mesh')
            if mesh is None:
                continue
            label = element.get('name','')
            uri = mesh.get('filename')
            if not uri.startswith(PACKAGE) or not np.array_equal(np.fromstring(mesh.get('scale','1 1 1'),sep=' '),[1,1,1]):
                raise ValueError(f'Unexpected calibrated collision asset: {uri}')
            path = package_path/uri.removeprefix(PACKAGE)
            enclosure = label.removeprefix('cad_self__').removeprefix('cad_world__').startswith('cad_enclosure__')
            protected = any(word in name for word in ('gripper_tip_', 'gripper_driving_'))
            # Keep measured finger pad and drive rod surfaces at tighter bounds.
            cap = min(surface, contact) if protected else surface
            expansion = min(solid, contact) if protected else solid
            # Limit this topology change to accessory housings with surveyed circular bores.
            accessory = ('HeadPan', 'WristPrint', 'LeftCamera_FixedMount', 'RightCamera_FixedMount')
            hole_limit = bore_radius if not protected and label.startswith(accessory) else 0.
            cached = [] if name == 'base_link' or scope(label) == 'self' else spheres.get(name,{}).get('spheres',[])
            cover = local_spheres(element.find('origin'),cached)
            uses[MeshInput(path,output,enclosure,cap,expansion,hole_limit,cover,
                           minimum_triangle_fraction,meshoptimizer_library)].append((name,label,mesh))
    return uses


def totals(records: list[dict]) -> dict:
    output = {}
    for scope_name in ('world','self','all'):
        rows = [r for r in records if scope_name=='all' or r['scope'] in ('both',scope_name)]
        output[scope_name] = {key: {f'{prefix}{field}':sum(r[key][field] for r in rows if not prefix or r['enclosure'])
                                    for prefix in ('','enclosure_') for field in ('vertices','triangles')}
                              for key in ('before','after')}
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--surface-mm', type=float, required=True)
    parser.add_argument('--enclosure-mm', type=float, required=True)
    parser.add_argument('--contact-mm', type=float, required=True)
    parser.add_argument('--bore-radius-mm', type=float, default=0.)
    parser.add_argument('--minimum-triangle-fraction', type=float, default=.25,
                        help='Smallest fraction of source triangles to try, default 0.25')
    parser.add_argument('--meshoptimizer-library', type=Path,
                        help='Optional meshoptimizer 1.0 shared library for an additional surface candidate')
    parser.add_argument('--jobs', type=int, default=4)
    args = parser.parse_args()
    surface, solid, contact = np.array([args.surface_mm,args.enclosure_mm,args.contact_mm])/1000
    bore_radius = args.bore_radius_mm/1000
    if (not all(np.isfinite(x) and x>0 for x in (surface,solid,contact)) or args.jobs<1
            or not np.isfinite(bore_radius) or bore_radius < 0):
        parser.error('Distance limits and job count must be finite and positive')
    if not 0 < args.minimum_triangle_fraction < 1:
        parser.error('Minimum triangle fraction must be between zero and one')
    output = args.output.resolve()
    assets = output/'assets'
    assets.mkdir(parents=True)
    description = args.urdf.read_bytes()
    root = ET.fromstring(description)
    library = args.meshoptimizer_library.resolve(strict=True) if args.meshoptimizer_library else None
    inputs = group(root,args.package,assets,surface,solid,contact,bore_radius,
                   args.minimum_triangle_fraction,library)
    records = []
    with ProcessPoolExecutor(max_workers=args.jobs) as executor:
        for index,(item,record) in enumerate(zip(inputs,executor.map(optimize_one,inputs)),start=1):
            for name,label,mesh in inputs[item]:
                if 'result' in record:
                    mesh.set('filename',(assets/record['result']).as_uri())
                records.append(dict(record,link=name,collision=label,scope=scope(label),
                                    enclosure=item.enclosure,source=str(item.path.relative_to(args.package))))
            print(f'{index}/{len(inputs)} {inputs[item][0][1]} {record["before"]["triangles"]} -> '
                  f'{record["after"]["triangles"]}', flush=True)
    root_out = output/'robot-candidate.urdf'
    ET.ElementTree(root).write(root_out,encoding='unicode')
    report = {'source_urdf_sha256':hashlib.sha256(description).hexdigest(),
              'limits_m':{'surface':surface,'enclosure':solid,'gripper_contact':contact,
                          'bore_radius':bore_radius,'bore_length':.006},
              'totals':totals(records),'records':records}
    if args.minimum_triangle_fraction != .25:
        report['minimum_triangle_fraction'] = args.minimum_triangle_fraction
    if library is not None:
        report['meshoptimizer_library_sha256'] = hashlib.sha256(library.read_bytes()).hexdigest()
    (output/'geometry.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report['totals']),flush=True)


if __name__=='__main__':
    main()
