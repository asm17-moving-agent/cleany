"""Fit selected arm and head housings independently in their owning link frames."""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import linprog, minimize_scalar
from scipy.spatial import ConvexHull
from scipy.spatial.distance import pdist
import trimesh

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'ros2_ws/src/cleany_skill_executor')]
from compare_sphere_fits import cell, clip, enclosing_ball
from prune_sphere_fit import certify_cell
from cleany_skill_executor.collision_sphere_model import collision_signatures


def resolve(uri: str) -> Path:
    return ROOT/'ros2_ws/src'/uri.removeprefix('package://')


def coverage(tetra, centers, radii, padding):
    return np.column_stack([
        np.max(np.linalg.norm(tetra-center, axis=2), axis=1) <= radius-padding+1e-10
        for center, radius in zip(centers, radii)])


def collision_components(link: ET.Element, mesh_name: str | None = None):
    """Return complete connected components in the owning link frame."""
    for collision in link.findall('collision'):
        mesh, origin = collision.find('geometry/mesh'), collision.find('origin')
        if mesh is None or (mesh_name and not mesh.get('filename').endswith('/'+mesh_name)):
            continue
        geometry = trimesh.load_mesh(resolve(mesh.get('filename')), process=True)
        geometry.apply_scale(np.fromstring(mesh.get('scale', '1 1 1'), sep=' '))
        rpy = np.fromstring(origin.get('rpy', '0 0 0') if origin is not None else '0 0 0', sep=' ')
        xyz = np.fromstring(origin.get('xyz', '0 0 0') if origin is not None else '0 0 0', sep=' ')
        rotation = trimesh.transformations.euler_matrix(*rpy)[:3, :3]
        for component in geometry.split(only_watertight=False):
            yield component.vertices @ rotation.T + xyz


def selected_components(link: ET.Element, selectors: list[dict]) -> list[np.ndarray]:
    """Select whole housings, including their necks, without clipping triangles."""
    selected = []
    for selector in selectors:
        parts = []
        for vertices in collision_components(link, selector['mesh']):
            axis = selector.get('axis', 0)
            if vertices[:, axis].min() < selector.get('minimum_m', -np.inf):
                continue
            if vertices[:, axis].max() > selector.get('maximum_m', np.inf):
                continue
            parts.append(vertices)
        if len(parts) != selector['expected_components']:
            raise ValueError('Housing component selection changed: '+link.get('name'))
        selected.extend(parts)
    return selected


def sphere_pair_volume(centers: np.ndarray, radii: np.ndarray) -> float:
    """Return the union volume of two spheres, including their overlap."""
    distance = float(np.linalg.norm(centers[1]-centers[0]))
    a, b = radii
    if distance <= abs(a-b):
        return float(4*np.pi*max(radii)**3/3)
    overlap = 0.
    if distance < a+b:
        overlap = np.pi*(a+b-distance)**2*(distance**2+2*distance*(a+b)
                    -3*(a-b)**2)/(12*distance)
    return float(4*np.pi*np.sum(radii**3)/3-overlap)


def fit_housing_pair(
    parts: list[np.ndarray], axis: int, padding: float,
) -> tuple[list[tuple[np.ndarray, float]], dict]:
    """Split along a link-local housing axis and minimize occupied volume.

    Convex clipping includes every cut-face vertex. A bounded search refines
    each sampled local minimum; this is not a global two-sphere optimum.
    Returned radii exclude padding, which is included in the search objective.
    """
    if type(axis) is not int or axis not in (0, 1, 2):
        raise ValueError('A two-sphere housing needs partition_axis 0, 1 or 2')
    points = np.vstack(parts)
    low, high = points[:, axis].min(), points[:, axis].max()
    direction = np.eye(3)[axis]

    def evaluate(fraction: float):
        groups = clip(parts, direction, low+fraction*(high-low))
        balls = [cell(group) for group in groups]
        centers = np.array([ball.center for ball in balls])
        radii = np.array([ball.radius+padding+1e-9 for ball in balls])
        return sphere_pair_volume(centers, radii), balls

    fractions = np.linspace(.1, .9, 17)
    candidates = [(float(f), *evaluate(float(f))) for f in fractions]
    scores = [row[1] for row in candidates]
    for i in range(len(fractions)):
        if scores[i] > min(scores[max(0, i-1):min(len(scores), i+2)]):
            continue
        result = minimize_scalar(lambda f: evaluate(f)[0], method='bounded',
            bounds=(fractions[max(0, i-1)], fractions[min(len(fractions)-1, i+1)]),
            options={'xatol': 1e-6})
        candidates.append((float(result.x), *evaluate(float(result.x))))
    fraction, volume, balls = min(candidates, key=lambda row: row[1])
    before = sum(ConvexHull(p).volume for p in parts)
    after = sum(ConvexHull(p).volume for ball in balls for p in ball.pieces)
    if not np.isclose(before, after, rtol=1e-8, atol=1e-15):
        raise ValueError('Housing split lost source volume')
    return [(b.center, b.radius) for b in balls], {
        'axis': axis, 'cut_m': float(low+fraction*(high-low)),
        'fraction': fraction, 'sphere_union_volume_m3': volume,
        'source_volume_m3': before, 'partition_volume_m3': after}


def minimum_ball_bound(vertices: np.ndarray, center: np.ndarray, radius: float,
                       padding: float) -> dict:
    """Bound any enclosing radius below by a weighted support-point variance.

    Nonnegative weights summing to one give a lower bound for every center.
    Agreement with the fitted enclosing radius certifies the one-ball optimum
    numerically, rather than relying on the optimizer's success flag.
    """
    distance = np.linalg.norm(vertices-center, axis=1)
    support = np.unique(vertices[distance >= radius-1e-7], axis=0)
    weights = linprog(np.zeros(len(support)),
        A_eq=np.vstack((np.ones(len(support)), ((support-center)/radius).T)),
        b_eq=np.array([1., 0., 0., 0.]), bounds=(0, None), method='highs')
    if weights.success:
        values = np.maximum(weights.x, 0.)
        values /= values.sum()
        mean = values @ support
        bound = float(np.sqrt(values @ np.square(support-mean).sum(axis=1)))
        witness = {'points_m': support.tolist(), 'weights': values.tolist()}
    else:
        hull = vertices[ConvexHull(vertices).vertices]
        bound = float(pdist(hull).max()/2)
        witness = {'fallback': 'farthest_vertex_pair'}
    return {'diameter_mm': 2000*(radius+padding),
            'diameter_lower_bound_mm': 2000*(bound+padding),
            'diameter_gap_mm': 2000*max(0., radius-bound), **witness}


def refine_housing_ball(parts: list[np.ndarray], balls: list[tuple[np.ndarray, float]],
                        index: int, padding: float, slack: float, iterations: int):
    """Refit one ball against solid points uncovered by the fixed other ball.

    Failed solid certificates supply actual uncovered points for the next fit.
    Only a complete tetrahedron certificate permits returning a smaller ball.
    Slack is a numerical enclosure allowance in metres, separate from padding.
    """
    if len(balls) != 2 or type(index) is not int or index not in (0, 1):
        raise ValueError('Refinement needs one selected ball in a pair')
    if not np.isfinite(slack) or slack <= 0 or type(iterations) is not int or iterations < 1:
        raise ValueError('Invalid refinement tolerance or iteration budget')
    centers = np.array([b[0] for b in balls])
    radii = np.array([b[1]+padding+1e-9 for b in balls])
    before = radii[index]
    fixed = 1-index
    points = np.concatenate(parts)
    points = points[np.linalg.norm(points-centers[fixed], axis=1) > radii[fixed]-padding+1e-10]
    if len(points) == 0:
        raise ValueError('No uncovered geometry needs the selected ball')
    for step in range(iterations):
        center, radius = enclosing_ball(points)
        centers[index], radii[index] = center, radius+padding+slack+1e-9
        failures = []
        proofs = [certify_cell(part, centers, radii, padding, failure_points=failures)
                  for part in parts]
        if all(proof is not None for proof in proofs):
            if radii[index] >= before:
                raise ValueError('Refinement did not shrink the selected ball')
            result = list(balls)
            result[index] = (center, radius+slack)
            return result, {'part_index': index, 'fixed_part_index': fixed,
                'method': 'uncovered solid counterexamples with fixed other ball',
                'iterations': step+1, 'constraint_points': len(points), 'slack_m': slack,
                'before_diameter_mm': float(2000*before),
                'after_diameter_mm': float(2000*radii[index]),
                'sphere_union_volume_m3': sphere_pair_volume(centers, radii)}
        if not failures:
            raise ValueError('Refinement solid certificate exhausted its subdivision budget')
        points = np.unique(np.vstack((points, failures)), axis=0)
    raise ValueError('Refinement did not obtain complete solid coverage')


def primitive_components(link: ET.Element, selector: dict) -> list[np.ndarray]:
    """Enclose selected boxes and complete cylinders in link-local boxes.

    The cylinder box encloses the continuous surface, including points between
    polygon samples. An unknown primitive or a changed selection fails closed.
    """
    parts = []
    for collision in link.findall('collision'):
        if selector['collision_name_contains'] not in collision.get('name', ''):
            continue
        box = collision.find('geometry/box')
        cylinder = collision.find('geometry/cylinder')
        if box is not None:
            size = np.fromstring(box.get('size'), sep=' ')
        elif cylinder is not None:
            size = np.array([2*float(cylinder.get('radius'))]*2
                            + [float(cylinder.get('length'))])
        else:
            raise ValueError('Unsupported primitive in '+link.get('name'))
        if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
            raise ValueError('Invalid primitive dimensions')
        origin = collision.find('origin')
        xyz = np.fromstring(origin.get('xyz', '0 0 0') if origin is not None else '0 0 0', sep=' ')
        rpy = np.fromstring(origin.get('rpy', '0 0 0') if origin is not None else '0 0 0', sep=' ')
        rotation = trimesh.transformations.euler_matrix(*rpy)[:3, :3]
        parts.append(trimesh.creation.box(extents=size).vertices @ rotation.T + xyz)
    if len(parts) != selector['expected_components']:
        raise ValueError('Primitive selection changed: '+link.get('name'))
    return parts


def append_primitive_regions(data: dict, report: dict, arrays: dict,
                             root: ET.Element, regions: dict) -> None:
    """Append certified primitive balls without refitting the existing mesh balls."""
    for side in regions['sides']:
        for region in regions.get('primitive_regions', []):
            name = region['link'].format(side=side)
            count = region['sphere_count']
            if count not in (1, 2):
                raise ValueError('Primitive sphere_count must be one or two')
            assert not any(c['link'] == name and c['region'] == region['name']
                           for c in data['rigid_covers'])
            parts = primitive_components(root.find(f"link[@name='{name}']"), region['selector'])
            vertices = np.concatenate(parts)
            padding = data['padding_m']
            if count == 2:
                balls, split = fit_housing_pair(parts, region.get('partition_axis'), padding)
                if 'refine_part_index' in region:
                    balls, refinement = refine_housing_ball(parts, balls,
                        region['refine_part_index'], padding, region['refinement_slack_m'],
                        region['refinement_max_iterations'])
                    split['refinement'] = refinement
            else:
                balls = [enclosing_ball(vertices)]
            spheres = data['links'][name]['spheres']
            index = len(spheres)
            for part_index, (center, radius) in enumerate(balls):
                spheres.append({'center': center.tolist(), 'radius': float(radius+padding+1e-9)})
                cover = {'link': name, 'sphere_index': index+part_index, 'region': region['name'],
                         'primitive_selector': region['selector'], 'part_index': part_index,
                         'sphere_count': count}
                data['rigid_covers'].append(cover)
                report.setdefault('primitive_regions', []).append(cover)
                if count == 1:
                    report['minimum_ball_bounds'][f'{name}#{index}'] = {
                        **minimum_ball_bound(vertices, center, radius, padding),
                        'link': name, 'sphere_index': index, 'region': region['name'],
                        'primitive_selector': region['selector']}
            if count == 2:
                report.setdefault('partitioned_primitive_regions', []).append({
                    'link': name, 'region': region['name'], 'sphere_indices': [index, index+1],
                    'selector': region['selector'], 'partition': split})
            centers = np.array([s['center'] for s in spheres[index:]])
            radii = np.array([s['radius'] for s in spheres[index:]])
            first = int(arrays[name+'__source_piece_indices'].max())+1
            chunks, owners, sphere_owners = [], [], []
            for i, part in enumerate(parts):
                proof = certify_cell(part, centers, radii, padding)
                assert proof is not None, (name, i)
                chunks.append(proof[0])
                owners.append(np.full(len(proof[0]), first+i, dtype=int))
                sphere_owners.append(proof[1]+index)
            added = np.concatenate(chunks)
            additions = {
                'source_vertices': vertices,
                'source_piece_indices': np.concatenate([
                    np.full(len(part), first+i, dtype=int) for i, part in enumerate(parts)]),
                'tetrahedra': added, 'cell_indices': np.concatenate(owners),
                'sphere_indices': np.concatenate(sphere_owners)}
            for suffix, values in additions.items():
                key = name+'__'+suffix
                arrays[key] = np.concatenate((arrays[key], values))
            arrays[name+'__effective_centers'] = np.array([s['center'] for s in spheres])
            arrays[name+'__effective_radii'] = np.array([s['radius'] for s in spheres])
            row = report['links'][name]
            row.update(after=len(spheres), certificate_tetrahedra=len(arrays[name+'__tetrahedra']),
                       primitive_components_added=len(parts))
            row.pop('sampled_support_increase_mm', None)
    report['rigid_covers'] = data['rigid_covers']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--previous', type=Path, required=True)
    parser.add_argument('--certificate', type=Path, required=True)
    parser.add_argument('--regions', type=Path, default=ROOT/'ros2_ws/src/cleany_skill_executor/config/collision_sphere_regions.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    old = json.loads(args.previous.read_text())
    assert hashlib.sha256(args.certificate.read_bytes()).hexdigest() == old['coverage_certificate_sha256']
    xml = args.urdf.read_text()
    root = ET.fromstring(xml)
    data = deepcopy(old)
    regions = json.loads(args.regions.read_text())
    assert regions['schema'] == 2
    data.update(schema=2, model='joint_simplified', joint_covers=[], rigid_covers=[])
    data['fitting_regions'] = regions
    data['fitting_regions_sha256'] = hashlib.sha256(args.regions.read_bytes()).hexdigest()
    data['links'].pop('top_base_link')
    signatures = collision_signatures(xml, set(data['links']), resolve)
    assert all(signatures[name] == entry['collision_sha256'] for name, entry in data['links'].items())
    padding = data['padding_m']
    rigid, fit_bounds, partitioned = {}, {}, []
    requested = [{**region, 'link': region['link'].format(side=side)}
                 for side in regions['sides'] for region in regions['regions']]
    requested.extend(regions.get('fixed_regions', []))
    for region in requested:
        name = region['link']
        link = root.find(f"link[@name='{name}']")
        parts = (selected_components(link, region['selectors']) if region['selectors']
                 else list(collision_components(link)))
        vertices = np.concatenate(parts)
        count = region.get('sphere_count', 1)
        if count not in (1, 2):
            raise ValueError('Housing sphere_count must be one or two')
        if count == 2:
            balls, split = fit_housing_pair(parts, region.get('partition_axis'), padding)
        else:
            balls = [enclosing_ball(vertices)]
        spheres = rigid.setdefault(name, [])
        indices = []
        for part_index, (center, radius) in enumerate(balls):
            index = len(spheres)
            indices.append(index)
            spheres.append({'center': center.tolist(), 'radius': float(radius+padding+1e-9)})
            if count == 1:
                fit_bounds[name if index == 0 else f'{name}#{index}'] = {
                    **minimum_ball_bound(vertices, center, radius, padding),
                    'link': name, 'sphere_index': index, 'region': region['name']}
            data['rigid_covers'].append({'link': name, 'sphere_index': index,
                'region': region['name'], 'selectors': region['selectors'],
                'part_index': part_index, 'sphere_count': count})
        if count == 2:
            centers = np.array([spheres[i]['center'] for i in indices])
            radii = np.array([spheres[i]['radius'] for i in indices])
            assert all(certify_cell(p, centers, radii, padding) is not None for p in parts), name
            partitioned.append({'link': name, 'region': region['name'],
                'sphere_indices': indices, 'selectors': region['selectors'],
                'partition': split})
    proofs = np.load(args.certificate, allow_pickle=False)
    arrays, report = {}, {'links': {}, 'joint_covers': data['joint_covers'],
                          'rigid_covers': data['rigid_covers'],
                          'partitioned_regions': partitioned,
                          'minimum_ball_bounds': fit_bounds,
                          'previous_certificate_sha256': old['coverage_certificate_sha256']}
    directions = trimesh.creation.icosphere(subdivisions=3).vertices
    for name, entry in data['links'].items():
        original = old['links'][name]['spheres']
        tetra = proofs[name+'__tetrahedra']
        fixed = list(rigid.get(name, []))
        spheres = fixed + deepcopy(original)
        protected = len(fixed)
        cell_indices = proofs[name+'__cell_indices']
        if any(row['link'] == name for row in partitioned):
            # Repartition the original solid cells so a stale tetrahedron
            # crossing the new sphere boundary cannot retain a redundant ball.
            source = proofs[name+'__source_vertices']
            pieces = proofs[name+'__source_piece_indices']
            c = np.array([s['center'] for s in spheres])
            r = np.array([s['radius'] for s in spheres])
            chunks, owners = [], []
            for piece in np.unique(pieces):
                proof = certify_cell(source[pieces == piece], c, r, padding)
                assert proof is not None, (name, piece)
                chunks.append(proof[0])
                owners.append(np.full(len(proof[0]), piece, dtype=int))
            tetra, cell_indices = np.concatenate(chunks), np.concatenate(owners)
        if fixed:
            for iteration in range(3):
                effective = spheres
                centers = np.array([s['center'] for s in effective])
                radii = np.array([s['radius'] for s in effective])
                cover = coverage(tetra, centers, radii, padding)
                assert cover.any(axis=1).all()
                active = list(range(len(effective)))
                # Retain the housing ball while deleting redundant old spheres.
                for index in sorted(range(protected, len(spheres)), key=lambda i: int(cover[:, i].sum())):
                    remaining = [i for i in active if i != index]
                    if cover[:, remaining].any(axis=1).all():
                        active.remove(index)
                own = [i for i in active if i < len(spheres)]
                priority = list(range(protected))
                priority += sorted([i for i in own if i >= protected], key=lambda i: radii[i])
                assignment = np.array(priority)[np.argmax(cover[:, priority], axis=1)]
                shrunk = []
                for index in own:
                    if index < protected:
                        shrunk.append(spheres[index]); continue
                    vertices = tetra[assignment == index].reshape(-1, 3)
                    if not len(vertices):
                        continue
                    points = np.unique(vertices, axis=0)
                    if len(points) >= 4:
                        points = points[ConvexHull(points).vertices]
                    center, radius = enclosing_ball(points)
                    shrunk.append({'center': center.tolist(), 'radius': radius+padding+1e-9})
                spheres = shrunk
        if any(row['link'] == name for row in partitioned):
            # A new partition can cover the solid with fewer balls than a
            # fixed set of old tetrahedra suggests. Recertify each deletion.
            centers = np.array([s['center'] for s in spheres])
            radii = np.array([s['radius'] for s in spheres])
            active = list(range(len(spheres)))
            piece_ids = np.unique(pieces)
            cells = [source[pieces == piece] for piece in piece_ids]
            for index in sorted(range(protected, len(spheres)), key=lambda i: radii[i]):
                remaining = [i for i in active if i != index]
                if all(certify_cell(cell, centers[remaining], radii[remaining], padding)
                       is not None for cell in cells):
                    active = remaining
            spheres = [spheres[i] for i in active]
            checked = [certify_cell(cell, centers[active], radii[active], padding)
                       for cell in cells]
            assert all(proof is not None for proof in checked), name
            tetra = np.concatenate([proof[0] for proof in checked])
            cell_indices = np.concatenate([np.full(len(proof[0]), piece, dtype=int)
                for piece, proof in zip(piece_ids, checked)])
        entry['spheres'] = spheres
        effective = spheres
        centers, radii = np.array([s['center'] for s in effective]), np.array([s['radius'] for s in effective])
        cover = coverage(tetra, centers, radii, padding)
        assert cover.any(axis=1).all(), name
        indices = np.argmax(cover, axis=1)
        residual = float((np.linalg.norm(tetra-centers[indices, None], axis=2)-radii[indices, None]).max())
        old_centers = np.array([s['center'] for s in original]); old_radii = np.array([s['radius'] for s in original])
        support_increase = float(((centers @ directions.T+radii[:, None]).max(0)
                                 -(old_centers @ directions.T+old_radii[:, None]).max(0)).max())
        for suffix in ('source_vertices', 'source_piece_indices'):
            arrays[name+'__'+suffix] = proofs[name+'__'+suffix]
        arrays[name+'__tetrahedra'] = tetra
        arrays[name+'__cell_indices'] = cell_indices
        arrays[name+'__sphere_indices'] = indices
        arrays[name+'__effective_centers'], arrays[name+'__effective_radii'] = centers, radii
        report['links'][name] = {'before': len(original), 'after': len(spheres),
            'inherited_joint_spheres': 0, 'certificate_tetrahedra': len(tetra),
            'max_enclosure_residual_m': residual, 'sampled_support_increase_mm': support_increase*1000}
        print(name, len(original), '->',len(spheres),'support increase mm',round(support_increase*1000,2),flush=True)
    append_primitive_regions(data, report, arrays, root, regions)
    np.savez_compressed(args.output/'certificates.npz', **arrays)
    data['coverage_certificate_sha256'] = hashlib.sha256((args.output/'certificates.npz').read_bytes()).hexdigest()
    data['source_model_sha256'] = hashlib.sha256(args.previous.read_bytes()).hexdigest()
    data['coverage_kind'] = 'complete tetrahedra with independent link-local housing and primitive balls'
    data.pop('coverage_samples', None)
    data.pop('uncovered_samples', None)
    report.update(before=sum(len(x['spheres']) for x in old['links'].values()),
                  after=sum(len(x['spheres']) for x in data['links'].values()),
                  mast_spheres_removed=13, complete_solid_coverage=True,
                  limitations=['Larger joint balls can reject narrow passages; sampled support is not a worst-case bound.'])
    (args.output/'collision_spheres_joint_simplified.json').write_text(json.dumps(data,indent=2)+'\n')
    (args.output/'fitting.json').write_text(json.dumps(report,indent=2)+'\n')
    print('TOTAL',report['before'],'->',report['after'],flush=True)


if __name__ == '__main__':
    main()
