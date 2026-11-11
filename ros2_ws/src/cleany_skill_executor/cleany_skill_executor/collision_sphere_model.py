"""Load certified link-local spheres only for their exact source geometry."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Callable
import xml.etree.ElementTree as ET

import numpy as np


MODEL_PATH = Path(__file__).parent / 'data' / 'collision_spheres_joint_simplified.json'


def mesh_path(uri: str) -> Path:
    if uri.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory

        package, relative = uri[len('package://'):].split('/', 1)
        return Path(get_package_share_directory(package)) / relative
    return Path(uri)


def collision_signatures(urdf: str, names: set[str],
                         resolve: Callable[[str], Path] = mesh_path) -> dict[str, str]:
    """Bind spheres to collision origins, scales and mesh file contents."""
    checksums: dict[str, str] = {}

    def canonical(element: ET.Element) -> list:
        attributes = dict(element.attrib)
        for key in ('xyz', 'rpy', 'scale', 'size', 'radius', 'length'):
            if key in attributes:
                attributes[key] = [float(v) for v in attributes[key].split()]
        if element.tag == 'mesh':
            uri = element.get('filename')
            if uri not in checksums:
                checksums[uri] = hashlib.sha256(resolve(uri).read_bytes()).hexdigest()
            attributes['sha256'] = checksums[uri]
        return [element.tag, attributes, [canonical(child) for child in element]]

    signatures = {}
    for link in ET.fromstring(urdf).findall('link'):
        name = link.get('name')
        if name in names:
            encoded = json.dumps([canonical(c) for c in link.findall('collision')],
                                 sort_keys=True, separators=(',', ':'), allow_nan=False)
            signatures[name] = hashlib.sha256(encoded.encode()).hexdigest()
    return signatures


def asset_sha256() -> str:
    return hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest()


def joint_signature(joint: ET.Element) -> str:
    """Bind a shared joint-centered ball to its parent/child transforms and axis."""
    elements = []
    for tag in ('parent', 'child', 'origin', 'axis'):
        element = joint.find(tag)
        attrs = dict(element.attrib) if element is not None else {}
        for key in ('xyz', 'rpy'):
            if key in attrs:
                attrs[key] = [float(v) for v in attrs[key].split()]
        elements.append([tag, attrs])
    return hashlib.sha256(json.dumps([joint.get('name'), joint.get('type'), elements],
                                    sort_keys=True).encode()).hexdigest()


def validate_joint_covers(urdf: str, data: dict) -> None:
    """A ball at a revolute joint origin encloses the same child region at every angle."""
    root = ET.fromstring(urdf)
    for cover in data['joint_covers']:
        joint = root.find(f"joint[@name='{cover['joint']}']")
        if (joint is None or joint.get('type') != 'revolute'
                or joint_signature(joint) != cover['joint_sha256']
                or joint.find('parent').get('link') != cover['parent']
                or joint.find('child').get('link') != cover['child']):
            raise ValueError('Certified joint cover geometry mismatch: '+cover['joint'])
        sphere = data['links'][cover['parent']]['spheres'][cover['sphere_index']]
        center = np.fromstring(joint.find('origin').get('xyz', '0 0 0'), sep=' ')
        if (not np.allclose(sphere['center'], center, atol=1e-12, rtol=0)
                or sphere['radius'] != cover['radius_m']):
            raise ValueError('Certified joint ball is not centered on its joint: '+cover['joint'])


def apply_sphere_model(model, *, resolve: Callable[[str], Path] = mesh_path) -> None:
    """Replace proposal spheres; preserve complete collision validation bodies.

    Headless profiles select the matching subset of the certified model. A
    changed collision mesh or origin fails explicitly instead of using a stale
    cover. Shared joint balls additionally bind the relative joint frames; their
    centers stay at the joint origin for every measured revolute joint angle.
    """
    if model.config.sphere_model == 'generated':
        return
    data = json.loads(MODEL_PATH.read_text())
    if data['schema'] != 2 or data['model'] != model.config.sphere_model:
        raise ValueError('Unsupported collision sphere asset')
    if abs(data['padding_m'] - model.config.sphere_surface_radius_m) > 1e-12:
        raise ValueError('Certified sphere padding differs from configured padding')
    validate_joint_covers(model.xml, data)
    names = set(model.spheres)
    if not names or not names.issubset(data['links']):
        raise ValueError('Certified sphere asset does not cover the robot mesh links')
    signatures = collision_signatures(model.xml, names, resolve)
    replacement = {}
    for name in model.spheres:
        entry = data['links'][name]
        if signatures.get(name) != entry['collision_sha256']:
            raise ValueError('Certified sphere geometry mismatch: ' + name)
        spheres = entry['spheres']
        centers = np.asarray([s['center'] for s in spheres], dtype=float)
        radii = np.asarray([s['radius'] for s in spheres], dtype=float)
        if (centers.shape != (len(spheres), 3) or not len(spheres)
                or not np.isfinite(centers).all() or not np.isfinite(radii).all()
                or (radii <= 0).any()):
            raise ValueError('Invalid certified collision spheres: ' + name)
        replacement[name] = spheres
    model.spheres = replacement
    reduction = model.sphere_reduction
    reduction['method'] = data['model']
    reduction['sphere_asset_sha256'] = asset_sha256()
    reduction['coverage_certificate_sha256'] = data['coverage_certificate_sha256']
    for name, spheres in replacement.items():
        previous = reduction['links'][name]
        reduction['links'][name] = {'before': previous['before'], 'after': len(spheres)}
    reduction['after'] = sum(len(spheres) for spheres in replacement.values())
    reduction['after_self_collision_pairs'] = sum(
        len(replacement[a]) * len(replacement[b])
        for a in replacement for b in replacement if a < b
        and b not in model.ignore.get(a, ()) and a not in model.ignore.get(b, ()))
