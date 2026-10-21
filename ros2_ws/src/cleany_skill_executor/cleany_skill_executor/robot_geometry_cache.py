"""Persist immutable robot geometry; runtime state and CUDA objects are never cached."""
from __future__ import annotations

import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
from tempfile import NamedTemporaryFile
import xml.etree.ElementTree as ET
from zipfile import BadZipFile

import numpy as np

from .curobo_collision import ConvexBody


def model_cache_path(urdf: str, srdf: str, config) -> Path | None:
    if not config.geometry_cache_directory:
        return None
    from ament_index_python.packages import get_package_share_directory

    digest = hashlib.sha256()
    inputs = [urdf, srdf, json.dumps([
        config.spheres_per_link, config.sphere_surface_radius_m, config.sphere_budget_per_link])]
    for package in ('numpy', 'scipy', 'trimesh', 'nvidia_curobo'):
        inputs.append(package + '=' + importlib.metadata.version(package))
    source = Path(__file__).parent
    for relative in ('curobo_model.py', 'curobo_collision.py', 'geometry_cache.py',
                     'core/sphere_reduction.py', 'robot_geometry_cache.py'):
        inputs.append(hashlib.sha256((source / relative).read_bytes()).hexdigest())
    for uri in sorted({mesh.get('filename') for mesh in ET.fromstring(urdf).findall('.//collision/geometry/mesh')}):
        if uri.startswith('package://'):
            package, relative = uri[len('package://'):].split('/', 1)
            path = Path(get_package_share_directory(package)) / relative
        else:
            path = Path(uri)
        inputs.extend((uri, hashlib.sha256(path.read_bytes()).hexdigest()))
    for item in inputs:
        digest.update(item.encode())
        digest.update(b'\0')
    return Path(config.geometry_cache_directory).expanduser() / ('model-' + digest.hexdigest() + '.npz')


def _checksum(metadata: str, arrays: list[np.ndarray]) -> str:
    digest = hashlib.sha256(metadata.encode())
    for array in arrays:
        digest.update(str((array.dtype.str, array.shape)).encode())
        digest.update(array.tobytes())
    return digest.hexdigest()


def load_model(path: Path | None, model) -> bool:
    if path is None:
        return False
    try:
        with np.load(path, allow_pickle=False) as stored:
            metadata = str(stored['metadata'])
            data = json.loads(metadata)
            owners = data.pop('body_owners')
            arrays = [stored[f'body_{i}_{part}'].copy()
                      for i in range(len(owners)) for part in ('vertices', 'planes')]
            if str(stored['checksum']) != _checksum(metadata, arrays):
                return False
        if set(data) != {'limits', 'velocity_limits', 'spheres', 'collision_links',
                         'ignore', 'sphere_reduction'}:
            return False
        bodies = []
        for i, owner in enumerate(owners):
            vertices, planes = arrays[2*i:2*i+2]
            for values, width in ((vertices, 3), (planes, 4)):
                if (values.dtype != np.float64 or values.ndim != 2 or values.shape[1] != width
                        or not len(values) or not np.isfinite(values).all()):
                    return False
                values.setflags(write=False)
            bodies.append(ConvexBody(owner, vertices, planes))
        data['limits'] = {name: tuple(bounds) for name, bounds in data['limits'].items()}
        # Install only after the entire bundle has passed integrity checks.
        model.__dict__.update(data, bodies=bodies)
        return True
    except (OSError, ValueError, TypeError, KeyError, IndexError, EOFError, BadZipFile):
        return False


def save_model(path: Path | None, model) -> None:
    if path is None:
        return
    metadata = json.dumps({**{name: getattr(model, name) for name in (
        'limits', 'velocity_limits', 'spheres', 'collision_links', 'ignore', 'sphere_reduction')},
        'body_owners': [body.owner for body in model.bodies]}, sort_keys=True, allow_nan=False)
    arrays = {f'body_{i}_{part}': getattr(body, part) for i, body in enumerate(model.bodies)
              for part in ('vertices', 'planes')}
    temporary = None
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with NamedTemporaryFile(dir=path.parent, suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            np.savez_compressed(stream, metadata=metadata,
                                checksum=_checksum(metadata, list(arrays.values())), **arrays)
        os.replace(temporary, path)
    except OSError:
        pass  # An unavailable performance cache cannot prevent model construction.
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
