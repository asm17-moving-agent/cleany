"""Content-addressed, optional cache for expensive mesh sphere fitting."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Callable
from zipfile import BadZipFile

import numpy as np


def fitted_geometry(path: Path, scale: tuple[float, ...], count: int,
                    radius: float, versions: str, directory: str,
                    fit: Callable) -> tuple[tuple[np.ndarray, ...], bool]:
    def checked(values):
        centers, radii, vertices = values
        if (centers.ndim != 2 or centers.shape[1] != 3 or not len(centers)
                or radii.shape != (len(centers),)
                or vertices.ndim != 2 or vertices.shape[1] != 3 or not len(vertices)
                or any(not np.isfinite(v).all() for v in values)
                or np.any(radii <= 0)):
            raise ValueError(f'Invalid fitted collision geometry: {path}')
        for value in values:
            value.setflags(write=False)
        return values

    cache = None
    if directory:
        digest = hashlib.sha256(path.read_bytes())
        digest.update(json.dumps([2, scale, count, radius, versions,
                                  'convex_hull/subdivide']).encode())
        cache = Path(directory).expanduser() / (digest.hexdigest()+'.npz')
        try:
            with np.load(cache, allow_pickle=False) as stored:
                values = tuple(stored[name].copy() for name in ('centers', 'radii', 'vertices'))
                if str(stored['data_sha256']) != _array_digest(values):
                    raise ValueError('Geometry cache checksum differs')
            return checked(values), True
        except (OSError, ValueError, TypeError, KeyError, EOFError, BadZipFile):
            pass
    values = checked(tuple(np.asarray(v) for v in fit()))
    if cache is not None:
        temporary = None
        try:
            cache.parent.mkdir(parents=True, exist_ok=True)
            with NamedTemporaryFile(dir=cache.parent, suffix='.tmp', delete=False) as stream:
                temporary = Path(stream.name)
                np.savez_compressed(stream, data_sha256=_array_digest(values),
                    **dict(zip(('centers', 'radii', 'vertices'), values)))
            os.replace(temporary, cache)
        except OSError:
            # A read-only/full cache must not prevent authoritative mesh fitting.
            pass
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
    return values, False


def _array_digest(values) -> str:
    digest = hashlib.sha256()
    for value in values:
        digest.update(str((value.dtype.str, value.shape)).encode())
        digest.update(value.tobytes())
    return digest.hexdigest()
