"""Optional meshoptimizer 1.0 backend for offline collision-mesh fitting."""
from __future__ import annotations

import ctypes
from functools import lru_cache
from pathlib import Path

import numpy as np

from optimize_arm_collision_meshes import Mesh


_UINT = ctypes.POINTER(ctypes.c_uint32)
_FLOAT = ctypes.POINTER(ctypes.c_float)
_SIZE = ctypes.c_size_t
_LOCK_BORDER = 1 << 0
_ABSOLUTE_ERROR = 1 << 2


@lru_cache(maxsize=1)
def _library(path: Path) -> ctypes.CDLL:
    backend = ctypes.CDLL(str(path))
    backend.meshopt_simplify.argtypes = [
        _UINT, _UINT, _SIZE, _FLOAT, _SIZE, _SIZE, _SIZE, ctypes.c_float, ctypes.c_uint, _FLOAT]
    backend.meshopt_simplify.restype = _SIZE
    backend.meshopt_simplifyWithUpdate.argtypes = [
        _UINT, _SIZE, _FLOAT, _SIZE, _SIZE, _FLOAT, _SIZE, _FLOAT, _SIZE, ctypes.c_void_p,
        _SIZE, ctypes.c_float, ctypes.c_uint, _FLOAT]
    backend.meshopt_simplifyWithUpdate.restype = _SIZE
    return backend


def simplify(source: Mesh, path: Path, target_triangles: int, error_m: float, update: bool) -> Mesh:
    """Propose a lower-resolution surface with locked open CAD borders."""
    library = _library(path)
    points = np.array(source.vertices, dtype=np.float32, order='C', copy=True)
    indices = np.array(source.faces, dtype=np.uint32, order='C', copy=True)
    vertex_buffer = points.ctypes.data_as(_FLOAT)
    source_buffer = indices.ctypes.data_as(_UINT)
    stride = points.strides[0]
    options = _ABSOLUTE_ERROR | _LOCK_BORDER
    target = target_triangles * 3
    if update:
        output = indices
        count = library.meshopt_simplifyWithUpdate(source_buffer, indices.size, vertex_buffer,
            len(points), stride, None, 0, None, 0, None, target, error_m, options, None)
    else:
        output = np.empty_like(indices)
        count = library.meshopt_simplify(output.ctypes.data_as(_UINT), source_buffer,
            indices.size, vertex_buffer, len(points), stride, target, error_m, options, None)
    return Mesh(points.astype(float), output.ravel()[:count].reshape(-1, 3).astype(np.int64))
