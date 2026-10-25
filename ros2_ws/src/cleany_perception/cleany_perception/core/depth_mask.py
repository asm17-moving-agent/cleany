"""Reject ambiguous depth islands inside a 2D instance segmentation."""
import math
import numpy as np
from numpy.typing import NDArray


def dominant_depth_mask(depth: NDArray, mask: NDArray, *, minimum_depth_m: float, maximum_depth_m: float,
                        maximum_jump_m: float, minimum_fraction: float) -> NDArray[np.bool_]:
    """Keep the dominant 8-connected depth surface, or fail on ambiguity.

    Connectivity uses adjacent depth differences, preserving sloped surfaces.
    Discarded pixels remain available to the surrounding obstacle cloud. This
    uses neither object-class dimensions nor simulator ground truth.
    """
    if (not math.isfinite(maximum_jump_m) or maximum_jump_m <= 0
            or not .5 < minimum_fraction <= 1):
        raise ValueError('Invalid depth-component thresholds')
    depth = np.asarray(depth)
    mask = np.asarray(mask, dtype=bool)
    if depth.ndim != 2 or depth.shape != mask.shape:
        raise ValueError('Depth and mask dimensions must match')
    valid = mask & np.isfinite(depth) & (depth >= minimum_depth_m) & (depth <= maximum_depth_m)
    height, width = depth.shape
    visited = np.zeros_like(valid)
    largest = []
    for y, x in zip(*np.nonzero(valid)):
        if visited[y, x]:
            continue
        visited[y, x] = True
        component = [(y, x)]
        for row, column in component:
            for dy, dx in ((-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)):
                yy, xx = row+dy, column+dx
                if (0 <= yy < height and 0 <= xx < width and valid[yy, xx]
                        and not visited[yy, xx]
                        and abs(float(depth[yy, xx])-float(depth[row, column])) <= maximum_jump_m):
                    visited[yy, xx] = True
                    component.append((yy, xx))
        if len(component) > len(largest):
            largest = component
    if not largest or len(largest) < minimum_fraction*int(valid.sum()):
        raise ValueError('Instance mask has no dominant connected depth surface')
    cleaned = np.zeros_like(valid)
    rows, columns = zip(*largest)
    cleaned[rows, columns] = True
    return cleaned
