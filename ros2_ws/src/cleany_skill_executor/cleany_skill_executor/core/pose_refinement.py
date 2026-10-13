"""Bounded numerical pose refinement; no robot commands or collision claims."""
from collections.abc import Callable
import numpy as np


def refine_pose(residual: Callable[[np.ndarray], np.ndarray], seed, bounds,
                iterations: int = 30) -> tuple[float, ...]:
    limits = np.asarray(bounds, dtype=float)
    q = np.asarray(seed, dtype=float)
    if (q.ndim != 1 or limits.shape != (len(q), 2) or iterations < 1
            or not np.isfinite(limits).all() or not np.isfinite(q).all()
            or np.any(limits[:, 0] >= limits[:, 1])):
        raise ValueError('Invalid bounded pose refinement inputs')
    q = np.clip(q, limits[:,0], limits[:,1])
    damping = 1e-4
    def evaluate(values):
        result = np.asarray(residual(values), dtype=float)
        if result.ndim != 1 or not len(result) or not np.isfinite(result).all():
            raise ValueError('Pose residual must be a finite vector')
        return result
    error = evaluate(q)
    for _ in range(iterations):
        columns = []
        for index in range(len(q)):
            trial = q.copy()
            delta = min(1e-4, (limits[index,1]-limits[index,0])/2)
            if q[index]+delta > limits[index,1]:
                delta = -delta
            trial[index] += delta
            columns.append((evaluate(trial)-error)/delta)
        jacobian = np.column_stack(columns)
        step = np.linalg.solve(jacobian.T@jacobian+damping*np.eye(len(q)), -jacobian.T@error)
        if np.max(np.abs(step)) > .25:
            step *= .25/np.max(np.abs(step))
        trial = np.clip(q+step, limits[:,0], limits[:,1])
        next_error = evaluate(trial)
        if np.linalg.norm(next_error) < np.linalg.norm(error):
            if np.max(np.abs(trial-q)) < 1e-6:
                break
            q, error = trial, next_error
            damping = max(1e-7, damping/3)
        else:
            damping = min(1., damping*10)
            if damping == 1.:
                break
        if np.linalg.norm(error) < 1e-5:
            break
    return tuple(float(v) for v in q)
