"""Configure numerical workers before importing the ROS grasp server."""
from __future__ import annotations

import os


def main(args: list[str] | None = None) -> None:
    """Start the server with an explicit OpenBLAS worker budget."""
    value = os.environ.get('CLEANY_GRASP_BLAS_THREADS', '1')
    try:
        threads = int(value)
    except ValueError:
        raise ValueError('CLEANY_GRASP_BLAS_THREADS must be a positive integer') from None
    if threads < 1:
        raise ValueError('CLEANY_GRASP_BLAS_THREADS must be a positive integer')
    # NumPy initializes its pool at import; changing this in GraspNode is too late.
    os.environ['OPENBLAS_NUM_THREADS'] = str(threads)
    print(f'grasp_server: OPENBLAS_NUM_THREADS={threads}', flush=True)
    from cleany_grasping.grasp_node import main as run_server

    run_server(args=args)
