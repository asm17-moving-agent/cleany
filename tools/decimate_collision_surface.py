"""CPU-only Open3D worker; JSON in/out, run in the existing motion image."""
import json
import sys

import numpy as np
import open3d as o3d


def main() -> None:
    request = json.load(sys.stdin)
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
    source = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(np.asarray(request['vertices'], dtype=float)),
        o3d.utility.Vector3iVector(np.asarray(request['faces'], dtype=np.int32)),
    )
    result = source.simplify_quadric_decimation(request['target_triangles'])
    # Certify and serialize the same float32 coordinates as binary STL.
    print(json.dumps({'vertices': np.asarray(result.vertices).astype(np.float32).tolist(),
        'faces': np.asarray(result.triangles).tolist(), 'open3d_version': o3d.__version__}))


if __name__ == '__main__':
    main()
