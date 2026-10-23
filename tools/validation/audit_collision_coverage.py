"""Build the actual collision model and measure independent CAD surface coverage.

Run in the pinned motion container after building its ROS overlay. This loads
URDF/SRDF only; it never creates a ROS node, controller or motion request.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from ament_index_python.packages import get_package_share_directory

from cleany_skill_executor.curobo_model import CuRoboConfig, RobotModel
from cleany_skill_executor.robot_geometry_cache import model_cache_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--srdf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cache', type=Path, required=True)
    args = parser.parse_args()
    config = CuRoboConfig(geometry_cache_directory=str(args.cache))
    started = time.monotonic()
    model = RobotModel(args.urdf.read_text(), args.srdf.read_text(), config)
    report = {'model_build_sec': time.monotonic() - started, 'config': vars(config),
              'full_cache': model_cache_path(args.urdf.read_text(), args.srdf.read_text(), config).name,
              'sphere_reduction': model.sphere_reduction, 'links': {},
              'bodies': [{'owner': body.owner, 'vertices': len(body.vertices), 'planes': len(body.planes)}
                         for body in model.bodies]}
    rng = np.random.default_rng(20261023)
    for link in ET.fromstring(args.urdf.read_text()).findall('link'):
        name = link.get('name')
        if name not in model.spheres:
            continue
        samples = []
        for collision in link.findall('collision'):
            element = collision.find('geometry/mesh')
            if element is None:
                continue
            uri = element.get('filename')
            package, relative = uri.removeprefix('package://').split('/', 1)
            path = Path(get_package_share_directory(package)) / relative.replace('meshes/collision/', 'meshes/')
            mesh = trimesh.load_mesh(path, process=True)
            mesh.apply_scale([float(v) for v in element.get('scale', '1 1 1').split()])
            indices = rng.choice(len(mesh.faces), size=20000, p=mesh.area_faces / mesh.area_faces.sum())
            uv = rng.random((len(indices), 2))
            uv[uv.sum(axis=1) > 1] = 1 - uv[uv.sum(axis=1) > 1]
            triangles = mesh.triangles[indices]
            surface = triangles[:, 0] + uv[:, :1] * (triangles[:, 1] - triangles[:, 0]) + uv[:, 1:] * (triangles[:, 2] - triangles[:, 0])
            origin = collision.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            rotation = trimesh.transformations.euler_matrix(*rpy)[:3, :3]
            samples.append(np.vstack((mesh.vertices, surface)) @ rotation.T + xyz)
        points = np.concatenate(samples)
        spheres = model.spheres[name]
        centers = np.array([sphere['center'] for sphere in spheres])
        radii = np.array([sphere['radius'] for sphere in spheres])
        gap = (np.linalg.norm(points[:, None, :] - centers, axis=2) - radii).min(axis=1)
        report['links'][name] = {'cad_vertices_and_surface_samples': len(points),
            'uncovered': int((gap > 1e-9).sum()), 'max_signed_gap_m': float(gap.max()),
            'sphere_radius_range_m': [float(radii.min()), float(radii.max())]}
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)
    assert all(item['uncovered'] == 0 for item in report['links'].values()), 'Uncovered CAD geometry'


if __name__ == '__main__':
    main()
