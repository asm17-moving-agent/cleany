"""Create static USD geometry from the materialized Gazebo world without resizing."""
import json
from pathlib import Path

import numpy as np
from pxr import Gf, UsdGeom, UsdPhysics


from mesh_geometry import read_geometry


def create_facility(stage, output: Path):
    entries = json.loads((output / "config/world_geometry.json").read_text())
    mesh_cache, floor = {}, None
    colliders = 0
    for index, entry in enumerate(entries):
        path = f"/Facility18/{entry['model']}/{entry['kind']}_{index}"
        shape = entry["shape"]
        scale = None
        if shape["type"] == "box":
            geometry = UsdGeom.Cube.Define(stage, path)
            geometry.CreateSizeAttr(1.)
            scale = shape["size"]
        elif shape["type"] == "cylinder":
            geometry = UsdGeom.Cylinder.Define(stage, path)
            geometry.CreateRadiusAttr(shape["radius"])
            geometry.CreateHeightAttr(shape["length"])
            geometry.CreateAxisAttr("Z")
        elif shape["type"] == "sphere":
            geometry = UsdGeom.Sphere.Define(stage, path)
            geometry.CreateRadiusAttr(shape["radius"])
        elif shape["type"] == "mesh":
            asset = shape["asset"]
            if asset not in mesh_cache:
                mesh_cache[asset] = read_geometry(output / asset)
            points, counts, indices = mesh_cache[asset]
            geometry = UsdGeom.Mesh.Define(stage, path)
            geometry.CreatePointsAttr(points)
            geometry.CreateFaceVertexCountsAttr(counts)
            geometry.CreateFaceVertexIndicesAttr(indices)
            geometry.CreateSubdivisionSchemeAttr("none")
            geometry.CreateDoubleSidedAttr(entry["model"] != "facility_ceiling")
            scale = shape["scale"]
        else:
            raise ValueError(shape)
        transform = np.array(entry["transform"])
        if scale is not None:
            transform[:3, :3] = transform[:3, :3] @ np.diag(scale)
        geometry.AddTransformOp().Set(Gf.Matrix4d(transform.T.tolist()))
        if entry["kind"] == "collision":
            UsdPhysics.CollisionAPI.Apply(geometry.GetPrim())
            geometry.CreateVisibilityAttr("invisible")
            colliders += 1
            if shape["type"] == "mesh":
                UsdPhysics.MeshCollisionAPI.Apply(geometry.GetPrim()).CreateApproximationAttr("none")
            if entry["model"] == "ground_plane":
                floor = geometry.GetPrim()
        else:
            geometry.CreateDisplayColorAttr([entry["color"]])
            geometry.CreateDisplayOpacityAttr([entry["opacity"]])
    assert floor is not None and colliders == sum(e["kind"] == "collision" for e in entries), (floor, colliders)
    (output / "evaluations/world-import.json").write_text(json.dumps({
        "source": "existing facility18 generated SDF", "static_colliders": colliders,
        "visuals": len(entries)-colliders, "unique_meshes": len(mesh_cache),
        "mesh_collision": "unchanged triangle surfaces, no convex hull replacement",
        "visual_limit": "geometry preserved; diffuse colors replace floor/chair textures"}, indent=2))
    return floor
