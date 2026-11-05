"""Check the saved CAD pose as a cuRobo mesh world on CUDA."""

import argparse
import json
import tempfile
from hashlib import sha256
from pathlib import Path
from urllib.parse import unquote, urlsplit
from xml.etree import ElementTree as ET


def _source_hashes(package: Path) -> dict:
    return {
        "source_urdf_sha256": sha256(
            (package / "urdf/cleany_freecad_final.urdf").read_bytes()
        ).hexdigest(),
        "source_geometry_sha256": sha256(
            (package / "config/geometry/freecad_final.json").read_bytes()
        ).hexdigest(),
    }


def mesh_query_distance(meshes: dict, cuboids: dict | None = None) -> float:
    """Cover deep solid interiors, not just the default near-surface band."""
    import numpy as np

    # An object's bounding-box diagonal bounds its interior-to-surface distance.
    # This is a search radius, not collision padding or a robot calibration value.
    diagonals = [float(np.linalg.norm(mesh.extents)) for mesh in meshes.values()]
    diagonals.extend(
        float(np.linalg.norm(box["dims"])) for box in (cuboids or {}).values()
    )
    return max([0.1, *diagonals])


def _cuboid_hash(cuboids: dict) -> str:
    return sha256(
        json.dumps(cuboids, sort_keys=True, allow_nan=False).encode()
    ).hexdigest()


def load_world_shapes(package: Path, cache: Path):
    """Load native cuboids and mesh objects bound to the same CAD snapshot."""
    import numpy as np
    import trimesh

    cache = cache.resolve()
    manifest = json.loads((cache / "manifest.json").read_text())
    if manifest.get("schema") not in (
        "cleany.freecad_mesh_world/v1",
        "cleany.freecad_collision_world/v2",
    ):
        raise ValueError("Unknown cuRobo mesh world cache")
    if any(
        manifest.get(key) != value for key, value in _source_hashes(package).items()
    ):
        raise ValueError("cuRobo mesh world cache was built from a different CAD model")
    if (
        "source_kinematic_urdf_sha256" in manifest
        and manifest["source_kinematic_urdf_sha256"]
        != sha256(
            (package / "urdf/cleany_freecad_kinematic.urdf").read_bytes()
        ).hexdigest()
    ):
        raise ValueError("cuRobo mesh world cache was built from different CAD joints")
    meshes = {}
    for name, record in manifest["meshes"].items():
        path = (cache / record["file"]).resolve()
        if not path.is_relative_to(cache / "meshes") or path.suffix != ".glb":
            raise ValueError("Invalid cuRobo mesh world asset path")
        if sha256(path.read_bytes()).hexdigest() != record["sha256"]:
            raise ValueError("cuRobo mesh world asset was modified")
        meshes[name] = trimesh.load(path, process=False, force="mesh")
    cuboids = manifest.get("cuboids", {})
    if manifest["schema"] == "cleany.freecad_collision_world/v2":
        if _cuboid_hash(cuboids) != manifest.get("cuboids_sha256"):
            raise ValueError("cuRobo cuboid data was modified")
    elif cuboids:
        raise ValueError("Legacy mesh worlds cannot declare cuboids")
    if meshes.keys() & cuboids.keys():
        raise ValueError("Duplicate cuRobo collision object name")
    for box in cuboids.values():
        pose, dims = np.asarray(box["pose"]), np.asarray(box["dims"])
        if (
            pose.shape != (7,)
            or dims.shape != (3,)
            or not np.isfinite(pose).all()
            or not np.isfinite(dims).all()
            or (dims <= 0).any()
            or abs(np.linalg.norm(pose[3:]) - 1) > 1e-8
        ):
            raise ValueError("Invalid cuRobo cuboid")
    lidar = np.asarray(manifest["lidar_probe"], dtype=float)
    if lidar.shape != (3,) or not np.isfinite(lidar).all():
        raise ValueError("Invalid cuRobo probe in cache")
    return meshes, cuboids, lidar


def load_world_cache(package: Path, cache: Path):
    """Load preview meshes, tessellating native cuboids only for this helper."""
    import trimesh

    meshes, cuboids, lidar = load_world_shapes(package, cache)
    for name, box in cuboids.items():
        mesh = trimesh.creation.box(extents=box["dims"])
        pose = box["pose"]
        mesh.apply_transform(_pose_matrix(pose))
        mesh.vertices = mesh.vertices.astype("float32").astype(float)
        meshes[name] = mesh
    return meshes, lidar


def _pose_matrix(pose: list[float]):
    import numpy as np
    from scipy.spatial.transform import Rotation

    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat([*pose[4:], pose[3]]).as_matrix()
    matrix[:3, 3] = pose[:3]
    return matrix


def write_world_cache(
    package: Path, output: Path, cache: Path | None = None, *, mesh_boxes: bool = False
) -> dict:
    """Cache the prepared mesh objects without changing their triangles."""
    import numpy as np
    import trimesh

    output = output.resolve()
    if output.exists() or output.is_relative_to(package.resolve()):
        raise ValueError("Use a new world cache outside the description package")
    primitive_cache = (
        cache
        and json.loads((cache / "manifest.json").read_text()).get("mode")
        == "moveit_primitives"
    )
    if primitive_cache:
        meshes, cuboids, lidar = _load_primitive_world(
            package, cache, native_boxes=not mesh_boxes
        )
    else:
        meshes, lidar = load_snapshot(package, cache)
        cuboids = {}
    manifest = {
        "schema": "cleany.freecad_collision_world/v2"
        if cuboids
        else "cleany.freecad_mesh_world/v1",
        **_source_hashes(package),
        "source_collision_cache_sha256": sha256(
            (cache / "manifest.json").read_bytes()
        ).hexdigest()
        if cache
        else None,
        "max_surface_error_m": json.loads((cache / "manifest.json").read_text()).get(
            "max_surface_error_m"
        )
        if cache
        else None,
        "lidar_probe": lidar.tolist(),
        "meshes": {},
        "cuboids": cuboids,
        "cuboids_sha256": _cuboid_hash(cuboids),
        "triangles": 0,
        "bytes": 0,
        "required_mesh_query_distance_m": mesh_query_distance(meshes, cuboids),
    }
    if (
        cache
        and json.loads((cache / "manifest.json").read_text()).get("mode")
        == "moveit_primitives"
    ):
        source_manifest = json.loads((cache / "manifest.json").read_text())
        manifest.update(
            model_variant="freecad_kinematic_saved_pose",
            source_kinematic_urdf_sha256=sha256(
                (package / "urdf/cleany_freecad_kinematic.urdf").read_bytes()
            ).hexdigest(),
            max_surface_error_m=source_manifest["retained_surface_error_m"],
            primitive_cylinder_sections=64,
            body_enclosure=source_manifest.get("body_enclosure"),
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=output.parent) as temporary:
        stage = Path(temporary) / "world"
        (stage / "meshes").mkdir(parents=True)
        for index, (name, mesh) in enumerate(meshes.items()):
            relative = f"meshes/{index:02d}-{name}.glb"
            path = stage / relative
            # cuRobo needs geometry only. Dropping inherited materials also
            # prevents older trimesh versions from returning a multi-part Scene.
            trimesh.Trimesh(
                vertices=mesh.vertices, faces=mesh.faces, process=False
            ).export(path)
            restored = trimesh.load(path, process=False, force="mesh")
            if not np.array_equal(mesh.triangles, restored.triangles):
                raise ValueError(f"World cache changed triangles: {name}")
            manifest["meshes"][name] = {
                "file": relative,
                "sha256": sha256(path.read_bytes()).hexdigest(),
            }
            manifest["triangles"] += len(mesh.faces)
            manifest["bytes"] += path.stat().st_size
        (stage / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        stage.rename(output)
    return manifest


def load_primitive_snapshot(package: Path, cache: Path):
    """Bake the all-mesh CAD q=0 representation for preview or comparison."""
    meshes, _, lidar = _load_primitive_world(package, cache, native_boxes=False)
    return meshes, lidar


def _load_primitive_world(package: Path, cache: Path, *, native_boxes: bool):
    """Bake CAD q=0, keeping primitives and declared enclosures separate.

    Cylinders use 64 circumscribing sides. Float32 outward rounding encloses the
    analytic cylinder in its local frame; this is not physical collision padding.
    All transformed mesh coordinates are rounded to the GLB float32 format.
    """
    import numpy as np
    import trimesh
    from cleany_moveit_config.freecad_collision import inspection_urdf
    from scipy.spatial.transform import Rotation

    root = ET.fromstring(inspection_urdf(package, "freecad_kinematic", cache))

    def transform(element):
        matrix = np.eye(4)
        origin = element.find("origin")
        if origin is not None:
            matrix[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
            matrix[:3, :3] = Rotation.from_euler(
                "xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
            ).as_matrix()
        return matrix

    frames = {"base_link": np.eye(4)}
    pending = list(root.findall("joint"))
    while pending:
        count = len(pending)
        for joint in pending[:]:
            parent, child = (
                joint.find("parent").get("link"),
                joint.find("child").get("link"),
            )
            if joint.get("type") not in (
                "fixed",
                "continuous",
                "revolute",
                "prismatic",
            ):
                raise ValueError("Unsupported CAD joint in saved-pose cache")
            if parent in frames:
                frames[child] = frames[parent] @ transform(joint)
                pending.remove(joint)
        if len(pending) == count:
            raise ValueError("Disconnected CAD joint tree")
    meshes, cuboids = {}, {}
    for link in root.findall("link"):
        pieces = []
        for index, collision in enumerate(link.findall("collision")):
            shape = collision.find("geometry")[0]
            placement = frames[link.get("name")] @ transform(collision)
            if shape.tag == "box" and native_boxes:
                x, y, z, w = Rotation.from_matrix(placement[:3, :3]).as_quat()
                cuboids[f"{link.get('name')}_solid_{index}"] = {
                    "pose": [
                        *placement[:3, 3].tolist(),
                        float(w),
                        float(x),
                        float(y),
                        float(z),
                    ],
                    "dims": np.fromstring(shape.get("size"), sep=" ").tolist(),
                }
                continue
            if shape.tag == "mesh":
                part = trimesh.load(
                    unquote(urlsplit(shape.get("filename")).path),
                    process=False,
                    force="mesh",
                )
                part.apply_scale(np.fromstring(shape.get("scale", "1 1 1"), sep=" "))
            elif shape.tag == "box":
                part = trimesh.creation.box(
                    extents=np.fromstring(shape.get("size"), sep=" ")
                )
            elif shape.tag == "cylinder":
                radius = float(shape.get("radius"))
                half = float(shape.get("length")) / 2
                radial = float(
                    np.nextafter(
                        np.float32(radius / np.cos(np.pi / 64)), np.float32(np.inf)
                    )
                )
                axial = float(np.nextafter(np.float32(half), np.float32(np.inf)))
                part = trimesh.creation.cylinder(
                    radius=radial, height=2 * axial, sections=64
                )
            else:
                raise ValueError("Unsupported CAD primitive")
            part.apply_transform(placement)
            part.vertices = part.vertices.astype(np.float32).astype(float)
            if shape.tag == "mesh" and not collision.get("name", "").startswith(
                "cad_enclosure__"
            ):
                pieces.append(part)
            else:
                # A single concatenated mesh can give the wrong inside sign on
                # coincident/interior faces of adjacent or overlapping solids.
                meshes[f"{link.get('name')}_solid_{index}"] = part
        if pieces:
            meshes[link.get("name")] = trimesh.util.concatenate(pieces)
    metadata = json.loads((package / "config/geometry/freecad_final.json").read_text())
    lidar = next(part for part in metadata["parts"] if part["name"] == "RPLidarA1M8")
    source = trimesh.load(
        package / lidar["meshes"][0]["mesh"], process=False, force="mesh"
    )
    return meshes, cuboids, source.triangles[0].mean(axis=0)


def load_snapshot(package: Path, cache: Path | None = None):
    """Read collision meshes already expressed in the CAD base frame."""
    import trimesh

    package = package.resolve()
    if (
        cache is not None
        and json.loads((cache / "manifest.json").read_text()).get("mode")
        == "moveit_primitives"
    ):
        return load_primitive_snapshot(package, cache)
    if cache is None:
        root = ET.parse(package / "urdf/cleany_freecad_final.urdf").getroot()
    else:
        from cleany_moveit_config.freecad_collision import inspection_urdf

        root = ET.fromstring(inspection_urdf(package, "freecad_final", cache))
    if root.get("name") != "cleany_freecad_final_static":
        raise ValueError("Expected the fixed FreeCAD reference model")
    for joint in root.findall("joint"):
        if (
            joint.get("type") != "fixed"
            or joint.find("parent").get("link") != "base_link"
        ):
            raise ValueError("The snapshot needs fixed children of base_link")
    for origin in root.findall("joint/origin") + root.findall("link/collision/origin"):
        if any(
            float(v) != 0 for key in ("xyz", "rpy") for v in origin.get(key, "").split()
        ):
            raise ValueError("The snapshot collision geometry must be in base_link")

    meshes = {}
    lidar_probe = None
    for link in root.findall("link"):
        pieces = []
        for collision in link.findall("collision"):
            mesh = collision.find("geometry/mesh")
            prefix = "package://cleany_description/"
            uri = mesh.get("filename", "") if mesh is not None else ""
            if uri.startswith(prefix):
                relative = Path(uri.removeprefix(prefix))
                if (
                    not relative.is_relative_to("meshes/freecad_final")
                    or ".." in relative.parts
                ):
                    raise ValueError(f"Unexpected FreeCAD collision asset: {relative}")
                path = package / relative
            elif cache is not None and uri.startswith("file://"):
                url = urlsplit(uri)
                path = Path(unquote(url.path)).resolve()
                if url.netloc or not path.is_relative_to(cache.resolve() / "meshes"):
                    raise ValueError(f"Unexpected indexed collision asset: {uri}")
            else:
                raise ValueError("Expected a FreeCAD collision mesh")
            part = trimesh.load(path, process=path.suffix == ".stl", force="mesh")
            part.apply_scale([float(v) for v in mesh.get("scale", "1 1 1").split()])
            pieces.append(part)
            if lidar_probe is None and "RPLidarA1M8" in path.stem:
                lidar_probe = part.triangles[0].mean(axis=0)
        if pieces:
            meshes[link.attrib["name"]] = trimesh.util.concatenate(pieces)
    if lidar_probe is None:
        raise ValueError("LiDAR geometry is missing from the snapshot")
    return meshes, lidar_probe


def check_cuda(
    package: Path, cache: Path | None = None, world_cache: Path | None = None
) -> dict:
    """Compare a probe on the LiDAR surface with one clear of the robot."""
    import numpy as np
    import torch
    from curobo.geom.sdf.world import CollisionQueryBuffer, WorldCollisionConfig
    from curobo.geom.sdf.world_mesh import WorldMeshCollision
    from curobo.geom.types import Cuboid, Mesh, WorldConfig
    from curobo.types.base import TensorDeviceType

    if cache is not None and world_cache is not None:
        raise ValueError("Choose collision_cache or world_cache")
    if world_cache:
        meshes, cuboids, lidar = load_world_shapes(package, world_cache)
    elif (
        cache
        and json.loads((cache / "manifest.json").read_text()).get("mode")
        == "moveit_primitives"
    ):
        meshes, cuboids, lidar = _load_primitive_world(
            package, cache, native_boxes=True
        )
    else:
        meshes, lidar = load_snapshot(package, cache)
        cuboids = {}
    upper = [mesh.bounds[1] for mesh in meshes.values()]
    upper.extend(
        np.asarray(box["pose"][:3]) + np.linalg.norm(box["dims"])
        for box in cuboids.values()
    )
    far = np.max(upper, axis=0) + 1
    tensor = TensorDeviceType(device=torch.device("cuda:0"))
    world = WorldConfig(
        cuboid=[Cuboid(name=name, **box) for name, box in cuboids.items()],
        mesh=[
            Mesh(
                name=name,
                pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                vertices=mesh.vertices,
                faces=mesh.faces,
            )
            for name, mesh in meshes.items()
        ],
    )
    checker = WorldMeshCollision(
        WorldCollisionConfig(
            tensor_args=tensor,
            world_model=world,
            cache={"mesh": len(meshes), "obb": len(cuboids)},
            max_distance=mesh_query_distance(meshes, cuboids),
        )
    )
    # Two 2 mm test probes. They do not set collision padding on the robot.
    query = tensor.to_device([[*lidar, 0.002], [*far, 0.002]]).view(1, 1, 2, 4)
    buffer = CollisionQueryBuffer.initialize_from_shape(
        query.shape, tensor, checker.collision_types
    )
    costs = (
        checker.get_sphere_collision(
            query, buffer, tensor.to_device([1.0]), tensor.to_device([0.0])
        )
        .flatten()
        .tolist()
    )
    if not (costs[0] > 0 and costs[1] == 0):
        raise RuntimeError(f"Unexpected cuRobo collision results: {costs}")
    metadata = (
        json.loads(((world_cache or cache) / "manifest.json").read_text())
        if (world_cache or cache)
        else {}
    )
    primitive_snapshot = (
        metadata.get("mode") == "moveit_primitives"
        or metadata.get("model_variant") == "freecad_kinematic_saved_pose"
    )
    return {
        "model": "freecad_kinematic_saved_pose"
        if primitive_snapshot
        else "freecad_final",
        "collision_objects": len(meshes) + len(cuboids),
        "mesh_objects": len(meshes),
        "native_cuboids": len(cuboids),
        "mesh_query_distance_m": mesh_query_distance(meshes, cuboids),
        "triangles": sum(len(mesh.faces) for mesh in meshes.values()),
        "lidar_surface_collision": True,
        "far_probe_clear": True,
        "device": str(tensor.device),
        "indexed_collision": cache is not None,
        "preassembled_world": world_cache is not None,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("description_package", type=Path)
    parser.add_argument("--collision-cache", type=Path)
    parser.add_argument(
        "--world-cache", type=Path, help="Read preassembled static link meshes"
    )
    parser.add_argument(
        "--write-world-cache", type=Path, help="Bake link meshes without running CUDA"
    )
    parser.add_argument(
        "--mesh-boxes", action="store_true", help="Bake boxes as meshes for comparison"
    )
    args = parser.parse_args()
    if args.mesh_boxes and not args.write_world_cache:
        parser.error("--mesh-boxes requires --write-world-cache")
    if args.write_world_cache:
        if args.world_cache:
            parser.error("--write-world-cache cannot be combined with --world-cache")
        result = write_world_cache(
            args.description_package,
            args.write_world_cache,
            args.collision_cache,
            mesh_boxes=args.mesh_boxes,
        )
        print(json.dumps({key: result[key] for key in ("triangles", "bytes")}))
    else:
        print(
            json.dumps(
                check_cuda(
                    args.description_package, args.collision_cache, args.world_cache
                )
            )
        )
