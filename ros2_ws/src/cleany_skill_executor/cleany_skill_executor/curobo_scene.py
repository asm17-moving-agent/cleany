"""Convert a complete MoveIt world snapshot without silently losing geometry."""

from __future__ import annotations

import math

from shape_msgs.msg import SolidPrimitive


def world_from_scene(
    scene, base_frame: str = "base_link", *, optimization: bool = False
) -> dict:
    if scene.robot_state.attached_collision_objects:
        raise ValueError("cuRobo attached objects are not supported yet")
    if scene.world.octomap.octomap.data:
        raise ValueError("cuRobo OctoMap conversion is not supported yet")
    world = {"cuboid": {}, "sphere": {}, "cylinder": {}, "mesh": {}}
    acm = scene.allowed_collision_matrix
    allowed_objects = set()
    for name, entry in zip(acm.entry_names, acm.entry_values):
        if any(entry.enabled):
            allowed_objects.add(name)
    allowed_objects.update(
        name
        for name, enabled in zip(acm.default_entry_names, acm.default_entry_values)
        if enabled
    )

    def pose_values(pose):
        p, q = pose.position, pose.orientation
        values = [p.x, p.y, p.z, q.w, q.x, q.y, q.z]
        if not all(math.isfinite(v) for v in values):
            raise ValueError("non-finite world pose")
        norm = math.sqrt(sum(v * v for v in values[3:]))
        if norm < 1e-9:
            raise ValueError("zero world quaternion")
        return values[:3] + [v / norm for v in values[3:]]

    for obj in scene.world.collision_objects:
        if obj.header.frame_id not in ("", base_frame):
            raise ValueError(f"world object {obj.id} is not expressed in {base_frame}")
        if (
            obj.planes
            or len(obj.primitives) != len(obj.primitive_poses)
            or len(obj.meshes) != len(obj.mesh_poses)
        ):
            raise ValueError(f"unsupported or incomplete collision geometry: {obj.id}")
        # Sparse-sphere optimization cannot apply object/link ACM pairs.
        # Only omit these objects from that cost; the GPU convex acceptance
        # validator always receives them and applies the exact boolean pairs.
        if optimization and obj.id in allowed_objects:
            continue
        for index, (primitive, pose) in enumerate(
            zip(obj.primitives, obj.primitive_poses)
        ):
            key = f"{obj.id}/primitive/{index}"
            dims = list(primitive.dimensions)
            if not all(math.isfinite(v) and v > 0 for v in dims):
                raise ValueError(f"invalid world dimensions: {obj.id}")
            common = {"pose": pose_values(pose)}
            if primitive.type == SolidPrimitive.BOX and len(dims) == 3:
                world["cuboid"][key] = dict(common, dims=dims)
            elif primitive.type == SolidPrimitive.SPHERE and len(dims) == 1:
                world["sphere"][key] = dict(common, radius=dims[0])
            elif primitive.type == SolidPrimitive.CYLINDER and len(dims) == 2:
                world["cylinder"][key] = dict(common, height=dims[0], radius=dims[1])
            else:
                raise ValueError(f"unsupported world primitive: {obj.id}")
        for index, (mesh, pose) in enumerate(zip(obj.meshes, obj.mesh_poses)):
            if (
                not mesh.vertices
                or not mesh.triangles
                or any(
                    i >= len(mesh.vertices)
                    for triangle in mesh.triangles
                    for i in triangle.vertex_indices
                )
            ):
                raise ValueError(f"invalid world mesh: {obj.id}")
            world["mesh"][f"{obj.id}/mesh/{index}"] = {
                "pose": pose_values(pose),
                "vertices": [[v.x, v.y, v.z] for v in mesh.vertices],
                "faces": [list(t.vertex_indices) for t in mesh.triangles],
            }
    if sum(len(values) for values in world.values()) > 128:
        raise ValueError("cuRobo world exceeds the 128-object cache")
    return world
