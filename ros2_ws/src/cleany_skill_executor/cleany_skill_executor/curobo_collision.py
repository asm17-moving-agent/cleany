"""Conservative convex-envelope validation, batched on CUDA via NVIDIA Warp.

cuRobo's sparse spheres guide optimization. This independent GPU acceptance
test encloses every source collision mesh, including the base. Face-plane
separation is sufficient to prove separation; omitting edge-cross axes can
only cause conservative rejections, never turn overlap into separation.
This is discrete state validation, not a continuous-collision certificate.
"""

from dataclasses import dataclass
import itertools

import numpy as np


@dataclass(frozen=True)
class ConvexBody:
    owner: str
    vertices: np.ndarray
    planes: np.ndarray


def convex_body(owner: str, vertices) -> ConvexBody:
    from scipy.spatial import ConvexHull

    points = np.asarray(vertices, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError(f"invalid collision vertices: {owner}")
    if np.max(np.abs(points)) > 1e10:
        raise ValueError(
            f"collision coordinates exceed float32 numerical range: {owner}"
        )
    hull = ConvexHull(points)
    # Retain outward planes and their support vertices in float64 on the host.
    # Runtime uses a positive separation margin to cover float32 roundoff.
    return ConvexBody(owner, points[hull.vertices], np.unique(hull.equations, axis=0))


class ContactPolicy:
    """Complete MoveIt boolean ACM: explicit pairs, then defaults, else deny."""

    def __init__(self, acm) -> None:
        names = list(acm.entry_names)
        if len(set(names)) != len(names) or len(acm.entry_values) != len(names):
            raise ValueError("malformed collision matrix")
        rows = [list(row.enabled) for row in acm.entry_values]
        if any(len(row) != len(names) for row in rows):
            raise ValueError("non-square collision matrix")
        if any(
            rows[i][j] != rows[j][i]
            for i in range(len(names))
            for j in range(len(names))
        ):
            raise ValueError("asymmetric collision matrix")
        self.explicit = {
            (a, b): rows[i][j] for i, a in enumerate(names) for j, b in enumerate(names)
        }
        if len(acm.default_entry_names) != len(acm.default_entry_values):
            raise ValueError("malformed collision defaults")
        if len(set(acm.default_entry_names)) != len(acm.default_entry_names):
            raise ValueError("duplicate collision defaults")
        self.defaults = dict(zip(acm.default_entry_names, acm.default_entry_values))

    def allows(self, a: str, b: str) -> bool:
        if (a, b) in self.explicit:
            return self.explicit[a, b]
        defaults = [self.defaults[n] for n in (a, b) if n in self.defaults]
        if defaults:
            return all(defaults)
        # A live ACM may re-enable even SRDF-adjacent pairs. Never fill missing
        # entries from the static SRDF in a complete scene snapshot.
        return False


def world_bodies(world: dict) -> list[ConvexBody]:
    """Conservative envelopes; concave meshes intentionally become convex."""
    import trimesh
    from scipy.spatial.transform import Rotation

    result = []
    for kind, objects in world.items():
        for key, obj in objects.items():
            if kind == "cuboid":
                vertices = (
                    np.array(list(itertools.product((-0.5, 0.5), repeat=3)))
                    * obj["dims"]
                )
            elif kind == "sphere":
                mesh = trimesh.creation.icosphere(subdivisions=2, radius=1.0)
                # Circumscribe, not inscribe, the analytic sphere.
                distance = np.min(
                    np.sum(mesh.face_normals * mesh.triangles[:, 0], axis=1)
                )
                vertices = mesh.vertices * (obj["radius"] / distance)
            elif kind == "cylinder":
                count = 32
                radius = obj["radius"] / np.cos(np.pi / count)
                vertices = np.array(
                    [
                        [radius * np.cos(a), radius * np.sin(a), z]
                        for a in np.arange(count) * 2 * np.pi / count
                        for z in (-obj["height"] / 2, obj["height"] / 2)
                    ]
                )
            elif kind == "mesh":
                vertices = np.asarray(obj["vertices"])
            else:
                raise ValueError(f"unsupported GPU envelope: {kind}")
            pose = obj["pose"]
            rotation = Rotation.from_quat(
                [pose[4], pose[5], pose[6], pose[3]]
            ).as_matrix()
            result.append(
                convex_body(key.rsplit("/", 2)[0], vertices @ rotation.T + pose[:3])
            )
    return result


class GpuCollisionValidator:
    def __init__(self, model, tensor_args, margin: float, *, obstacle_builder=world_bodies) -> None:
        import warp as wp

        self.wp = wp
        self.model = model
        self.tensor_args = tensor_args
        self.margin = margin
        self._obstacle_builder = obstacle_builder
        self._key = None

    def prepare(self, world: dict, acm) -> None:
        policy = ContactPolicy(acm)
        key = repr((world, policy.explicit, policy.defaults))
        if key == self._key:
            return
        obstacles = self._obstacle_builder(world)
        if any(b.owner in self.model.collision_links for b in obstacles):
            raise ValueError("world object ID aliases a robot collision link")
        bodies = self.model.bodies + obstacles
        robot_count = len(self.model.bodies)
        pairs = [
            (a, b)
            for a in range(robot_count)
            for b in range(a + 1, len(bodies))
            if bodies[a].owner != bodies[b].owner
            and not policy.allows(bodies[a].owner, bodies[b].owner)
        ]
        vertex_offsets = [0]
        plane_offsets = [0]
        centers = []
        radii = []
        for body in bodies:
            vertex_offsets.append(vertex_offsets[-1] + len(body.vertices))
            plane_offsets.append(plane_offsets[-1] + len(body.planes))
            center = body.vertices.mean(axis=0)
            centers.append(center)
            radii.append(np.max(np.linalg.norm(body.vertices - center, axis=1)))
        wp = self.wp
        device = str(self.tensor_args.device)
        self.vertices = wp.array(
            np.concatenate([b.vertices for b in bodies]), dtype=wp.vec3, device=device
        )
        self.planes = wp.array(
            np.concatenate([b.planes for b in bodies]), dtype=wp.vec4, device=device
        )
        self.vo = wp.array(vertex_offsets, dtype=int, device=device)
        self.po = wp.array(plane_offsets, dtype=int, device=device)
        self.centers = wp.array(centers, dtype=wp.vec3, device=device)
        self.radii = wp.array(radii, dtype=float, device=device)
        magnitude = max(float(np.max(np.abs(b.vertices))) for b in bodies)
        self.effective_margin = max(
            self.margin, 32 * np.finfo(np.float32).eps * (1 + magnitude)
        )
        self.pairs = wp.array(
            np.asarray(pairs, dtype=np.int32).reshape(-1, 2),
            dtype=wp.vec2i,
            device=device,
        )
        self.body_links = [
            self.model.collision_links.index(b.owner) for b in self.model.bodies
        ]
        self.body_links += [len(self.model.collision_links)] * (
            len(bodies) - robot_count
        )
        self._key = key

    def valid(self, fk_state):
        import torch
        from cleany_skill_executor.curobo_collision_kernel import convex_pairs

        wp = self.wp
        batch = fk_state.links_position.shape[0]
        xyz = torch.cat(
            (
                fk_state.links_position,
                torch.zeros((batch, 1, 3), device=self.tensor_args.device),
            ),
            dim=1,
        )
        identity = torch.zeros((batch, 1, 4), device=self.tensor_args.device)
        identity[:, :, 0] = 1
        wxyz = torch.cat((fk_state.links_quaternion, identity), dim=1)
        indices = torch.tensor(self.body_links, device=self.tensor_args.device)
        xyz = xyz[:, indices].contiguous()
        xyzw = wxyz[:, indices][:, :, [1, 2, 3, 0]].contiguous()
        collisions = torch.zeros(
            batch, dtype=torch.int32, device=self.tensor_args.device
        )
        wp.launch(
            convex_pairs,
            dim=(batch, len(self.pairs)),
            inputs=[
                self.vertices,
                self.planes,
                self.vo,
                self.po,
                self.centers,
                self.radii,
                self.pairs,
                wp.from_torch(xyz, dtype=wp.vec3),
                wp.from_torch(xyzw, dtype=wp.quat),
                self.effective_margin,
                wp.from_torch(collisions),
            ],
            device=str(self.tensor_args.device),
            stream=wp.stream_from_torch(torch.cuda.current_stream()),
        )
        return collisions == 0
