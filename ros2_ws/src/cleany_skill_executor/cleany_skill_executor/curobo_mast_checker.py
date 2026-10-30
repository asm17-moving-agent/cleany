"""Add arm-versus-mast cost without turning head mounting contact into an obstacle.

The ordinary world checker still receives every proposal sphere, including the
head. A separate cuboid query receives only arm spheres. Original full-robot
MoveIt/GPU validation remains responsible for accepting states and paths.
"""
from __future__ import annotations

import torch
from curobo.geom.sdf.world import CollisionQueryBuffer, WorldCollisionConfig, WorldPrimitiveCollision
from curobo.geom.types import WorldConfig


class MastWorldCollision:
    """Per-layout view over a shared scene checker; safe for captured CUDA graphs."""

    def __init__(self, environment, cuboids: dict, kinematics: dict, tensor_args):
        self.environment = environment
        self.tensor_args = tensor_args
        names = [name for name in kinematics['collision_link_names']
                 for _ in kinematics['collision_spheres'][name]]
        self.sphere_count = len(names)
        self.excluded = torch.tensor([i for i, name in enumerate(names)
                                      if not name.startswith(('left_', 'right_'))],
                                     dtype=torch.long, device=tensor_args.device)
        self.mast = WorldPrimitiveCollision(WorldCollisionConfig(
            tensor_args=tensor_args, world_model=WorldConfig.from_dict({'cuboid': cuboids}),
            cache={'obb': len(cuboids)}))

    def __getattr__(self, name):
        return getattr(self.environment, name)

    @property
    def collision_types(self):
        return {**self.environment.collision_types, 'primitive': True}

    def _query(self, method, query, buffer, *args, **kwargs):
        if query.shape[-2] != self.sphere_count:
            raise ValueError('Mast checker sphere layout differs from robot configuration')
        ordinary = getattr(self.environment, method)(query, buffer, *args, **kwargs)
        # A negative radius is cuRobo's supported inactive-sphere convention.
        # Clone avoids mutating FK tensors used by the ordinary world query.
        masked = query.clone()
        masked[..., 3].index_fill_(-1, self.excluded, -1.)
        if not hasattr(buffer, '_cleany_mast_buffers'):
            buffer._cleany_mast_buffers = {}
        if id(self) not in buffer._cleany_mast_buffers:
            buffer._cleany_mast_buffers[id(self)] = CollisionQueryBuffer.initialize_from_shape(
                query.shape, self.tensor_args, {'primitive': True})
        mast_buffer = buffer._cleany_mast_buffers[id(self)]
        mast_buffer.update_buffer_shape(query.shape, self.tensor_args, {'primitive': True})
        extra = getattr(self.mast, method)(masked, mast_buffer, *args, **kwargs)
        if kwargs.get('compute_esdf', False):
            # Disabled spheres must not replace the ordinary scene's ESDF.
            extra = extra.clone()
            extra.index_fill_(-1, self.excluded, -torch.inf)
            return torch.maximum(ordinary, extra)
        return ordinary + extra

    def get_sphere_distance(self, query_sphere, collision_query_buffer, *args, **kwargs):
        return self._query('get_sphere_distance', query_sphere, collision_query_buffer, *args, **kwargs)

    def get_sphere_collision(self, query_sphere, collision_query_buffer, *args, **kwargs):
        return self._query('get_sphere_collision', query_sphere, collision_query_buffer, *args, **kwargs)

    def get_swept_sphere_distance(self, query_sphere, collision_query_buffer, *args, **kwargs):
        return self._query('get_swept_sphere_distance', query_sphere, collision_query_buffer, *args, **kwargs)

    def get_swept_sphere_collision(self, query_sphere, collision_query_buffer, *args, **kwargs):
        return self._query('get_swept_sphere_collision', query_sphere, collision_query_buffer, *args, **kwargs)
