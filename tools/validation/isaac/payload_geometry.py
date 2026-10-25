"""Bounds of the authored collision geometry for the independent outcome oracle."""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PayloadGeometry:
    vertices: np.ndarray
    bottom: tuple[float, float, float] | None = None

    @classmethod
    def from_payload(cls, payload: dict) -> 'PayloadGeometry':
        from cleany_mujoco_sim.tabletop_shapes import TabletopShape, cup_vertices, tissue_vertices
        size = tuple(payload['collision']['size_m'])
        shape = TabletopShape(payload['visual_type'], payload['geometry'])
        height = size[-1]
        if shape.kind == 'paper_cup':
            vertices, _ = cup_vertices(size, shape, collision=True)
            thickness = shape.number('wall_thickness_m')
            bottom = (shape.number('bottom_diameter_m')/2,
                      thickness/2, -height/2+thickness/2)
        else:
            vertices, _ = tissue_vertices(size, shape)
            bottom = None
        points = np.asarray(vertices, dtype=float)
        points[:, 2] -= height/2
        return cls(points, bottom)

    def bounds(self, center: np.ndarray, rotation: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return base-frame extrema of the collision vertices and bottom cylinder.

        rotation maps payload-local column vectors into the base frame. This
        reads simulator pose only for outcome assessment, never grasp planning.
        """
        points = self.vertices @ rotation.T + center
        low, high = points.min(axis=0), points.max(axis=0)
        if self.bottom is not None:
            radius, half_height, local_z = self.bottom
            disk_center = center + rotation[:, 2]*local_z
            extent = radius*np.linalg.norm(rotation[:, :2], axis=1) + half_height*np.abs(rotation[:, 2])
            low = np.minimum(low, disk_center-extent)
            high = np.maximum(high, disk_center+extent)
        return low, high
