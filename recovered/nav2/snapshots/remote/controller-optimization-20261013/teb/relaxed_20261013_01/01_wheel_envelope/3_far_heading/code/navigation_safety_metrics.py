"""Planar clearance of the configured padded base and a box test fixture."""

from __future__ import annotations

import math


def rectangle_clearance(
    robot_pose: tuple[float, float, float],
    robot_half_size: tuple[float, float],
    obstacle_xy: tuple[float, float],
    obstacle_half_size: tuple[float, float],
) -> float:
    """Return metres between rectangles; zero includes contact/overlap.

    The obstacle is aligned with the evaluation map. This is a footprint
    metric, not a Gazebo contact-sensor measurement or a 3-D arm check.
    """
    def vertices(center, half_size, angle):
        c, s = math.cos(angle), math.sin(angle)
        return [(center[0]+c*x-s*y, center[1]+s*x+c*y)
                for x, y in [(-half_size[0], -half_size[1]), (half_size[0], -half_size[1]),
                             (half_size[0], half_size[1]), (-half_size[0], half_size[1])]]

    a = vertices(robot_pose[:2], robot_half_size, robot_pose[2])
    b = vertices(obstacle_xy, obstacle_half_size, 0.0)
    separated = False
    for polygon in (a, b):
        for p, q in zip(polygon, polygon[1:]+polygon[:1]):
            axis = (p[1]-q[1], q[0]-p[0])
            pa = [v[0]*axis[0]+v[1]*axis[1] for v in a]
            pb = [v[0]*axis[0]+v[1]*axis[1] for v in b]
            separated |= max(pa) < min(pb) or max(pb) < min(pa)
    if not separated:
        return 0.0

    def point_segment(p, a, b):
        dx, dy = b[0]-a[0], b[1]-a[1]
        t = max(0.0, min(1.0, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
        return math.hypot(p[0]-a[0]-t*dx, p[1]-a[1]-t*dy)

    return min(point_segment(p, u, v)
               for points, edges in ((a, b), (b, a))
               for p in points for u, v in zip(edges, edges[1:]+edges[:1]))
