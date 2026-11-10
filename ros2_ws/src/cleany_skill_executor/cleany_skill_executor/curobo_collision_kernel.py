"""CUDA-only conservative separating-plane test. No ROS/CPU collision calls."""

import warp as wp


@wp.kernel
def convex_pairs(
    vertices: wp.array(dtype=wp.vec3),
    planes: wp.array(dtype=wp.vec4),
    vo: wp.array(dtype=int),
    po: wp.array(dtype=int),
    centers: wp.array(dtype=wp.vec3),
    radii: wp.array(dtype=float),
    pairs: wp.array(dtype=wp.vec2i),
    positions: wp.array2d(dtype=wp.vec3),
    rotations: wp.array2d(dtype=wp.quat),
    margin: float,
    collisions: wp.array(dtype=int),
):
    batch, pair = wp.tid()
    a = pairs[pair][0]
    b = pairs[pair][1]
    ca = wp.quat_rotate(rotations[batch, a], centers[a]) + positions[batch, a]
    cb = wp.quat_rotate(rotations[batch, b], centers[b]) + positions[batch, b]
    if wp.length(ca - cb) > radii[a] + radii[b] + margin:
        return
    # Test both polyhedra's outward face planes. A certified separating face
    # suffices. Missing edge-cross axes means false positives, not negatives.
    for side in range(2):
        source = a
        target = b
        if side == 1:
            source = b
            target = a
        for face in range(po[source], po[source + 1]):
            plane = planes[face]
            normal = wp.quat_rotate(
                rotations[batch, source], wp.vec3(plane[0], plane[1], plane[2])
            )
            offset = plane[3] - wp.dot(normal, positions[batch, source])
            outside = bool(True)
            for index in range(vo[target], vo[target + 1]):
                point = (
                    wp.quat_rotate(rotations[batch, target], vertices[index])
                    + positions[batch, target]
                )
                if wp.dot(normal, point) + offset <= margin:
                    outside = False
                    break
            if outside:
                return
    wp.atomic_max(collisions, batch, 1)
