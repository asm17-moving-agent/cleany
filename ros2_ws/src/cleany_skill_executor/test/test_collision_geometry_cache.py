from copy import deepcopy

import numpy as np
import pytest
from geometry_msgs.msg import Point
from shape_msgs.msg import MeshTriangle
from cleany_interfaces.msg import GraspCandidate, ObservedObjectGeometry
from cleany_grasping.core.collision_geometry import observed_convex_prism
from cleany_skill_executor.collision_geometry_cache import CollisionGeometryCache, candidate_bounding_radius


def test_radius_encloses_mesh_beyond_trimmed_obb_and_keeps_obb_minimum():
    message, candidate = geometry_pair()
    size = candidate.target_object.obb_size
    size.x = size.y = size.z = .01
    cache = CollisionGeometryCache()
    cache.put(message)
    assert candidate_bounding_radius(candidate, cache) == pytest.approx(np.sqrt(.04**2+.02**2+.01**2))
    size.x = size.y = size.z = .1
    assert candidate_bounding_radius(candidate, cache) == pytest.approx(np.sqrt(3)*.05)
    assert candidate_bounding_radius(candidate) == pytest.approx(np.sqrt(3)*.05)
    candidate.header.stamp.sec += 1
    with pytest.raises(ValueError, match='Missing matching'):
        candidate_bounding_radius(candidate, cache)


def geometry_pair():
    message = ObservedObjectGeometry(snapshot_id='snapshot', object_id=2)
    message.header.frame_id = 'base_link'
    message.header.stamp.sec = 10
    message.mesh_pose.position.x = .5
    points = np.array([(x,y,.01) for x in (-.04,.04) for y in (-.02,.02)])
    prism = observed_convex_prism(points, np.zeros(3), np.eye(3), .02)
    message.mesh.vertices = [Point(x=float(x), y=float(y), z=float(z)) for x,y,z in prism.vertices]
    message.mesh.triangles = [MeshTriangle(vertex_indices=list(map(int,t))) for t in prism.triangles]
    candidate = GraspCandidate(snapshot_id='snapshot', object_id=2)
    candidate.header = deepcopy(message.header)
    candidate.target_object.obb_pose = deepcopy(message.mesh_pose)
    return message, candidate


def test_cache_requires_exact_capture_identity_and_returns_independent_copy():
    message, candidate = geometry_pair()
    cache = CollisionGeometryCache()
    cache.put(message)
    result = cache.get(candidate)
    assert result == message
    result.mesh.vertices[0].x = 99.
    message.mesh.vertices[0].x = 88.
    assert abs(cache.get(candidate).mesh.vertices[0].x) < 1
    for attribute in ('snapshot_id', 'object_id'):
        wrong = deepcopy(candidate)
        setattr(wrong, attribute, 'other' if attribute == 'snapshot_id' else 3)
        assert cache.get(wrong) is None
    wrong = deepcopy(candidate)
    wrong.header.stamp.sec += 1
    assert cache.get(wrong) is None
    wrong = deepcopy(candidate)
    wrong.header.frame_id = 'map'
    assert cache.get(wrong) is None
    wrong = deepcopy(candidate)
    wrong.target_object.obb_pose.position.x += .001
    assert cache.get(wrong) is None


@pytest.mark.parametrize('fault', ['nan', 'open', 'reverse', 'bad_index', 'stamp', 'quaternion'])
def test_invalid_geometry_is_not_cached(fault):
    message, candidate = geometry_pair()
    if fault == 'nan': message.mesh.vertices[0].x = float('nan')
    elif fault == 'open': message.mesh.triangles.pop()
    elif fault == 'reverse':
        for triangle in message.mesh.triangles:
            triangle.vertex_indices = list(map(int, reversed(triangle.vertex_indices)))
    elif fault == 'bad_index': message.mesh.triangles[0].vertex_indices = [0,1,999]
    elif fault == 'stamp': message.header.stamp.sec = 0
    elif fault == 'quaternion': message.mesh_pose.orientation.w = 0.
    cache = CollisionGeometryCache()
    with pytest.raises(ValueError): cache.put(message)
    assert cache.get(candidate) is None


def test_cache_evicts_old_entries():
    message, candidate = geometry_pair()
    cache = CollisionGeometryCache(capacity=1)
    cache.put(message)
    message.object_id = 3
    cache.put(message)
    assert cache.get(candidate) is None
