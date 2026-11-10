"""선택적 관측 충돌 메시의 정확한 출처를 보관하는 크기 제한 캐시입니다."""
from collections import Counter, OrderedDict
from copy import deepcopy
import threading

import numpy as np


def candidate_bounding_radius(candidate, cache=None) -> float:
    """OBB와 출처가 정확히 일치하는 관측 로컬 메시를 모두 둘러쌉니다."""
    size = candidate.target_object.obb_size
    dimensions = np.array((size.x, size.y, size.z), dtype=float)
    if not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
        raise ValueError('Bounding radius requires positive finite OBB dimensions')
    radius = float(np.linalg.norm(dimensions) / 2)
    if cache is not None:
        geometry = cache.get(candidate)
        if geometry is None:
            raise ValueError('Missing matching collision geometry for bounding radius')
        radius = max(radius, max(float(np.linalg.norm((v.x, v.y, v.z)))
                                 for v in geometry.mesh.vertices))
    return radius


def subscribe_collision_geometry(node, cache, *, callback_group=None, record=False):
    from cleany_interfaces.msg import ObservedObjectGeometry
    from rclpy.qos import DurabilityPolicy, QoSProfile
    def receive(message):
        try:
            cache.put(message)
        except ValueError as error:
            node.get_logger().warning(f'Rejected observed collision geometry: {error}')
            return
        if record and getattr(node, '_artifact_directory', None) is not None:
            node._record_pipeline_message('collision_geometry', message)
    return node.create_subscription(ObservedObjectGeometry,
        str(node.get_parameter('collision_geometry_topic').value), receive,
        QoSProfile(depth=16, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        callback_group=callback_group)


class CollisionGeometryCache:
    """검증된 닫힌 볼록 메시 복사본을 크기 제한 안에서 콜백 간 보관합니다.

    조회에는 snapshot 및 물체 식별, header와 OBB 자세가 일치해야 합니다.
    출처가 없거나 다르면 다른 메시로 대체하지 않고 None을 반환합니다.
    """
    def __init__(self, capacity: int = 64) -> None:
        if capacity < 1:
            raise ValueError('Geometry cache capacity must be positive')
        self._capacity = capacity
        self._values = OrderedDict()
        self._lock = threading.Lock()

    def put(self, message) -> None:
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        if not message.snapshot_id or message.object_id <= 0 or not message.header.frame_id or stamp <= 0:
            raise ValueError('Invalid collision geometry provenance')
        vertices = np.array([(p.x,p.y,p.z) for p in message.mesh.vertices])
        faces = np.array([list(t.vertex_indices) for t in message.mesh.triangles], dtype=int)
        p, q = message.mesh_pose.position, message.mesh_pose.orientation
        if (not 6 <= len(vertices) <= 512 or not 8 <= len(faces) <= 2048
                or vertices.shape != (len(vertices),3) or faces.shape != (len(faces),3)
                or not np.isfinite(vertices).all()
                or not np.isfinite((p.x,p.y,p.z,q.x,q.y,q.z,q.w)).all()
                or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > 1e-5
                or faces.min() < 0 or faces.max() >= len(vertices)):
            raise ValueError('Invalid collision geometry mesh or pose')
        edges = Counter((int(a),int(b)) for face in faces for a,b in zip(face,np.roll(face,-1)))
        if any(a == b or count != 1 or edges[b,a] != 1 for (a,b),count in edges.items()):
            raise ValueError('Collision geometry must be a closed oriented surface')
        for indices in faces:
            triangle = vertices[indices]
            normal = np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0])
            length = np.linalg.norm(normal)
            if length <= 1e-14 or np.max((vertices-triangle[0]) @ normal/length) > 1e-7:
                raise ValueError('Collision geometry must be nondegenerate and convex')
        with self._lock:
            key = message.snapshot_id, message.object_id
            self._values[key] = deepcopy(message)
            self._values.move_to_end(key)
            while len(self._values) > self._capacity:
                self._values.popitem(last=False)

    def get(self, candidate):
        with self._lock:
            message = self._values.get((candidate.snapshot_id, candidate.object_id))
            if message is None:
                return None
            if message.header != candidate.header:
                return None
            expected = candidate.target_object.obb_pose
            if message.mesh_pose != expected:
                return None
            return deepcopy(message)
