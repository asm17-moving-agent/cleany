from __future__ import annotations

import math
import struct

import numpy as np
import rclpy
from cleany_interfaces.msg import GraspCandidate
from cleany_interfaces.srv import PlanGrasp
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from cleany_grasping.anygrasp_adapter import AnyGraspPredictor, ModelUnavailableError
from cleany_grasping.core.models import PointCloud
from cleany_grasping.core.selector import GraspConfig, rank_grasps
from cleany_grasping.debug_image import debug_image_message, render_grasp_debug_image
from cleany_grasping.geometric_predictor import (
    GeometricGraspConfig,
    GeometricGraspPredictor,
)


def _quaternion_from_rotation(matrix: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        values = (
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        permutations = ((0, 1, 2), (1, 2, 0), (2, 0, 1))
        i, j, k = permutations[index]
        scale = math.sqrt(1.0 + matrix[i, i] - matrix[j, j] - matrix[k, k]) * 2.0
        quaternion = [0.0, 0.0, 0.0, (matrix[k, j] - matrix[j, k]) / scale]
        quaternion[i] = 0.25 * scale
        quaternion[j] = (matrix[i, j] + matrix[j, i]) / scale
        quaternion[k] = (matrix[i, k] + matrix[k, i]) / scale
        values = tuple(quaternion)
    norm = math.sqrt(sum(value * value for value in values))
    return tuple(value / norm for value in values)


def _rotation_from_quaternion(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        )
    )


def point_cloud_from_message(message: PointCloud2) -> PointCloud:
    fields = {field.name: field.offset for field in message.fields}
    if not {'x', 'y', 'z', 'rgb'} <= fields.keys() or message.point_step <= 0:
        raise ValueError('PointCloud2 must contain x, y, z, and rgb fields')
    endian = '>' if message.is_bigendian else '<'
    points, colors = [], []
    payload = bytes(message.data)
    for row in range(message.height):
        for column in range(message.width):
            offset = row * message.row_step + column * message.point_step
            xyz = tuple(
                struct.unpack_from(endian + 'f', payload, offset + fields[name])[0]
                for name in ('x', 'y', 'z')
            )
            packed = struct.unpack_from(endian + 'I', payload, offset + fields['rgb'])[0]
            if np.isfinite(xyz).all():
                points.append(xyz)
                colors.append(((packed >> 16) & 255, (packed >> 8) & 255, packed & 255))
    return PointCloud(np.asarray(points).reshape((-1, 3)), np.asarray(colors).reshape((-1, 3)) / 255.0)


class GraspNode(Node):
    def __init__(self, predictor=None, **kwargs) -> None:
        super().__init__('grasp_server', **kwargs)
        self.declare_parameter('service_name', 'grasp/plan')
        self.declare_parameter('planning_frame', 'base_link')
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('license_path', '')
        self.declare_parameter('predictor_type', 'geometric')
        self.declare_parameter('debug_image_topic', 'grasp/debug_image')
        self.declare_parameter('maximum_gripper_width_m', 0.10)
        self.declare_parameter('workspace_margin_m', 0.04)
        self.declare_parameter('target_contact_margin_m', 0.015)
        self.declare_parameter('geometric.opening_margin_m', 0.008)
        self.declare_parameter('geometric.grasp_depth_m', 0.025)
        self.declare_parameter('geometric.finger_thickness_m', 0.010)
        self.declare_parameter('geometric.finger_length_m', 0.045)
        self.declare_parameter('geometric.palm_depth_m', 0.018)
        self.declare_parameter('geometric.collision_clearance_m', 0.003)
        self.declare_parameter('geometric.plane_distance_threshold_m', 0.006)
        self.declare_parameter('geometric.plane_ransac_iterations', 160)
        self.declare_parameter('geometric.extent_trim_percentile', 0.5)
        self.declare_parameter('geometric.axis_search_step_degrees', 1.0)
        self.declare_parameter(
            'geometric.yaw_offsets_degrees', [-20.0, -10.0, 0.0, 10.0, 20.0]
        )
        self.declare_parameter('geometric.maximum_candidates', 12)
        self.declare_parameter('canonical_to_tcp_rotation', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('tcp_approach_axis', [1.0, 0.0, 0.0])
        self._predictor = predictor or self._create_predictor()
        self._debug_publisher = self.create_publisher(
            Image,
            str(self.get_parameter('debug_image_topic').value),
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._service = self.create_service(
            PlanGrasp, str(self.get_parameter('service_name').value), self._plan
        )

    def _plan(self, request, response):
        response.candidates = []
        try:
            self._validate(request)
            target = point_cloud_from_message(request.target_cloud)
            context = point_cloud_from_message(request.context_cloud)
            config = GraspConfig(
                workspace_margin_m=float(self.get_parameter('workspace_margin_m').value),
                target_contact_margin_m=float(self.get_parameter('target_contact_margin_m').value),
                maximum_gripper_width_m=float(self.get_parameter('maximum_gripper_width_m').value),
                canonical_to_tcp_rotation=np.asarray(self.get_parameter('canonical_to_tcp_rotation').value).reshape((3, 3)),
                tcp_approach_axis=np.asarray(self.get_parameter('tcp_approach_axis').value),
            )
            target_min = target.points.min(axis=0)
            target_max = target.points.max(axis=0)
            workspace = np.column_stack(
                (
                    target_min - config.workspace_margin_m,
                    target_max + config.workspace_margin_m,
                )
            ).reshape(-1)
            candidates = self._predictor.predict(target, context, workspace)
            ranked = rank_grasps(candidates, target, config)
            grasp = ranked[0] if ranked else None
            self._publish_debug(
                target,
                context,
                candidates,
                self._selected_raw_index(candidates, grasp),
                request.context_cloud,
            )
            if grasp is None:
                return self._fail(response, response.ERROR_NO_GRASP_CANDIDATE, 'No valid grasp candidate')
            for grasp in ranked:
                rotation, translation, approach = self._to_planning_frame(
                    grasp.rotation,
                    grasp.translation,
                    grasp.approach_direction,
                    request.context_cloud,
                )
                candidate = GraspCandidate()
                candidate.header.stamp = request.context_cloud.header.stamp
                candidate.header.frame_id = str(self.get_parameter('planning_frame').value)
                candidate.snapshot_id = request.snapshot_id
                candidate.object_id = request.object_id
                candidate.target_object = request.target_object
                candidate.tcp_pose.position.x, candidate.tcp_pose.position.y, candidate.tcp_pose.position.z = translation
                quaternion = _quaternion_from_rotation(rotation)
                candidate.tcp_pose.orientation.x, candidate.tcp_pose.orientation.y, candidate.tcp_pose.orientation.z, candidate.tcp_pose.orientation.w = quaternion
                candidate.approach_direction.x, candidate.approach_direction.y, candidate.approach_direction.z = approach
                candidate.required_opening_m = grasp.required_opening_m
                candidate.grasp_depth_m = grasp.depth_m
                candidate.score = grasp.score
                response.candidates.append(candidate)
            response.success = True
            response.error_code = response.ERROR_NONE
            response.message = f'Generated {len(response.candidates)} grasp candidates'
            return response
        except ModelUnavailableError as error:
            return self._fail(response, response.ERROR_MODEL_UNAVAILABLE, str(error))
        except (ValueError, TransformException) as error:
            return self._fail(response, response.ERROR_INVALID_INPUT, str(error))
        except Exception as error:
            return self._fail(response, response.ERROR_INTERNAL, f'Unexpected grasp error: {error}')

    def _create_predictor(self):
        predictor_type = str(self.get_parameter('predictor_type').value).strip().lower()
        if predictor_type == 'anygrasp':
            return AnyGraspPredictor(
                str(self.get_parameter('checkpoint_path').value),
                str(self.get_parameter('license_path').value),
            )
        if predictor_type == 'geometric':
            return GeometricGraspPredictor(
                GeometricGraspConfig(
                    maximum_gripper_width_m=float(
                        self.get_parameter('maximum_gripper_width_m').value
                    ),
                    opening_margin_m=float(
                        self.get_parameter('geometric.opening_margin_m').value
                    ),
                    grasp_depth_m=float(
                        self.get_parameter('geometric.grasp_depth_m').value
                    ),
                    finger_thickness_m=float(
                        self.get_parameter('geometric.finger_thickness_m').value
                    ),
                    finger_length_m=float(
                        self.get_parameter('geometric.finger_length_m').value
                    ),
                    palm_depth_m=float(
                        self.get_parameter('geometric.palm_depth_m').value
                    ),
                    collision_clearance_m=float(
                        self.get_parameter('geometric.collision_clearance_m').value
                    ),
                    plane_distance_threshold_m=float(
                        self.get_parameter(
                            'geometric.plane_distance_threshold_m'
                        ).value
                    ),
                    plane_ransac_iterations=int(
                        self.get_parameter('geometric.plane_ransac_iterations').value
                    ),
                    extent_trim_percentile=float(
                        self.get_parameter('geometric.extent_trim_percentile').value
                    ),
                    axis_search_step_degrees=float(
                        self.get_parameter('geometric.axis_search_step_degrees').value
                    ),
                    yaw_offsets_degrees=tuple(
                        float(value)
                        for value in self.get_parameter(
                            'geometric.yaw_offsets_degrees'
                        ).value
                    ),
                    maximum_candidates=int(
                        self.get_parameter('geometric.maximum_candidates').value
                    ),
                )
            )
        raise ValueError(
            f'Unsupported predictor_type {predictor_type!r}; use geometric or anygrasp'
        )

    @staticmethod
    def _selected_raw_index(candidates, grasp) -> int | None:
        if grasp is None:
            return None
        for index, candidate in enumerate(candidates):
            if math.isclose(candidate.score, grasp.score) and np.allclose(
                candidate.translation, grasp.translation
            ):
                return index
        return None

    def _publish_debug(
        self,
        target: PointCloud,
        context: PointCloud,
        candidates,
        selected_index: int | None,
        cloud_message: PointCloud2,
    ) -> None:
        rendered = render_grasp_debug_image(
            target,
            context,
            candidates,
            selected_index=selected_index,
        )
        self._debug_publisher.publish(
            debug_image_message(
                rendered,
                Time.from_msg(cloud_message.header.stamp).nanoseconds,
                cloud_message.header.frame_id,
            )
        )

    @staticmethod
    def _fail(response, code, message):
        response.success, response.error_code, response.message = False, code, message
        return response

    @staticmethod
    def _validate(request) -> None:
        if not request.snapshot_id or request.object_id == 0:
            raise ValueError('snapshot_id and object_id are required')
        if request.target_object.object_id != request.object_id:
            raise ValueError('target object ID does not match request object_id')
        left, right = request.target_cloud.header, request.context_cloud.header
        if left.frame_id != right.frame_id or left.stamp != right.stamp:
            raise ValueError('Target and context clouds must share frame and timestamp')

    def _to_planning_frame(self, rotation, translation, approach, cloud_message):
        target_frame = str(self.get_parameter('planning_frame').value)
        source_frame = cloud_message.header.frame_id
        if target_frame == source_frame:
            return rotation, translation, approach
        transform = self._tf_buffer.lookup_transform(
            target_frame, source_frame, Time.from_msg(cloud_message.header.stamp), timeout=Duration(seconds=0.5)
        ).transform
        q = transform.rotation
        frame_rotation = _rotation_from_quaternion(q.x, q.y, q.z, q.w)
        frame_translation = np.array((transform.translation.x, transform.translation.y, transform.translation.z))
        return (
            frame_rotation @ rotation,
            frame_rotation @ translation + frame_translation,
            frame_rotation @ approach,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
