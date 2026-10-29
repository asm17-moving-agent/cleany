from __future__ import annotations

from copy import deepcopy
import math
from action_msgs.msg import GoalStatus, GoalStatusArray

import numpy as np
import rclpy
from cleany_interfaces.msg import GraspCandidate, ObservedObjectGeometry
from geometry_msgs.msg import Point
from shape_msgs.msg import MeshTriangle
from cleany_interfaces.srv import PlanGrasp
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener

from cleany_grasping.anygrasp_adapter import AnyGraspPredictor, ModelUnavailableError
from cleany_grasping.core.models import PointCloud
from cleany_grasping.core.collision_geometry import observed_convex_prism
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


def _transform_target_object(target_object, rotation, translation):
    """Return a copy of an object OBB expressed in the destination frame."""
    transformed = deepcopy(target_object)
    pose = transformed.obb_pose
    position = np.array(
        (pose.position.x, pose.position.y, pose.position.z), dtype=float
    )
    transformed_position = rotation @ position + translation
    pose.position.x, pose.position.y, pose.position.z = transformed_position
    source_orientation = _rotation_from_quaternion(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    quaternion = _quaternion_from_rotation(rotation @ source_orientation)
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = quaternion
    return transformed


def observed_geometry_message(points, target_object, header, snapshot_id, object_id):
    pose = target_object.obb_pose
    q, p = pose.orientation, pose.position
    mesh = observed_convex_prism(points, np.array((p.x,p.y,p.z)),
        _rotation_from_quaternion(q.x,q.y,q.z,q.w), target_object.obb_size.z)
    geometry = ObservedObjectGeometry()
    geometry.header = deepcopy(header)
    geometry.snapshot_id, geometry.object_id = snapshot_id, object_id
    geometry.mesh_pose = deepcopy(pose)
    geometry.mesh.vertices = [Point(x=float(x), y=float(y), z=float(z)) for x,y,z in mesh.vertices]
    geometry.mesh.triangles = [MeshTriangle(vertex_indices=list(map(int, face))) for face in mesh.triangles]
    return geometry


def point_cloud_from_message(message: PointCloud2) -> PointCloud:
    descriptors = {field.name: field for field in message.fields}
    if not {'x', 'y', 'z', 'rgb'} <= descriptors.keys() or message.point_step <= 0:
        raise ValueError('PointCloud2 must contain x, y, z, and rgb fields')
    for name in ('x', 'y', 'z', 'rgb'):
        field = descriptors[name]
        types = (PointField.FLOAT32, PointField.UINT32) if name == 'rgb' else (PointField.FLOAT32,)
        if (field.count != 1 or field.datatype not in types
                or field.offset < 0 or field.offset + 4 > message.point_step):
            raise ValueError('PointCloud2 fields must fit one point with float32 XYZ and packed RGB')
    if not message.height or not message.width:
        return PointCloud(np.empty((0, 3)), np.empty((0, 3)))
    if message.row_step < message.width * message.point_step:
        raise ValueError('PointCloud2 row_step is shorter than its points')
    required_bytes = message.height * message.row_step
    if len(message.data) < required_bytes:
        raise ValueError('PointCloud2 data is shorter than its declared layout')
    fields = {name: field.offset for name, field in descriptors.items()}
    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype({
        'names': ['x', 'y', 'z', 'rgb'],
        'formats': [endian + 'f4'] * 3 + [endian + 'u4'],
        'offsets': [fields[name] for name in ('x', 'y', 'z', 'rgb')],
        'itemsize': message.point_step,
    })
    records = np.ndarray(
        (message.height, message.width), dtype=dtype, buffer=message.data,
        strides=(message.row_step, message.point_step),
    )
    # Filter in wire precision, then retain the core's owned float64 arrays.
    finite = np.isfinite(records['x'])
    finite &= np.isfinite(records['y'])
    finite &= np.isfinite(records['z'])
    count = int(np.count_nonzero(finite))
    points = np.empty((count, 3), dtype=np.float64)
    colors = np.empty((count, 3), dtype=np.float64)
    for axis, name in enumerate(('x', 'y', 'z')):
        points[:, axis] = records[name][finite]
    rgb = records['rgb'][finite]
    for axis, shift in enumerate((16, 8, 0)):
        colors[:, axis] = (rgb >> shift) & 255
    colors /= 255.0
    return PointCloud(points, colors)


def _predict_and_rank(
    predictor,
    target: PointCloud,
    context: PointCloud,
    workspace: np.ndarray,
    config: GraspConfig,
):
    candidates = predictor.predict(target, context, workspace)
    ranked = rank_grasps(candidates, target, config)
    should_retry = getattr(
        predictor,
        'should_retry_full_resolution',
        None,
    )
    retry = getattr(predictor, 'predict_full_resolution', None)
    retried = False
    if (
        not ranked
        and callable(should_retry)
        and should_retry(context)
        and callable(retry)
    ):
        candidates = retry(target, context, workspace)
        ranked = rank_grasps(candidates, target, config)
        retried = True
    return candidates, ranked, retried


class GraspNode(Node):
    """Plan grasp candidates from a supplied observation without moving the robot.

    Services:
        grasp/plan (cleany_interfaces/srv/PlanGrasp): Server for candidate
            prediction and filtering; service_name default.

    Publishers:
        grasp/debug_image (sensor_msgs/msg/Image): Candidate overlay.
        /grasp/collision_geometry (cleany_interfaces/msg/ObservedObjectGeometry):
            Optional observed geometry when publish_collision_geometry is enabled.
            Both topic names are configurable defaults.

    TF:
        Resolves observation geometry into planning_frame, default base_link.

    QoS:
        Debug images and optional collision geometry are reliable and transient
        local, with depths 1 and 16 respectively.

    Notes:
        Predictor selection and model/license validation occur at startup.
        Request validation or planning failure is returned as a failed response.
    """
    def __init__(self, predictor=None, **kwargs) -> None:
        super().__init__('grasp_server', **kwargs)
        self.declare_parameter('service_name', 'grasp/plan')
        self.declare_parameter('planning_frame', 'base_link')
        self.declare_parameter('tf_cache_time_sec', 180.0)
        self.declare_parameter('optimization_profile', 'optimized')
        self.declare_parameter('cuda_cache_release_threshold_mb', 512.0)
        self.declare_parameter('cuda_gc_interval_calls', 16)
        self.declare_parameter('optimized_adaptive_maximum_points', 8000)
        self.declare_parameter('optimized_adaptive_minimum_region_points', 2000)
        for name, value in {
            'anygrasp_approach_direction': [0.0, 0.0, 0.0],
            'anygrasp_approach_threshold_degrees': 180.0,
            'anygrasp_region_steering_mode': 'workspace',
            'anygrasp_target_region_voxel_m': 0.01,
            'anygrasp_adaptive_point_policy': 'fixed',
            'anygrasp_adaptive_low_maximum_points': 4000,
            'anygrasp_adaptive_minimum_object_points': 400,
            'anygrasp_adaptive_minimum_object_extent_m': 0.015,
            'anygrasp_adaptive_maximum_context_ratio': 40.0,
        }.items():
            self.declare_parameter(name, value)
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('license_path', '')
        self.declare_parameter('predictor_type', 'geometric')
        self.declare_parameter('anygrasp_worker_enabled', True)
        self.declare_parameter('anygrasp_worker_idle_timeout_sec', 120.0)
        self.declare_parameter('anygrasp_worker_request_timeout_sec', 120.0)
        self.declare_parameter('gpu_lock_path', '')
        self.declare_parameter('anygrasp_prepare_service', '/grasp/prepare_window')
        self.declare_parameter('anygrasp_prepare_hold_sec', 180.0)
        self.declare_parameter('anygrasp_task_status_topic', '/manipulation/execute_skill/_action/status')
        self.declare_parameter('anygrasp_task_hold_sec', 600.0)
        self.declare_parameter('anygrasp_minimum_available_memory_mb', 0.0)
        for name in ('anygrasp_prepare_hold_sec', 'anygrasp_task_hold_sec'):
            value=float(self.get_parameter(name).value)
            if not math.isfinite(value) or value < 0:
                raise ValueError('AnyGrasp hold durations must be finite and nonnegative')
        self._task_hold_active = False
        self.declare_parameter('anygrasp_preload', False)
        self.declare_parameter('publish_collision_geometry', False)
        self.declare_parameter('collision_geometry_topic', '/grasp/collision_geometry')
        self.declare_parameter('debug_image_topic', 'grasp/debug_image')
        self.declare_parameter('maximum_gripper_width_m', 0.10)
        self.declare_parameter('gripper_height_m', 0.03)
        self.declare_parameter('workspace_margin_m', 0.04)
        self.declare_parameter('target_contact_margin_m', 0.015)
        self.declare_parameter('nms_rotation_threshold_degrees', 20.0)
        self.declare_parameter('geometric.opening_margin_m', 0.008)
        self.declare_parameter('geometric.grasp_depth_m', 0.025)
        self.declare_parameter('geometric.maximum_top_contact_depth_m', 0.0)
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
        self.declare_parameter('geometric.approach_tilt_degrees', 0.0)
        self.declare_parameter('geometric.search_approach_tilts', False)
        self.declare_parameter('geometric.include_reverse_closing_axis', False)
        self.declare_parameter('geometric.prefer_upward_closing_axis', False)
        self.declare_parameter('geometric.search_longitudinal_contacts', False)
        self.declare_parameter('geometric.longitudinal_offset_fractions', [0.0, -0.25, 0.25])
        self.declare_parameter('geometric.longitudinal_max_height_ratio', 0.4)
        self.declare_parameter('geometric.longitudinal_contact_height_offset_m', 0.0)
        self.declare_parameter('geometric.defer_support_plane_collision', False)
        self.declare_parameter('geometric.approach_reference_positions', [0.0])
        self.declare_parameter(
            'geometric.approach_tilt_options', [0.0, 8.0, 16.0, 24.0]
        )
        self.declare_parameter(
            'geometric.approach_tilt_direction',
            [1.0, 0.0, 0.0],
        )
        self.declare_parameter(
            'geometric.reject_robot_opposite_approach',
            False,
        )
        self.declare_parameter(
            'geometric.robot_reference_position',
            [0.0, 0.0, 0.0],
        )
        self.declare_parameter('geometric.maximum_candidates', 12)
        self.declare_parameter(
            'canonical_to_tcp_rotation',
            [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter('tcp_approach_axis', [0.0, -1.0, 0.0])
        self._predictor = predictor or self._create_predictor()
        if bool(self.get_parameter('anygrasp_preload').value):
            prepare = getattr(self._predictor, 'prepare', None)
            if prepare is not None:
                prepare()
        release_idle = getattr(self._predictor, 'release_if_idle', None)
        self._worker_idle_timer = self.create_timer(5.0, release_idle) if release_idle else None
        self._debug_publisher = self.create_publisher(
            Image,
            str(self.get_parameter('debug_image_topic').value),
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=float(self.get_parameter('tf_cache_time_sec').value)))
        self._geometry_publisher = None
        if self.get_parameter('publish_collision_geometry').value:
            self._geometry_publisher = self.create_publisher(ObservedObjectGeometry,
                str(self.get_parameter('collision_geometry_topic').value),
                QoSProfile(depth=16, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._service = self.create_service(
            PlanGrasp, str(self.get_parameter('service_name').value), self._plan
        )
        self._prepare_service = self.create_service(SetBool,
            str(self.get_parameter('anygrasp_prepare_service').value), self._prepare_window,
            callback_group=ReentrantCallbackGroup())
        self._task_status_subscription = self.create_subscription(GoalStatusArray,
            str(self.get_parameter('anygrasp_task_status_topic').value), self._task_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=ReentrantCallbackGroup())

    def _prepare_window(self, request, response):
        hold = getattr(self._predictor, 'hold_for', None)
        prepare = getattr(self._predictor, 'prepare', None)
        if hold is None or prepare is None:
            geometric = str(self.get_parameter('predictor_type').value).strip().lower() == 'geometric'
            response.success = geometric
            response.message = 'No model preparation required' if geometric else 'Persistent AnyGrasp worker is disabled'
            return response
        try:
            hold('vlm', float(self.get_parameter('anygrasp_prepare_hold_sec').value) if request.data else 0.)
            ready = prepare() if request.data else True
            response.success = ready is not False
            response.message = 'AnyGrasp ready' if request.data else 'AnyGrasp preparation window closed'
            if not response.success:
                hold('vlm', 0.)
                response.message = 'GPU has foreground work; preparation deferred'
        except (ModelUnavailableError, ValueError, OSError) as error:
            hold('vlm', 0.)
            response.success, response.message = False, str(error)
        return response

    def _task_status(self, message):
        hold = getattr(self._predictor, 'hold_for', None)
        if hold is not None:
            active = any(status.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                                          GoalStatus.STATUS_CANCELING) for status in message.status_list)
            if active or getattr(self, '_task_hold_active', False):
                hold('task', float(self.get_parameter('anygrasp_task_hold_sec').value) if active else 0.)
            self._task_hold_active = active

    def destroy_node(self):
        close = getattr(self._predictor, 'close', None)
        if close is not None:
            close()
        return super().destroy_node()

    def _plan(self, request, response):
        response.candidates = []
        try:
            self._validate(request)
            # Capture-time TF must survive inference and its cache eviction.
            frame_rotation, frame_translation = self._planning_transform(
                request.context_cloud.header
            )
            if request.target_object_header == request.context_cloud.header:
                object_rotation, object_translation = frame_rotation, frame_translation
            else:
                object_rotation, object_translation = self._planning_transform(
                    request.target_object_header
                )
            target_object = _transform_target_object(
                request.target_object, object_rotation, object_translation
            )
            target = point_cloud_from_message(request.target_cloud)
            context = point_cloud_from_message(request.context_cloud)
            config = GraspConfig(
                nms_rotation_threshold_rad=math.radians(float(
                    self.get_parameter('nms_rotation_threshold_degrees').value
                )),
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
            candidates, ranked, retried = _predict_and_rank(
                self._predictor, target, context, workspace, config)
            if retried:
                self.get_logger().warning('Adaptive sampling found no grasp; retried the full point cloud')
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
            if self._geometry_publisher is not None:
                header = deepcopy(request.context_cloud.header)
                header.frame_id = str(self.get_parameter('planning_frame').value)
                geometry = observed_geometry_message(target.points @ frame_rotation.T+frame_translation,
                    target_object, header, request.snapshot_id, request.object_id)
                self._geometry_publisher.publish(geometry)
            for grasp in ranked:
                rotation = frame_rotation @ grasp.rotation
                translation = (
                    frame_rotation @ grasp.translation + frame_translation
                )
                approach = frame_rotation @ grasp.approach_direction
                candidate = GraspCandidate()
                candidate.header.stamp = request.context_cloud.header.stamp
                candidate.header.frame_id = str(self.get_parameter('planning_frame').value)
                candidate.snapshot_id = request.snapshot_id
                candidate.object_id = request.object_id
                candidate.target_object = deepcopy(target_object)
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
            profile = str(self.get_parameter('optimization_profile').value)
            if profile not in ('reference', 'optimized'):
                raise ValueError('Unknown AnyGrasp optimization profile')
            factory = AnyGraspPredictor
            options = {}
            if bool(self.get_parameter('anygrasp_worker_enabled').value):
                from cleany_grasping.anygrasp_worker import AnyGraspWorkerPredictor
                factory = AnyGraspWorkerPredictor
                options.update(
                    gpu_lock_path=str(self.get_parameter('gpu_lock_path').value),
                    minimum_available_memory_mb=float(self.get_parameter('anygrasp_minimum_available_memory_mb').value),
                    idle_timeout_sec=float(self.get_parameter('anygrasp_worker_idle_timeout_sec').value),
                    request_timeout_sec=float(self.get_parameter('anygrasp_worker_request_timeout_sec').value))
            return factory(
                **options,
                checkpoint_path=str(self.get_parameter('checkpoint_path').value),
                license_path=str(self.get_parameter('license_path').value),
                maximum_gripper_width_m=float(
                    self.get_parameter('maximum_gripper_width_m').value
                ),
                gripper_height_m=float(
                    self.get_parameter('gripper_height_m').value
                ),
                cuda_cache_release_threshold_mb=float(self.get_parameter('cuda_cache_release_threshold_mb').value),
                cuda_gc_interval_calls=int(self.get_parameter('cuda_gc_interval_calls').value),
                approach_direction=(tuple(self.get_parameter('anygrasp_approach_direction').value)
                    if any(self.get_parameter('anygrasp_approach_direction').value) else None),
                approach_threshold_rad=math.radians(float(self.get_parameter('anygrasp_approach_threshold_degrees').value)),
                region_steering_mode=str(self.get_parameter('anygrasp_region_steering_mode').value),
                target_region_voxel_m=float(self.get_parameter('anygrasp_target_region_voxel_m').value),
                adaptive_point_policy=str(self.get_parameter('anygrasp_adaptive_point_policy').value),
                adaptive_low_maximum_points=int(self.get_parameter('anygrasp_adaptive_low_maximum_points').value),
                adaptive_minimum_object_points=int(self.get_parameter('anygrasp_adaptive_minimum_object_points').value),
                adaptive_minimum_object_extent_m=float(self.get_parameter('anygrasp_adaptive_minimum_object_extent_m').value),
                adaptive_maximum_context_ratio=float(self.get_parameter('anygrasp_adaptive_maximum_context_ratio').value),
                adaptive_sampling_enabled=profile == 'optimized',
                adaptive_maximum_points=int(self.get_parameter('optimized_adaptive_maximum_points').value),
                adaptive_minimum_region_points=int(self.get_parameter('optimized_adaptive_minimum_region_points').value),
            )
        if predictor_type == 'geometric':
            origins = list(self.get_parameter('geometric.approach_reference_positions').value)
            if origins == [0.0]:
                origins = []
            if len(origins) % 3:
                raise ValueError('Approach reference positions need flattened xyz triples')
            return GeometricGraspPredictor(
                GeometricGraspConfig(
                    defer_support_plane_collision=bool(self.get_parameter(
                        'geometric.defer_support_plane_collision').value),
                    longitudinal_offset_fractions=tuple(float(v) for v in self.get_parameter(
                        'geometric.longitudinal_offset_fractions').value) if self.get_parameter(
                            'geometric.search_longitudinal_contacts').value else (0.0,),
                    longitudinal_max_height_ratio=float(self.get_parameter(
                        'geometric.longitudinal_max_height_ratio').value),
                    longitudinal_contact_height_offset_m=float(self.get_parameter(
                        'geometric.longitudinal_contact_height_offset_m').value),
                    approach_reference_positions=tuple(tuple(float(v) for v in origins[i:i+3])
                                                       for i in range(0, len(origins), 3)),
                    include_reverse_closing_axis=bool(self.get_parameter(
                        'geometric.include_reverse_closing_axis').value),
                    prefer_upward_closing_axis=bool(self.get_parameter(
                        'geometric.prefer_upward_closing_axis').value),
                    approach_tilt_options=tuple(
                        self.get_parameter('geometric.approach_tilt_options').value
                    ) if self.get_parameter(
                        'geometric.search_approach_tilts'
                    ).value else (),
                    maximum_gripper_width_m=float(
                        self.get_parameter('maximum_gripper_width_m').value
                    ),
                    opening_margin_m=float(
                        self.get_parameter('geometric.opening_margin_m').value
                    ),
                    grasp_depth_m=float(
                        self.get_parameter('geometric.grasp_depth_m').value
                    ),
                    maximum_top_contact_depth_m=float(
                        self.get_parameter('geometric.maximum_top_contact_depth_m').value),
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
                    approach_tilt_degrees=float(
                        self.get_parameter(
                            'geometric.approach_tilt_degrees'
                        ).value
                    ),
                    approach_tilt_direction=tuple(
                        float(value)
                        for value in self.get_parameter(
                            'geometric.approach_tilt_direction'
                        ).value
                    ),
                    reject_robot_opposite_approach=bool(
                        self.get_parameter(
                            'geometric.reject_robot_opposite_approach'
                        ).value
                    ),
                    robot_reference_position=tuple(
                        float(value)
                        for value in self.get_parameter(
                            'geometric.robot_reference_position'
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
        if (
            not left.frame_id
            or left.frame_id != right.frame_id
            or left.stamp != right.stamp
            or left.stamp.sec * 1_000_000_000 + left.stamp.nanosec <= 0
        ):
            raise ValueError('Target and context clouds must share frame and positive capture timestamp')
        object_header = request.target_object_header
        if not object_header.frame_id or object_header.stamp != right.stamp:
            raise ValueError(
                'target OBB requires an explicit frame and the same capture timestamp as clouds'
            )
        size = request.target_object.obb_size
        pose = request.target_object.obb_pose
        values = (
            size.x,
            size.y,
            size.z,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if (
            not all(math.isfinite(value) for value in values)
            or min(size.x, size.y, size.z) <= 0.0
        ):
            raise ValueError('target OBB must have finite positive dimensions and pose')
        quaternion_norm = math.sqrt(
            sum(value * value for value in values[-4:])
        )
        if not math.isclose(quaternion_norm, 1.0, abs_tol=1e-4):
            raise ValueError('target OBB quaternion must be normalized')
        if left.stamp.sec == 0 and left.stamp.nanosec == 0:
            # TF interprets Time(0) as latest, never as capture at epoch zero.
            raise ValueError('cloud capture timestamp must be nonzero')
        if not 0 <= left.stamp.nanosec < 1_000_000_000:
            raise ValueError('cloud capture timestamp has invalid nanoseconds')

    def _planning_transform(self, header: Header) -> tuple[np.ndarray, np.ndarray]:
        target_frame = str(self.get_parameter('planning_frame').value)
        source_frame = header.frame_id
        if target_frame == source_frame:
            return np.eye(3), np.zeros(3)
        transform = self._tf_buffer.lookup_transform(
            target_frame, source_frame, Time.from_msg(header.stamp), timeout=Duration(seconds=0.5)
        ).transform
        q = transform.rotation
        frame_rotation = _rotation_from_quaternion(q.x, q.y, q.z, q.w)
        frame_translation = np.array((transform.translation.x, transform.translation.y, transform.translation.z))
        return frame_rotation, frame_translation


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
