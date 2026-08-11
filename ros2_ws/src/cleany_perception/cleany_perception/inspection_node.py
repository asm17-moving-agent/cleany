from __future__ import annotations

import threading
from collections.abc import Sequence

import numpy as np

import rclpy
from cleany_interfaces.action import InspectScene
from cleany_interfaces.msg import (
    DetectedObject2D,
    DetectedObject2DArray,
    DetectedObject3D,
    DetectedObject3DArray,
)
from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from cleany_perception.adapters.gemini_detector import GeminiDetector
from cleany_perception.adapters.sam2_segmenter import Sam2Segmenter
from cleany_perception.adapters.tf2_transform import Tf2TransformAdapter
from cleany_perception.core.geometry import quaternion_xyzw_from_rotation
from cleany_perception.core.point_cloud import colored_cloud_from_selection
from cleany_perception.core.models import (
    Detection2D,
    FailureKind,
    InspectionFailure,
    InspectionOutput,
    InspectionStage,
    PipelineConfig,
    RgbdSnapshot,
    RigidTransform,
)
from cleany_perception.core.pipeline import InspectionPipeline
from cleany_perception.core.ports import (
    DetectorPort,
    SegmenterPort,
    TransformPort,
)
from cleany_perception.debug_image import (
    debug_image_message,
    render_debug_image,
)
from cleany_perception.rgbd_snapshot import (
    RgbdSnapshotBuffer,
    snapshot_from_messages,
)
from cleany_perception.point_cloud_message import colored_point_cloud_message
from cleany_perception.snapshot_cache import (
    CachedDetectionSnapshot,
    DetectionSnapshotCache,
)


_FAILURE_CODES = {
    FailureKind.RGBD_TIMEOUT: InspectScene.Result.ERROR_RGBD_TIMEOUT,
    FailureKind.DETECTOR_API: InspectScene.Result.ERROR_DETECTOR_API,
    FailureKind.DETECTOR_RESPONSE: InspectScene.Result.ERROR_DETECTOR_RESPONSE,
    FailureKind.MASK: InspectScene.Result.ERROR_MASK,
    FailureKind.DEPTH: InspectScene.Result.ERROR_DEPTH,
    FailureKind.PLANE: InspectScene.Result.ERROR_PLANE,
    FailureKind.TF: InspectScene.Result.ERROR_TF,
    FailureKind.CANCELLED: InspectScene.Result.ERROR_CANCELLED,
    FailureKind.INTERNAL: InspectScene.Result.ERROR_INTERNAL,
}


_STAGE_CODES = {
    InspectionStage.WAITING_FOR_RGBD: (
        InspectScene.Feedback.STAGE_WAITING_FOR_RGBD
    ),
    InspectionStage.DETECTING: InspectScene.Feedback.STAGE_DETECTING,
    InspectionStage.SEGMENTING: InspectScene.Feedback.STAGE_SEGMENTING,
    InspectionStage.RECONSTRUCTING: InspectScene.Feedback.STAGE_RECONSTRUCTING,
    InspectionStage.TRANSFORMING: InspectScene.Feedback.STAGE_TRANSFORMING,
}


_LATCHED_DEBUG_IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class InspectionNode(Node):
    def __init__(
        self,
        detector: DetectorPort | None = None,
        segmenter: SegmenterPort | None = None,
        transformer: TransformPort | None = None,
        **kwargs,
    ) -> None:
        super().__init__('perception_inspector', **kwargs)
        self._declare_parameters()
        self._default_query = str(self.get_parameter('default_query').value)
        self._snapshot_timeout_seconds = float(
            self.get_parameter('snapshot_timeout_seconds').value
        )
        self._depth_16u_scale_m = float(
            self.get_parameter('depth_16u_scale_m').value
        )
        self._debug_republish_count = int(
            self.get_parameter('debug_republish_count').value
        )
        self._debug_republish_period_seconds = float(
            self.get_parameter('debug_republish_period_seconds').value
        )
        if self._debug_republish_count < 1:
            raise ValueError('Debug republish count must be at least one')
        if self._debug_republish_period_seconds <= 0.0:
            raise ValueError('Debug republish period must be positive')
        target_frame = str(self.get_parameter('target_frame').value)
        self._target_frame = target_frame

        if detector is None:
            detector = GeminiDetector(
                model=str(self.get_parameter('gemini_model').value),
                api_key_environment=str(
                    self.get_parameter('gemini_api_key_environment').value
                ),
                timeout_seconds=float(
                    self.get_parameter('detector_timeout_seconds').value
                ),
            )
        if segmenter is None:
            segmenter = Sam2Segmenter(
                model_config=str(
                    self.get_parameter('sam2_model_config').value
                ),
                checkpoint_path=str(
                    self.get_parameter('sam2_checkpoint').value
                ),
                device=str(self.get_parameter('sam2_device').value),
            )
        if transformer is None:
            transformer = Tf2TransformAdapter(
                self,
                timeout_seconds=float(
                    self.get_parameter('tf_timeout_seconds').value
                ),
                cache_seconds=float(
                    self.get_parameter('tf_cache_seconds').value
                ),
            )
        self._transformer = transformer
        self._pipeline = InspectionPipeline(
            detector=detector,
            segmenter=segmenter,
            transformer=transformer,
            target_frame=target_frame,
            config=self._pipeline_config(),
        )
        self._snapshot_cache = DetectionSnapshotCache(
            maximum_entries=int(
                self.get_parameter('snapshot_cache_max_entries').value
            ),
            ttl_seconds=float(
                self.get_parameter('snapshot_cache_ttl_seconds').value
            ),
        )

        self._snapshot_buffer = RgbdSnapshotBuffer()
        self._sensor_callback_group = MutuallyExclusiveCallbackGroup()
        self._action_callback_group = ReentrantCallbackGroup()
        self._create_sensor_subscriptions()
        self._objects_publisher = self.create_publisher(
            DetectedObject3DArray,
            str(self.get_parameter('objects_topic').value),
            10,
        )
        self._detections_publisher = self.create_publisher(
            DetectedObject2DArray,
            str(self.get_parameter('detections_topic').value),
            10,
        )
        self._debug_publisher = self.create_publisher(
            Image,
            str(self.get_parameter('debug_image_topic').value),
            qos_profile_sensor_data,
        )
        self._latched_debug_publisher = self.create_publisher(
            Image,
            str(self.get_parameter('latched_debug_image_topic').value),
            _LATCHED_DEBUG_IMAGE_QOS,
        )
        self._debug_lock = threading.Lock()
        self._debug_message = None
        self._debug_republishes_remaining = 0
        self._debug_timer = self.create_timer(
            self._debug_republish_period_seconds,
            self._republish_debug_image,
        )
        self._busy_lock = threading.Lock()
        self._busy = False
        self._action_server = ActionServer(
            self,
            InspectScene,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute,
            callback_group=self._action_callback_group,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

    def destroy_node(self) -> None:
        self._action_server.destroy()
        super().destroy_node()

    def _declare_parameters(self) -> None:
        self.declare_parameter('action_name', 'perception/inspect_scene')
        self.declare_parameter('objects_topic', 'perception/objects')
        self.declare_parameter('detections_topic', 'perception/detections_2d')
        self.declare_parameter('debug_image_topic', 'perception/debug_image')
        self.declare_parameter(
            'latched_debug_image_topic',
            'perception/debug_image_latched',
        )
        self.declare_parameter('debug_republish_count', 5)
        self.declare_parameter('debug_republish_period_seconds', 0.25)
        self.declare_parameter('color_image_topic', 'camera/color/image_raw')
        self.declare_parameter('color_info_topic', 'camera/color/camera_info')
        self.declare_parameter('depth_image_topic', 'camera/depth/image_raw')
        self.declare_parameter('depth_info_topic', 'camera/depth/camera_info')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter(
            'default_query',
            'Detect the box and can on the table.',
        )
        self.declare_parameter('snapshot_timeout_seconds', 2.0)
        self.declare_parameter('depth_16u_scale_m', 0.001)
        self.declare_parameter('snapshot_cache_max_entries', 2)
        self.declare_parameter('snapshot_cache_ttl_seconds', 120.0)
        self.declare_parameter(
            'gemini_model',
            'gemini-robotics-er-2-preview',
        )
        self.declare_parameter('gemini_api_key_environment', 'GEMINI_API_KEY')
        self.declare_parameter('detector_timeout_seconds', 30.0)
        self.declare_parameter('sam2_model_config', '')
        self.declare_parameter('sam2_checkpoint', '')
        self.declare_parameter('sam2_device', 'cuda')
        self.declare_parameter('tf_timeout_seconds', 0.5)
        self.declare_parameter('tf_cache_seconds', 60.0)
        self.declare_parameter('minimum_detection_confidence', 0.25)
        self.declare_parameter('maximum_detections', 10)
        self.declare_parameter('minimum_depth_m', 0.1)
        self.declare_parameter('maximum_depth_m', 3.0)
        self.declare_parameter('support_margin_pixels', 40)
        self.declare_parameter('support_sample_stride', 4)
        self.declare_parameter('plane_ransac_iterations', 200)
        self.declare_parameter('plane_distance_threshold_m', 0.006)
        self.declare_parameter('plane_minimum_inliers', 100)
        self.declare_parameter('plane_minimum_inlier_ratio', 0.35)
        self.declare_parameter('maximum_plane_tilt_degrees', 20.0)
        self.declare_parameter('minimum_object_points', 30)
        self.declare_parameter('minimum_object_height_m', 0.005)
        self.declare_parameter('minimum_obb_extent_m', 0.005)
        self.declare_parameter('grasp_context_margin_pixels', 80)
        self.declare_parameter('grasp_cloud_voxel_size_m', 0.005)
        self.declare_parameter('grasp_target_maximum_points', 12000)
        self.declare_parameter('grasp_context_maximum_points', 30000)

    def _pipeline_config(self) -> PipelineConfig:
        return PipelineConfig(
            minimum_detection_confidence=float(
                self.get_parameter('minimum_detection_confidence').value
            ),
            maximum_detections=int(
                self.get_parameter('maximum_detections').value
            ),
            minimum_depth_m=float(self.get_parameter('minimum_depth_m').value),
            maximum_depth_m=float(self.get_parameter('maximum_depth_m').value),
            support_margin_pixels=int(
                self.get_parameter('support_margin_pixels').value
            ),
            support_sample_stride=int(
                self.get_parameter('support_sample_stride').value
            ),
            plane_ransac_iterations=int(
                self.get_parameter('plane_ransac_iterations').value
            ),
            plane_distance_threshold_m=float(
                self.get_parameter('plane_distance_threshold_m').value
            ),
            plane_minimum_inliers=int(
                self.get_parameter('plane_minimum_inliers').value
            ),
            plane_minimum_inlier_ratio=float(
                self.get_parameter('plane_minimum_inlier_ratio').value
            ),
            maximum_plane_tilt_degrees=float(
                self.get_parameter('maximum_plane_tilt_degrees').value
            ),
            minimum_object_points=int(
                self.get_parameter('minimum_object_points').value
            ),
            minimum_object_height_m=float(
                self.get_parameter('minimum_object_height_m').value
            ),
            minimum_obb_extent_m=float(
                self.get_parameter('minimum_obb_extent_m').value
            ),
        )

    def _create_sensor_subscriptions(self) -> None:
        arguments = {
            'qos_profile': qos_profile_sensor_data,
            'callback_group': self._sensor_callback_group,
        }
        self.create_subscription(
            Image,
            str(self.get_parameter('color_image_topic').value),
            self._snapshot_buffer.add_color,
            **arguments,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('color_info_topic').value),
            self._snapshot_buffer.add_color_info,
            **arguments,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('depth_image_topic').value),
            self._snapshot_buffer.add_depth,
            **arguments,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('depth_info_topic').value),
            self._snapshot_buffer.add_depth_info,
            **arguments,
        )

    def _goal_callback(self, _goal_request) -> GoalResponse:
        with self._busy_lock:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle) -> InspectScene.Result:
        result = InspectScene.Result()
        result.detections = self._empty_detections()
        result.objects = self._empty_objects()
        try:
            has_snapshot = bool(goal_handle.request.snapshot_id)
            has_selection = goal_handle.request.selected_object_id != 0
            if has_snapshot != has_selection:
                result.success = False
                result.error_code = InspectScene.Result.ERROR_INVALID_SELECTION
                result.message = (
                    'snapshot_id and selected_object_id must be supplied '
                    'together'
                )
                goal_handle.abort()
                return result
            if has_snapshot:
                return self._execute_selection(goal_handle, result)
            self._feedback(
                goal_handle,
                InspectionStage.WAITING_FOR_RGBD,
                0,
                0,
                'Waiting for a new synchronized RGB-D snapshot',
            )
            baseline = self._snapshot_buffer.sequence
            messages = self._snapshot_buffer.wait_for_new(
                baseline,
                self._snapshot_timeout_seconds,
                cancelled=lambda: goal_handle.is_cancel_requested,
            )
            snapshot = snapshot_from_messages(
                messages,
                depth_16u_scale_m=self._depth_16u_scale_m,
            )
            capture_transform = self._lookup_capture_transform(snapshot)
            query = goal_handle.request.query.strip() or self._default_query
            detections = self._pipeline.detect(
                snapshot,
                query,
                progress=lambda stage, detections, objects, message: (
                    self._feedback(
                        goal_handle,
                        stage,
                        detections,
                        objects,
                        message,
                    )
                ),
                cancelled=lambda: goal_handle.is_cancel_requested,
            )
            self._feedback(
                goal_handle,
                InspectionStage.DETECTING,
                len(detections),
                0,
                f'Detected {len(detections)} objects',
            )
            snapshot_id = self._snapshot_id(
                snapshot.stamp_ns,
                messages.sequence,
            )
            self._snapshot_cache.put(
                snapshot_id,
                CachedDetectionSnapshot(
                    snapshot=snapshot,
                    detections=detections,
                    capture_transform=capture_transform,
                    color_frame=messages.color.header.frame_id,
                ),
            )
            detections_message = self._detections_message(
                detections,
                snapshot.stamp_ns,
                messages.color.header.frame_id,
                snapshot_id,
            )
            result.success = True
            result.error_code = InspectScene.Result.ERROR_NONE
            result.message = (
                f'Detected {len(detections)} objects; select an object_id '
                'for selected-object inspection'
            )
            result.detections = detections_message
            self._detections_publisher.publish(detections_message)
            debug_rgb = render_debug_image(
                snapshot.rgb,
                detections,
                (),
            )
            self._publish_debug_image(
                debug_image_message(
                    debug_rgb,
                    snapshot.stamp_ns,
                    messages.color.header.frame_id,
                ),
            )
            goal_handle.succeed()
            return result
        except InspectionFailure as error:
            result.success = False
            result.error_code = _FAILURE_CODES[error.kind]
            result.message = str(error)
            if error.kind == FailureKind.CANCELLED:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        except Exception as error:
            result.success = False
            result.error_code = InspectScene.Result.ERROR_INTERNAL
            result.message = f'Unexpected inspection error: {error}'
            goal_handle.abort()
            return result
        finally:
            with self._busy_lock:
                self._busy = False

    def _execute_selection(
        self,
        goal_handle,
        result: InspectScene.Result,
    ) -> InspectScene.Result:
        snapshot_id = goal_handle.request.snapshot_id
        cached = self._snapshot_cache.get(snapshot_id)
        if cached is None:
            result.success = False
            result.error_code = InspectScene.Result.ERROR_SNAPSHOT_NOT_FOUND
            result.message = f'Snapshot not found or expired: {snapshot_id}'
            goal_handle.abort()
            return result
        selected_id = int(goal_handle.request.selected_object_id)
        if selected_id < 1 or selected_id > len(cached.detections):
            result.success = False
            result.error_code = InspectScene.Result.ERROR_INVALID_SELECTION
            result.message = (
                f'Object ID {selected_id} is outside the snapshot range '
                f'1..{len(cached.detections)}'
            )
            goal_handle.abort()
            return result
        selected = cached.detections[selected_id - 1]
        output = self._pipeline.inspect_selected(
            cached.snapshot,
            cached.detections,
            selected,
            cached.capture_transform,
            progress=lambda stage, detections, objects, message: (
                self._feedback(
                    goal_handle,
                    stage,
                    detections,
                    objects,
                    message,
                )
            ),
            cancelled=lambda: goal_handle.is_cancel_requested,
        )
        detections_message = self._detections_message(
            cached.detections,
            cached.snapshot.stamp_ns,
            cached.color_frame,
            snapshot_id,
        )
        objects_message = self._objects_message(
            output,
            cached.snapshot.stamp_ns,
            snapshot_id,
            (selected_id,),
        )
        target_cloud, context_cloud = self._selected_cloud_messages(
            cached.snapshot,
            output.masks[0].mask,
            selected,
        )
        result.success = True
        result.error_code = InspectScene.Result.ERROR_NONE
        result.message = (
            f'Inspected selected object {selected_id}: {selected.label}'
        )
        result.detections = detections_message
        result.objects = objects_message
        result.target_cloud = target_cloud
        result.context_cloud = context_cloud
        self._objects_publisher.publish(objects_message)
        debug_rgb = render_debug_image(
            cached.snapshot.rgb,
            output.detections,
            output.masks,
            object_ids=(selected_id,),
        )
        self._publish_debug_image(
            debug_image_message(
                debug_rgb,
                cached.snapshot.stamp_ns,
                cached.color_frame,
            )
        )
        goal_handle.succeed()
        return result

    def _selected_cloud_messages(self, snapshot, target_mask, detection):
        height, width = snapshot.depth_m.shape
        margin = int(self.get_parameter('grasp_context_margin_pixels').value)
        x_min = max(0, int(detection.bbox.x_min) - margin)
        y_min = max(0, int(detection.bbox.y_min) - margin)
        x_max = min(width, int(detection.bbox.x_max + 0.999) + margin)
        y_max = min(height, int(detection.bbox.y_max + 0.999) + margin)
        context_mask = np.zeros((height, width), dtype=bool)
        context_mask[y_min:y_max, x_min:x_max] = True
        common = {
            'depth_m': snapshot.depth_m,
            'rgb': snapshot.rgb,
            'intrinsics': snapshot.intrinsics,
            'minimum_depth_m': float(self.get_parameter('minimum_depth_m').value),
            'maximum_depth_m': float(self.get_parameter('maximum_depth_m').value),
            'voxel_size_m': float(self.get_parameter('grasp_cloud_voxel_size_m').value),
        }
        target = colored_cloud_from_selection(
            selection=target_mask,
            maximum_points=int(self.get_parameter('grasp_target_maximum_points').value),
            **common,
        )
        context = colored_cloud_from_selection(
            selection=context_mask,
            maximum_points=int(self.get_parameter('grasp_context_maximum_points').value),
            **common,
        )
        stamp = Time(nanoseconds=snapshot.stamp_ns).to_msg()
        return (
            colored_point_cloud_message(target, stamp, snapshot.source_frame),
            colored_point_cloud_message(context, stamp, snapshot.source_frame),
        )

    def _publish_debug_image(self, message: Image) -> None:
        self._debug_publisher.publish(message)
        self._latched_debug_publisher.publish(message)
        with self._debug_lock:
            self._debug_message = message
            self._debug_republishes_remaining = (
                self._debug_republish_count - 1
            )

    def _republish_debug_image(self) -> None:
        with self._debug_lock:
            if (
                self._debug_message is None
                or self._debug_republishes_remaining <= 0
            ):
                return
            message = self._debug_message
            self._debug_republishes_remaining -= 1
        self._debug_publisher.publish(message)

    def _lookup_capture_transform(
        self,
        snapshot: RgbdSnapshot,
    ) -> RigidTransform:
        try:
            return self._transformer.lookup(
                self._target_frame,
                snapshot.source_frame,
                snapshot.stamp_ns,
            )
        except InspectionFailure:
            raise
        except Exception as error:
            raise InspectionFailure(
                FailureKind.TF,
                f'Failed to look up capture transform: {error}',
            ) from error

    @staticmethod
    def _feedback(
        goal_handle,
        stage: InspectionStage,
        detections: int,
        objects: int,
        message: str,
    ) -> None:
        feedback = InspectScene.Feedback()
        feedback.stage = _STAGE_CODES[stage]
        feedback.detections_2d = detections
        feedback.objects_3d = objects
        feedback.message = message
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _empty_detections() -> DetectedObject2DArray:
        return DetectedObject2DArray()

    @staticmethod
    def _empty_objects() -> DetectedObject3DArray:
        return DetectedObject3DArray()

    @staticmethod
    def _snapshot_id(stamp_ns: int, sequence: int) -> str:
        return f'rgbd-{stamp_ns:019d}-{sequence:06d}'

    @staticmethod
    def _detections_message(
        detections: Sequence[Detection2D],
        stamp_ns: int,
        frame_id: str,
        snapshot_id: str,
    ) -> DetectedObject2DArray:
        message = DetectedObject2DArray()
        message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        message.header.frame_id = frame_id
        message.snapshot_id = snapshot_id
        for object_id, detection in enumerate(detections, start=1):
            detected = DetectedObject2D()
            detected.object_id = object_id
            detected.label = detection.label
            detected.confidence = detection.confidence
            detected.x_min = detection.bbox.x_min
            detected.y_min = detection.bbox.y_min
            detected.x_max = detection.bbox.x_max
            detected.y_max = detection.bbox.y_max
            message.detections.append(detected)
        return message

    @staticmethod
    def _objects_message(
        output: InspectionOutput,
        stamp_ns: int,
        snapshot_id: str,
        object_ids: Sequence[int],
    ) -> DetectedObject3DArray:
        if len(object_ids) != len(output.objects):
            raise ValueError('Object IDs must match inspected objects')
        message = DetectedObject3DArray()
        message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        message.header.frame_id = output.target_frame
        message.snapshot_id = snapshot_id
        for object_id, inspected in zip(object_ids, output.objects):
            detected = DetectedObject3D()
            detected.object_id = object_id
            detected.label = inspected.label
            detected.confidence = inspected.confidence
            detected.obb_pose.position.x = float(inspected.box.center[0])
            detected.obb_pose.position.y = float(inspected.box.center[1])
            detected.obb_pose.position.z = float(inspected.box.center[2])
            quaternion = quaternion_xyzw_from_rotation(inspected.box.rotation)
            detected.obb_pose.orientation.x = quaternion[0]
            detected.obb_pose.orientation.y = quaternion[1]
            detected.obb_pose.orientation.z = quaternion[2]
            detected.obb_pose.orientation.w = quaternion[3]
            detected.obb_size.x = float(inspected.box.size[0])
            detected.obb_size.y = float(inspected.box.size[1])
            detected.obb_size.z = float(inspected.box.size[2])
            message.objects.append(detected)
        return message


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = InspectionNode()
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


if __name__ == '__main__':
    main()
