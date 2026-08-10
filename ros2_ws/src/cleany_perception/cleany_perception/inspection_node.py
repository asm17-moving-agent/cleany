from __future__ import annotations

import threading

import rclpy
from cleany_interfaces.action import InspectScene
from cleany_interfaces.msg import DetectedObject3D, DetectedObject3DArray
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
from cleany_perception.core.models import (
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


_DEBUG_IMAGE_QOS = QoSProfile(
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

        self._snapshot_buffer = RgbdSnapshotBuffer()
        self._sensor_callback_group = MutuallyExclusiveCallbackGroup()
        self._action_callback_group = ReentrantCallbackGroup()
        self._create_sensor_subscriptions()
        self._objects_publisher = self.create_publisher(
            DetectedObject3DArray,
            str(self.get_parameter('objects_topic').value),
            10,
        )
        self._debug_publisher = self.create_publisher(
            Image,
            str(self.get_parameter('debug_image_topic').value),
            _DEBUG_IMAGE_QOS,
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
        self.declare_parameter('debug_image_topic', 'perception/debug_image')
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
        result.objects = self._empty_objects()
        try:
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
            output = self._pipeline.inspect(
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
                capture_transform=capture_transform,
            )
            objects_message = self._objects_message(
                output,
                snapshot.stamp_ns,
                messages.sequence,
            )
            result.success = True
            result.error_code = InspectScene.Result.ERROR_NONE
            result.message = f'Inspected {len(output.objects)} objects'
            result.objects = objects_message
            self._objects_publisher.publish(objects_message)
            debug_rgb = render_debug_image(
                snapshot.rgb,
                output.detections,
                output.masks,
            )
            self._debug_publisher.publish(
                debug_image_message(
                    debug_rgb,
                    snapshot.stamp_ns,
                    messages.color.header.frame_id,
                )
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
    def _empty_objects() -> DetectedObject3DArray:
        return DetectedObject3DArray()

    @staticmethod
    def _objects_message(
        output: InspectionOutput,
        stamp_ns: int,
        sequence: int,
    ) -> DetectedObject3DArray:
        message = DetectedObject3DArray()
        message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        message.header.frame_id = output.target_frame
        message.snapshot_id = f'rgbd-{stamp_ns:019d}-{sequence:06d}'
        for object_id, inspected in enumerate(output.objects, start=1):
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
