from __future__ import annotations

import threading
import os
from pathlib import Path
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
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from cleany_perception.adapters.gemini_detector import GeminiClassifier, GeminiDetector
from cleany_perception.adapters.simulation_color import (
    SimulationColorDetector,
    SimulationColorSegmenter,
)
from cleany_perception.adapters.tf2_transform import Tf2TransformAdapter
from cleany_perception.adapters.yoloe_detector import YoloeDetector, YoloeMaskSegmenter
from cleany_perception.adapters.yoloe_gemini import YoloeGeminiDetector
from cleany_perception.model_runtime import (
    resolve_device, resolve_model_assets,
)
from cleany_perception.core.geometry import quaternion_xyzw_from_rotation
from cleany_perception.core.point_cloud import (
    colored_cloud_from_selection,
    transform_colored_cloud,
)
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
from cleany_perception.core.object_ranking import (
    ObjectRankingConfig,
    rank_detections_by_distance,
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
from cleany_perception.core.tracking import ObjectTracker, TrackingConfig, TrackingOutput
from cleany_interfaces.msg import TrackedObjectReference

from cleany_perception.snapshot_cache import (
    CachedDetectionSnapshot,
    DetectionSnapshotCache,
)
from cleany_interfaces.srv import GetSceneSnapshot


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
        if self._debug_republish_count < 0:
            raise ValueError('Debug republish count must be nonnegative')
        if self._debug_republish_period_seconds <= 0.0:
            raise ValueError('Debug republish period must be positive')
        target_frame = str(self.get_parameter('target_frame').value)
        self._target_frame = target_frame

        detector_type = str(self.get_parameter('detector_type').value)
        segmenter_type = str(self.get_parameter('segmenter_type').value)
        if detector_type == 'yoloe_gemini' and segmenter_type != 'yoloe_seg':
            raise ValueError('YOLOE+Gemini classification requires YOLOE segmentation')
        for kind, supplied in ((detector_type, detector),
                               (segmenter_type, segmenter)):
            if supplied is None and kind in ('yoloe', 'yoloe_gemini'):
                self._resolve_local_model('yoloe' if kind == 'yoloe_gemini' else kind)
        wrist_yoloe = None
        wrist_check_yoloe = None
        wrist_mouse_check_yoloe = None
        wrist_right_yoloe = None
        wrist_right_check_yoloe = None
        if detector is None:
            if detector_type == 'gemini':
                detector = GeminiDetector(
                    model=str(self.get_parameter('gemini_model').value),
                    api_key_environment=str(
                        self.get_parameter('gemini_api_key_environment').value
                    ),
                    timeout_seconds=float(
                        self.get_parameter('detector_timeout_seconds').value
                    ),
                )
            elif detector_type == 'simulation_color':
                detector = SimulationColorDetector(
                    minimum_pixels=int(
                        self.get_parameter(
                            'simulation_color_minimum_pixels'
                        ).value
                    ),
                    profile=str(
                        self.get_parameter(
                            'simulation_color_profile'
                        ).value
                    ),
                )
            elif detector_type in ('yoloe', 'yoloe_gemini'):
                def make_yoloe(
                    model_path: str, *, class_thresholds: tuple[float, ...] | None = None,
                ) -> YoloeDetector:
                    return YoloeDetector(
                        model_path=model_path,
                        classes=tuple(self.get_parameter('yoloe_classes').value),
                        device=str(self.get_parameter('yoloe_device').value),
                        image_size=int(self.get_parameter('yoloe_image_size').value),
                        confidence_threshold=float(self.get_parameter(
                            'minimum_detection_confidence').value),
                        iou_threshold=float(self.get_parameter('yoloe_iou_threshold').value),
                        maximum_detections=int(self.get_parameter('maximum_detections').value),
                        text_encoder_directory=str(self.get_parameter(
                            'yoloe_text_encoder_directory').value),
                        require_masks=segmenter_type == 'yoloe_seg',
                        class_confidence_thresholds=(
                            class_thresholds if class_thresholds is not None else
                            tuple(self.get_parameter(
                                'yoloe_class_confidence_thresholds').value or ())),
                    )

                yoloe = make_yoloe(str(self.get_parameter('yoloe_model_path').value))
                wrist_yoloe = yoloe
                if (segmenter_type == 'yoloe_seg'
                        and bool(self.get_parameter('enable_wrist_observation').value)):
                    def resolved_model_path(parameter_name: str) -> Path | None:
                        raw = str(self.get_parameter(parameter_name).value).strip()
                        if not raw:
                            return None
                        path = Path(raw).expanduser()
                        if not path.is_absolute():
                            path = Path(str(self.get_parameter(
                                'model_directory').value)).expanduser() / path
                        return path.resolve()

                    head_path = Path(str(self.get_parameter('yoloe_model_path').value)).resolve()
                    wrist_path = resolved_model_path('wrist_yoloe_model_path')
                    if wrist_path is not None and wrist_path != head_path:
                        wrist_yoloe = make_yoloe(str(wrist_path))
                    wrist_right_yoloe = wrist_yoloe
                    right_path = resolved_model_path('wrist_right_yoloe_model_path')
                    if right_path == head_path:
                        wrist_right_yoloe = yoloe
                    elif right_path == wrist_path:
                        wrist_right_yoloe = wrist_yoloe
                    elif right_path is not None:
                        wrist_right_yoloe = make_yoloe(str(right_path))
                    wrist_check_yoloe = wrist_yoloe
                    check_path = resolved_model_path('wrist_check_yoloe_model_path')
                    if check_path == head_path:
                        wrist_check_yoloe = yoloe
                    elif check_path == wrist_path:
                        wrist_check_yoloe = wrist_yoloe
                    elif check_path == right_path:
                        wrist_check_yoloe = wrist_right_yoloe
                    elif check_path is not None:
                        wrist_check_yoloe = make_yoloe(str(check_path))
                    wrist_mouse_check_yoloe = wrist_check_yoloe
                    mouse_check_path = resolved_model_path(
                        'wrist_mouse_check_yoloe_model_path')
                    if mouse_check_path is not None:
                        for path, adapter in (
                            (head_path, yoloe), (wrist_path, wrist_yoloe),
                            (right_path, wrist_right_yoloe),
                            (check_path, wrist_check_yoloe),
                        ):
                            if mouse_check_path == path:
                                wrist_mouse_check_yoloe = adapter
                                break
                        else:
                            wrist_mouse_check_yoloe = make_yoloe(
                                str(mouse_check_path))
                    wrist_right_check_yoloe = wrist_check_yoloe
                    right_check_path = resolved_model_path(
                        'wrist_right_check_yoloe_model_path')
                    right_check_thresholds = tuple(self.get_parameter(
                        'wrist_right_check_yoloe_class_confidence_thresholds'
                    ).value or ())
                    if right_check_path is not None and right_check_thresholds:
                        wrist_right_check_yoloe = make_yoloe(
                            str(right_check_path),
                            class_thresholds=right_check_thresholds)
                    else:
                        for path, adapter in (
                            (head_path, yoloe), (wrist_path, wrist_yoloe),
                            (right_path, wrist_right_yoloe),
                            (check_path, wrist_check_yoloe),
                        ):
                            if right_check_path is not None and right_check_path == path:
                                wrist_right_check_yoloe = adapter
                                break
                        else:
                            if right_check_path is not None:
                                wrist_right_check_yoloe = make_yoloe(
                                    str(right_check_path))
                if detector_type == 'yoloe_gemini':
                    detector = YoloeGeminiDetector(yoloe, GeminiClassifier(
                        model=str(self.get_parameter('gemini_model').value),
                        api_key_environment=str(self.get_parameter(
                            'gemini_api_key_environment').value),
                        timeout_seconds=float(self.get_parameter(
                            'detector_timeout_seconds').value),
                    ))
                else:
                    detector = yoloe
            else:
                raise ValueError(f'Unsupported detector_type: {detector_type}')
        if segmenter is None:
            if segmenter_type == 'simulation_color':
                segmenter = SimulationColorSegmenter(
                    profile=str(
                        self.get_parameter(
                            'simulation_color_profile'
                        ).value
                    )
                )
            elif segmenter_type == 'yoloe_seg':
                if detector_type not in ('yoloe', 'yoloe_gemini'):
                    raise ValueError('YOLOE masks require a YOLOE detector')
                segmenter = YoloeMaskSegmenter()
            else:
                raise ValueError(
                    f'Unsupported segmenter_type: {segmenter_type}'
                )
        if bool(self.get_parameter('preload_models').value):
            self.get_logger().info('Loading perception models')
            adapters = (('detector', detector), ('segmenter', segmenter))
            if wrist_yoloe is not None and wrist_yoloe is not yoloe:
                adapters += (('wrist detector', wrist_yoloe),)
            if (wrist_right_yoloe is not None
                    and wrist_right_yoloe is not wrist_yoloe
                    and wrist_right_yoloe is not yoloe):
                adapters += (('right wrist detector', wrist_right_yoloe),)
            if (wrist_check_yoloe is not None
                    and wrist_check_yoloe is not wrist_yoloe
                    and wrist_check_yoloe is not wrist_right_yoloe
                    and wrist_check_yoloe is not yoloe):
                adapters += (('wrist check detector', wrist_check_yoloe),)
            if (wrist_right_check_yoloe is not None
                    and wrist_right_check_yoloe not in (
                        yoloe, wrist_yoloe, wrist_right_yoloe, wrist_check_yoloe)):
                adapters += (('right wrist check detector', wrist_right_check_yoloe),)
            if (wrist_mouse_check_yoloe is not None
                    and wrist_mouse_check_yoloe not in (
                        yoloe, wrist_yoloe, wrist_right_yoloe,
                        wrist_check_yoloe, wrist_right_check_yoloe)):
                adapters += (('mouse wrist check detector', wrist_mouse_check_yoloe),)
            for name, adapter in adapters:
                prepare = getattr(adapter, 'prepare', None)
                if not callable(prepare):
                    raise ValueError(f'{name} cannot preload models')
                self.get_logger().debug(f'Loading {name} before action ready')
                prepare()
            detector_device = (
                f"remote API: {self.get_parameter('gemini_model').value}; access not yet verified"
                if detector_type == 'gemini' else self.get_parameter('yoloe_device').value)
            segmenter_device = ('YOLOE instance masks' if segmenter_type == 'yoloe_seg'
                                else 'simulation color')
            self.get_logger().info(
                'PERCEPTION MODELS READY: '
                f'{detector_type} ({detector_device}) + '
                f'{segmenter_type} ({segmenter_device}); '
                'no simulation fallback'
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
        self._object_ranking_config = ObjectRankingConfig(
            central_bbox_fraction=float(
                self.get_parameter(
                    'nearest_object_central_bbox_fraction'
                ).value
            ),
            minimum_valid_depth_pixels=int(
                self.get_parameter('nearest_object_minimum_depth_pixels').value
            ),
            minimum_depth_m=float(
                self.get_parameter('minimum_depth_m').value
            ),
            maximum_depth_m=float(
                self.get_parameter('maximum_depth_m').value
            ),
        )
        self._tracker = ObjectTracker(TrackingConfig(
            maximum_distance_m=float(self.get_parameter('tracking_maximum_distance_m').value),
            ambiguity_margin_m=float(self.get_parameter('tracking_ambiguity_margin_m').value),
            session_ttl_seconds=float(self.get_parameter('tracking_session_ttl_seconds').value),
            maximum_sessions=int(self.get_parameter('tracking_maximum_sessions').value),
            label_aliases=tuple(tuple(item.split('=', 1)) for item in
                                self.get_parameter('tracking_label_aliases').value),
        ))
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
        self._debug_timer = (
            self.create_timer(
                self._debug_republish_period_seconds,
                self._republish_debug_image,
            )
            if self._debug_republish_count > 1 else None
        )
        self._busy_lock = threading.Lock()
        self._busy = False
        self._wrist_service = None
        if bool(self.get_parameter('enable_wrist_observation').value):
            if detector_type != 'yoloe_gemini' or segmenter_type != 'yoloe_seg' or wrist_yoloe is None:
                raise ValueError('Wrist observation requires the local YOLOE-seg detector')
            from cleany_perception.wrist_service import WristService
            from dataclasses import asdict
            from cleany_perception.core.wrist_observation import WristObservationConfig
            wrist_config = WristObservationConfig(**{
                name: self.declare_parameter(f'wrist_{name}', default).value
                for name, default in asdict(WristObservationConfig()).items()
            })
            self._wrist_service = WristService(
                self, wrist_yoloe,
                check_detector=wrist_check_yoloe,
                mouse_check_detector=wrist_mouse_check_yoloe,
                right_detector=wrist_right_yoloe,
                right_check_detector=wrist_right_check_yoloe,
                failure_image_directory=str(self.get_parameter(
                    'wrist_failure_image_directory').value),
                config=wrist_config)
        self._snapshot_service = self.create_service(
            GetSceneSnapshot, 'perception/get_scene_snapshot',
            self._get_scene_snapshot, callback_group=self._action_callback_group)
        self._action_server = ActionServer(
            self,
            InspectScene,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute,
            callback_group=self._action_callback_group,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

    def _get_scene_snapshot(self, request, response):
        cached = self._snapshot_cache.get(request.snapshot_id)
        response.found = cached is not None
        response.message = 'Snapshot missing or expired'
        if cached is not None:
            response.detections = self._detections_message(
                cached.detections, cached.snapshot.stamp_ns, cached.color_frame,
                request.snapshot_id, cached.detection_distances_m,
                cached.tracking, cached.representative_frame)
            response.message = 'Cached snapshot; capture time and TTL unchanged'
        return response

    def destroy_node(self) -> None:
        if self._wrist_service is not None:
            self._wrist_service.close(wait=True)
        self._action_server.destroy()
        super().destroy_node()

    def _declare_parameters(self) -> None:
        self.declare_parameter('enable_wrist_observation', False)
        self.declare_parameter('preload_models', False)
        self.declare_parameter(
            'model_directory',
            os.environ.get('CLEANY_MODEL_DIR', str(Path.home() / 'models')),
        )
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
        self.declare_parameter('detector_type', 'yoloe_gemini')
        self.declare_parameter('segmenter_type', 'yoloe_seg')
        self.declare_parameter('simulation_color_minimum_pixels', 100)
        self.declare_parameter('simulation_color_profile', 'legacy')
        self.declare_parameter('yoloe_model_path', '')
        self.declare_parameter('wrist_yoloe_model_path', '')
        self.declare_parameter('wrist_right_yoloe_model_path', '')
        self.declare_parameter('wrist_check_yoloe_model_path', '')
        self.declare_parameter('wrist_mouse_check_yoloe_model_path', '')
        self.declare_parameter('wrist_right_check_yoloe_model_path', '')
        self.declare_parameter('wrist_failure_image_directory', '')
        self.declare_parameter(
            'wrist_right_check_yoloe_class_confidence_thresholds',
            Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter(
            'yoloe_classes',
            ['cup', 'wallet', 'crumpled tissue', 'lego brick'],
        )
        self.declare_parameter('yoloe_device', 'cuda')
        self.declare_parameter('yoloe_image_size', 640)
        self.declare_parameter('yoloe_iou_threshold', 0.5)
        self.declare_parameter(
            'yoloe_class_confidence_thresholds', Parameter.Type.DOUBLE_ARRAY
        )
        self.declare_parameter('yoloe_text_encoder_directory', '')
        self.declare_parameter('snapshot_timeout_seconds', 2.0)
        self.declare_parameter('depth_16u_scale_m', 0.001)
        self.declare_parameter('tracking_maximum_distance_m', 0.03)
        self.declare_parameter('tracking_ambiguity_margin_m', 0.01)
        self.declare_parameter('tracking_session_ttl_seconds', 600.0)
        self.declare_parameter('tracking_maximum_sessions', 32)
        self.declare_parameter('tracking_label_aliases',
                               [f'{alias}={label}' for alias, label in TrackingConfig().label_aliases])
        self.declare_parameter('snapshot_cache_max_entries', 2)
        self.declare_parameter('snapshot_cache_ttl_seconds', 120.0)
        self.declare_parameter(
            'gemini_model',
            'gemini-robotics-er-2-preview',
        )
        self.declare_parameter('gemini_api_key_environment', 'GEMINI_API_KEY')
        self.declare_parameter('detector_timeout_seconds', 30.0)
        self.declare_parameter('tf_timeout_seconds', 0.5)
        self.declare_parameter('tf_cache_seconds', 60.0)
        self.declare_parameter('minimum_detection_confidence', 0.25)
        self.declare_parameter('maximum_detections', 10)
        self.declare_parameter('minimum_depth_m', 0.1)
        self.declare_parameter('maximum_depth_m', 3.0)
        self.declare_parameter('nearest_object_central_bbox_fraction', 0.5)
        self.declare_parameter('nearest_object_minimum_depth_pixels', 20)
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

    def _resolve_local_model(self, kind: str) -> None:
        keys = (
            'yoloe_model_path', 'yoloe_text_encoder_directory'
        )
        resolved = resolve_model_assets(
            {key: str(self.get_parameter(key).value) for key in keys},
            str(self.get_parameter('model_directory').value), kind,
        )
        import torch

        device_key = kind + '_device'
        requested = str(self.get_parameter(device_key).value)
        resolved[device_key] = resolve_device(
            requested, torch.cuda.is_available()
        )
        for key, value in resolved.items():
            self.set_parameters([Parameter(key, value=value)])
            self.get_logger().debug(f'Perception runtime: {key}={value}')

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
            ranked_detections = rank_detections_by_distance(
                snapshot,
                detections,
                capture_transform,
                self._object_ranking_config,
            )
            detections = tuple(item.detection for item in ranked_detections)
            detection_distances_m = tuple(
                item.distance_m for item in ranked_detections
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
            session_id = goal_handle.request.tracking_session_id
            if session_id and self._target_frame != 'base_link':
                raise InspectionFailure(FailureKind.TF, 'Tracking requires base_link and a fixed base')
            if goal_handle.is_cancel_requested:
                raise InspectionFailure(FailureKind.CANCELLED, 'Observation canceled before tracking update')
            tracking = self._tracker.update(
                session_id, snapshot_id, [d.label for d in detections],
                [item.position for item in ranked_detections],
            )
            self._snapshot_cache.put(
                snapshot_id,
                CachedDetectionSnapshot(
                    snapshot=snapshot,
                    detections=detections,
                    detection_distances_m=detection_distances_m,
                    capture_transform=capture_transform,
                    color_frame=messages.color.header.frame_id,
                    tracking=tracking,
                    representative_frame=self._target_frame,
                ),
            )
            detections_message = self._detections_message(
                detections,
                snapshot.stamp_ns,
                messages.color.header.frame_id,
                snapshot_id,
                detection_distances_m,
                tracking, self._target_frame,
            )
            result.success = True
            result.error_code = InspectScene.Result.ERROR_NONE
            result.message = (
                f'Detected {len(detections)} objects ordered by nearest '
                'target-frame distance; select a distance-valid object_id '
                'for selected-object inspection'
            )
            result.detections = detections_message
            self._detections_publisher.publish(detections_message)
            if self._debug_republish_count:
                debug_rgb = render_debug_image(snapshot.rgb, detections, ())
                self._publish_debug_image(debug_image_message(
                    debug_rgb, snapshot.stamp_ns,
                    messages.color.header.frame_id,
                ))
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
        requested_session = goal_handle.request.tracking_session_id
        if requested_session and (cached.tracking is None or cached.tracking.session_id != requested_session):
            result.error_code = InspectScene.Result.ERROR_INVALID_SELECTION
            result.message = 'Tracking session does not match the cached observation'
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
            cached.detection_distances_m,
            cached.tracking, cached.representative_frame,
        )
        objects_message = self._objects_message(
            output,
            cached.snapshot.stamp_ns,
            snapshot_id,
            (selected_id,),
        )
        if cached.tracking is not None:
            objects_message.tracking_session_id = cached.tracking.session_id
            objects_message.tracking_epoch = cached.tracking.epoch
            for obj in objects_message.objects:
                tracked = cached.tracking.detections[obj.object_id - 1]
                obj.track_id, obj.tracking_state = tracked.track_id, tracked.state.value
        target_cloud, context_cloud = self._selected_cloud_messages(
            cached.snapshot,
            output.masks[0].mask,
            selected,
            cached.capture_transform,
            output.target_frame,
            support_plane=output.plane,
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
        if self._debug_republish_count:
            debug_rgb = render_debug_image(
                cached.snapshot.rgb, output.detections, output.masks,
                object_ids=(selected_id,),
            )
            self._publish_debug_image(debug_image_message(
                debug_rgb, cached.snapshot.stamp_ns, cached.color_frame,
            ))
        goal_handle.succeed()
        return result

    def _selected_cloud_messages(
        self,
        snapshot,
        target_mask,
        detection,
        capture_transform,
        target_frame,
        support_plane=None,
    ):
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
            'minimum_depth_m': float(
                self.get_parameter('minimum_depth_m').value
            ),
            'maximum_depth_m': float(
                self.get_parameter('maximum_depth_m').value
            ),
            'voxel_size_m': float(
                self.get_parameter('grasp_cloud_voxel_size_m').value
            ),
        }
        target = transform_colored_cloud(
            colored_cloud_from_selection(
                selection=target_mask,
                support_plane=support_plane,
                minimum_height_m=float(
                    self.get_parameter('minimum_object_height_m').value),
                maximum_points=int(
                    self.get_parameter('grasp_target_maximum_points').value
                ),
                **common,
            ),
            capture_transform,
        )
        context = transform_colored_cloud(
            colored_cloud_from_selection(
                selection=context_mask,
                maximum_points=int(
                    self.get_parameter('grasp_context_maximum_points').value
                ),
                **common,
            ),
            capture_transform,
        )
        stamp = Time(nanoseconds=snapshot.stamp_ns).to_msg()
        return (
            colored_point_cloud_message(target, stamp, target_frame),
            colored_point_cloud_message(context, stamp, target_frame),
        )

    def _publish_debug_image(self, message: Image) -> None:
        if self._debug_republish_count == 0:
            return
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
        detection_distances_m: Sequence[float | None],
        tracking: TrackingOutput | None = None,
        representative_frame: str = 'base_link',
    ) -> DetectedObject2DArray:
        if len(detections) != len(detection_distances_m):
            raise ValueError('Detection distances must match detections')
        message = DetectedObject2DArray()
        message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        message.header.frame_id = frame_id
        message.snapshot_id = snapshot_id
        message.representative_frame = representative_frame
        if tracking is not None:
            message.tracking_session_id, message.tracking_epoch = tracking.session_id, tracking.epoch
            message.missing_objects = [TrackedObjectReference(**vars(ref)) for ref in tracking.missing]
        for object_id, (detection, distance_m) in enumerate(
            zip(detections, detection_distances_m, strict=True),
            start=1,
        ):
            detected = DetectedObject2D()
            detected.object_id = object_id
            detected.label = detection.label
            detected.confidence = detection.confidence
            detected.sorting_category = detection.sorting_category
            detected.sorting_reason = detection.sorting_reason
            detected.distance_valid = distance_m is not None
            detected.distance_m = 0.0 if distance_m is None else distance_m
            if tracking is not None:
                tracked = tracking.detections[object_id - 1]
                detected.track_id, detected.tracking_state = tracked.track_id, tracked.state.value
                detected.position_valid = tracked.position is not None
                if tracked.position is not None:
                    (detected.representative_position.x, detected.representative_position.y,
                     detected.representative_position.z) = tracked.position
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
