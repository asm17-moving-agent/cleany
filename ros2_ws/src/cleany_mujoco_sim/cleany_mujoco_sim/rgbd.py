from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np
from cleany_interfaces.msg import DetectedObject3D, DetectedObject3DArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from cleany_mujoco_sim.extensions import MujocoSimulationContext
from cleany_mujoco_sim.state import ros_quaternion_from_mujoco


@dataclass(frozen=True)
class GroundTruthSpec:
    geom_name: str
    label: str


@dataclass(frozen=True)
class RgbdSensorConfig:
    width: int = 640
    height: int = 480
    rate_hz: float = 5.0
    camera_name: str = 'head_realsense_rgb'
    base_body_name: str = 'chassis'
    base_frame_id: str = 'base_link'
    color_frame_id: str = 'head_camera_rgb_optical_frame'
    depth_frame_id: str = 'head_camera_depth_optical_frame'
    color_image_topic: str = 'camera/color/image_raw'
    color_info_topic: str = 'camera/color/camera_info'
    depth_image_topic: str = 'camera/depth/image_raw'
    depth_info_topic: str = 'camera/depth/camera_info'
    ground_truth_topic: str = 'ground_truth/objects'

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError('RGB-D width and height must be positive')
        if self.rate_hz <= 0.0 or not math.isfinite(self.rate_hz):
            raise ValueError('RGB-D rate_hz must be finite and positive')
        for field_name in (
            'camera_name',
            'base_body_name',
            'base_frame_id',
            'color_frame_id',
            'depth_frame_id',
            'color_image_topic',
            'color_info_topic',
            'depth_image_topic',
            'depth_info_topic',
            'ground_truth_topic',
        ):
            if not getattr(self, field_name):
                raise ValueError(f'RGB-D {field_name} must not be empty')


@dataclass(frozen=True)
class RgbdCapture:
    color_image: Image
    color_info: CameraInfo
    depth_image: Image
    depth_info: CameraInfo
    ground_truth: DetectedObject3DArray


@dataclass(frozen=True)
class _ResolvedGroundTruth:
    object_id: int
    label: str
    geom_id: int
    size: tuple[float, float, float]


def camera_intrinsics(
    width: int,
    height: int,
    vertical_fov_degrees: float,
) -> tuple[float, float, float, float]:
    if width <= 0 or height <= 0:
        raise ValueError('Camera width and height must be positive')
    if not 0.0 < vertical_fov_degrees < 180.0:
        raise ValueError(
            'Camera vertical FOV must be between 0 and 180 degrees'
        )
    focal_length = 0.5 * height / math.tan(
        0.5 * math.radians(vertical_fov_degrees)
    )
    return (
        focal_length,
        focal_length,
        0.5 * (width - 1),
        0.5 * (height - 1),
    )


def sanitize_depth(depth: np.ndarray, far_plane_m: float) -> np.ndarray:
    if depth.ndim != 2:
        raise ValueError('Depth image must be two-dimensional')
    if far_plane_m <= 0.0 or not math.isfinite(far_plane_m):
        raise ValueError('Far plane distance must be finite and positive')
    result = np.asarray(depth, dtype=np.float32).copy()
    far_threshold = far_plane_m * (1.0 - 1e-5)
    invalid = (
        ~np.isfinite(result)
        | (result <= 0.0)
        | (result >= far_threshold)
    )
    result[invalid] = np.nan
    return result


def camera_info_msg(
    width: int,
    height: int,
    vertical_fov_degrees: float,
    stamp: Time,
    frame_id: str,
) -> CameraInfo:
    fx, fy, cx, cy = camera_intrinsics(
        width,
        height,
        vertical_fov_degrees,
    )
    msg = CameraInfo()
    msg.header.stamp = stamp.to_msg()
    msg.header.frame_id = frame_id
    msg.width = width
    msg.height = height
    msg.distortion_model = 'plumb_bob'
    msg.d = [0.0] * 5
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


def rgb_image_msg(rgb: np.ndarray, stamp: Time, frame_id: str) -> Image:
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError('RGB image must have shape HxWx3 and dtype uint8')
    contiguous = np.ascontiguousarray(rgb)
    msg = Image()
    msg.header.stamp = stamp.to_msg()
    msg.header.frame_id = frame_id
    msg.height = contiguous.shape[0]
    msg.width = contiguous.shape[1]
    msg.encoding = 'rgb8'
    msg.is_bigendian = False
    msg.step = contiguous.shape[1] * 3
    msg.data = contiguous.tobytes()
    return msg


def depth_image_msg(depth: np.ndarray, stamp: Time, frame_id: str) -> Image:
    if depth.ndim != 2:
        raise ValueError('Depth image must be two-dimensional')
    contiguous = np.ascontiguousarray(depth, dtype='<f4')
    msg = Image()
    msg.header.stamp = stamp.to_msg()
    msg.header.frame_id = frame_id
    msg.height = contiguous.shape[0]
    msg.width = contiguous.shape[1]
    msg.encoding = '32FC1'
    msg.is_bigendian = False
    msg.step = contiguous.shape[1] * 4
    msg.data = contiguous.tobytes()
    return msg


class _RgbdRenderer:
    """Render aligned color and metric depth from one OpenGL draw."""

    def __init__(
        self,
        model: mujoco.MjModel,
        width: int,
        height: int,
    ) -> None:
        if width > model.vis.global_.offwidth:
            raise ValueError(
                'RGB-D width exceeds the MuJoCo offscreen framebuffer'
            )
        if height > model.vis.global_.offheight:
            raise ValueError(
                'RGB-D height exceeds the MuJoCo offscreen framebuffer'
            )
        self._model = model
        self._scene = mujoco.MjvScene(model=model, maxgeom=10_000)
        self._scene_option = mujoco.MjvOption()
        self._camera = mujoco.MjvCamera()
        self._camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self._rect = mujoco.MjrRect(0, 0, width, height)
        self._rgb = np.empty((height, width, 3), dtype=np.uint8)
        self._depth = np.empty((height, width), dtype=np.float32)
        self._gl_context = mujoco.GLContext(width, height)
        self._mjr_context: mujoco.MjrContext | None = None
        try:
            self._gl_context.make_current()
            self._mjr_context = mujoco.MjrContext(
                model,
                mujoco.mjtFontScale.mjFONTSCALE_150.value,
            )
            mujoco.mjr_setBuffer(
                mujoco.mjtFramebuffer.mjFB_OFFSCREEN.value,
                self._mjr_context,
            )
            self._mjr_context.readDepthMap = (
                mujoco.mjtDepthMap.mjDEPTH_ZEROFAR
            )
        except Exception:
            self.close()
            raise

    @property
    def closed(self) -> bool:
        return self._mjr_context is None

    def close(self) -> None:
        if self._gl_context is not None:
            self._gl_context.free()
            self._gl_context = None
        if self._mjr_context is not None:
            self._mjr_context.free()
            self._mjr_context = None

    def render(
        self,
        data: mujoco.MjData,
        camera_id: int,
    ) -> tuple[np.ndarray, np.ndarray]:
        if self._mjr_context is None or self._gl_context is None:
            raise RuntimeError('RGB-D renderer is closed')
        self._camera.fixedcamid = camera_id
        self._gl_context.make_current()
        mujoco.mjv_updateScene(
            self._model,
            data,
            self._scene_option,
            None,
            self._camera,
            mujoco.mjtCatBit.mjCAT_ALL.value,
            self._scene,
        )
        mujoco.mjr_render(
            self._rect,
            self._scene,
            self._mjr_context,
        )
        mujoco.mjr_readPixels(
            self._rgb,
            self._depth,
            self._rect,
            self._mjr_context,
        )
        self._rgb[:] = np.flipud(self._rgb)
        self._depth[:] = np.flipud(self._depth)
        self._convert_depth_to_meters()
        return self._rgb, self._depth

    def _convert_depth_to_meters(self) -> None:
        extent = self._model.stat.extent
        znear = np.float32(self._model.vis.map.znear * extent)
        zfar = np.float32(self._model.vis.map.zfar * extent)
        c_coefficient = -(zfar + znear) / (zfar - znear)
        d_coefficient = -(np.float32(2) * zfar * znear) / (
            zfar - znear
        )
        c_coefficient = np.float32(-0.5) * c_coefficient - np.float32(0.5)
        d_coefficient = np.float32(-0.5) * d_coefficient
        metric_depth = d_coefficient / (
            self._depth.astype(np.float64) + c_coefficient
        )
        self._depth[:] = metric_depth.astype(np.float32)


class RgbdSensorBridge:
    def __init__(
        self,
        node: Node,
        context: MujocoSimulationContext,
        config: RgbdSensorConfig,
        ground_truth_specs: tuple[GroundTruthSpec, ...] = (),
    ) -> None:
        self._config = config
        self._camera_id = mujoco.mj_name2id(
            context.model,
            mujoco.mjtObj.mjOBJ_CAMERA,
            config.camera_name,
        )
        if self._camera_id < 0:
            raise ValueError(f'MuJoCo camera not found: {config.camera_name}')
        self._base_body_id = mujoco.mj_name2id(
            context.model,
            mujoco.mjtObj.mjOBJ_BODY,
            config.base_body_name,
        )
        if self._base_body_id < 0:
            raise ValueError(
                f'MuJoCo base body not found: {config.base_body_name}'
            )
        self._ground_truth = tuple(
            self._resolve_ground_truth(context.model, index + 1, spec)
            for index, spec in enumerate(ground_truth_specs)
        )
        self._renderer: _RgbdRenderer | None = _RgbdRenderer(
            context.model,
            width=config.width,
            height=config.height,
        )
        self._next_capture_time: float | None = None
        self._color_image_pub = node.create_publisher(
            Image,
            config.color_image_topic,
            qos_profile_sensor_data,
        )
        self._color_info_pub = node.create_publisher(
            CameraInfo,
            config.color_info_topic,
            qos_profile_sensor_data,
        )
        self._depth_image_pub = node.create_publisher(
            Image,
            config.depth_image_topic,
            qos_profile_sensor_data,
        )
        self._depth_info_pub = node.create_publisher(
            CameraInfo,
            config.depth_info_topic,
            qos_profile_sensor_data,
        )
        self._ground_truth_pub = node.create_publisher(
            DetectedObject3DArray,
            config.ground_truth_topic,
            qos_profile_sensor_data,
        )

    @property
    def closed(self) -> bool:
        return self._renderer is None

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def after_step(
        self,
        context: MujocoSimulationContext,
        stamp: Time,
    ) -> None:
        capture_time = stamp.nanoseconds * 1e-9
        if not self._capture_is_due(capture_time):
            return
        capture = self.capture(context, stamp)
        self._color_image_pub.publish(capture.color_image)
        self._color_info_pub.publish(capture.color_info)
        self._depth_image_pub.publish(capture.depth_image)
        self._depth_info_pub.publish(capture.depth_info)
        self._ground_truth_pub.publish(capture.ground_truth)
        self._schedule_next_capture(capture_time)

    def capture(
        self,
        context: MujocoSimulationContext,
        stamp: Time,
    ) -> RgbdCapture:
        if self._renderer is None:
            raise RuntimeError('RGB-D renderer is closed')
        model = context.model
        data = context.data
        rgb, raw_depth = self._renderer.render(data, self._camera_id)

        expected_rgb_shape = (
            self._config.height,
            self._config.width,
            3,
        )
        expected_depth_shape = (
            self._config.height,
            self._config.width,
        )
        if rgb.shape != expected_rgb_shape or rgb.dtype != np.uint8:
            raise RuntimeError(
                f'Unexpected RGB render output: {rgb.shape}, {rgb.dtype}'
            )
        if raw_depth.shape != expected_depth_shape:
            raise RuntimeError(
                f'Unexpected depth render output: {raw_depth.shape}'
            )

        far_plane_m = float(model.vis.map.zfar * model.stat.extent)
        depth = sanitize_depth(raw_depth, far_plane_m)
        vertical_fov_degrees = float(model.cam_fovy[self._camera_id])
        return RgbdCapture(
            color_image=rgb_image_msg(
                rgb,
                stamp,
                self._config.color_frame_id,
            ),
            color_info=camera_info_msg(
                self._config.width,
                self._config.height,
                vertical_fov_degrees,
                stamp,
                self._config.color_frame_id,
            ),
            depth_image=depth_image_msg(
                depth,
                stamp,
                self._config.depth_frame_id,
            ),
            depth_info=camera_info_msg(
                self._config.width,
                self._config.height,
                vertical_fov_degrees,
                stamp,
                self._config.depth_frame_id,
            ),
            ground_truth=self._ground_truth_msg(context, stamp),
        )

    def _capture_is_due(self, capture_time: float) -> bool:
        if self._next_capture_time is None:
            return True
        period = 1.0 / self._config.rate_hz
        if capture_time < self._next_capture_time - period:
            return True
        return capture_time >= self._next_capture_time - 1e-12

    def _schedule_next_capture(self, capture_time: float) -> None:
        period = 1.0 / self._config.rate_hz
        if (
            self._next_capture_time is None
            or capture_time < self._next_capture_time - period
        ):
            self._next_capture_time = capture_time + period
            return
        elapsed_periods = max(
            1,
            math.floor(
                (capture_time - self._next_capture_time) / period
            )
            + 1,
        )
        self._next_capture_time += elapsed_periods * period

    def _resolve_ground_truth(
        self,
        model: mujoco.MjModel,
        object_id: int,
        spec: GroundTruthSpec,
    ) -> _ResolvedGroundTruth:
        if not spec.geom_name or not spec.label:
            raise ValueError(
                'Ground-truth geom name and label must not be empty'
            )
        geom_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_GEOM,
            spec.geom_name,
        )
        if geom_id < 0:
            raise ValueError(
                f'MuJoCo ground-truth geom not found: {spec.geom_name}'
            )
        geom_type = model.geom_type[geom_id]
        if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
            size_array = 2.0 * model.geom_size[geom_id, :3]
        elif geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
            radius, half_height = model.geom_size[geom_id, :2]
            size_array = np.array(
                (2.0 * radius, 2.0 * radius, 2.0 * half_height)
            )
        else:
            raise ValueError(
                f'Unsupported ground-truth geom type for {spec.geom_name}: '
                f'{int(geom_type)}'
            )
        return _ResolvedGroundTruth(
            object_id=object_id,
            label=spec.label,
            geom_id=geom_id,
            size=tuple(float(value) for value in size_array),
        )

    def _ground_truth_msg(
        self,
        context: MujocoSimulationContext,
        stamp: Time,
    ) -> DetectedObject3DArray:
        data = context.data
        base_rotation = data.xmat[self._base_body_id].reshape(3, 3)
        base_position = data.xpos[self._base_body_id]
        msg = DetectedObject3DArray()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self._config.base_frame_id
        simulation_nanoseconds = round(float(data.time) * 1e9)
        msg.snapshot_id = f'sim-{simulation_nanoseconds:019d}'
        for resolved in self._ground_truth:
            geom_position = data.geom_xpos[resolved.geom_id]
            geom_rotation = data.geom_xmat[resolved.geom_id].reshape(3, 3)
            relative_position = base_rotation.T @ (
                geom_position - base_position
            )
            relative_rotation = base_rotation.T @ geom_rotation
            quaternion_wxyz = np.zeros(4, dtype=np.float64)
            mujoco.mju_mat2Quat(
                quaternion_wxyz,
                relative_rotation.reshape(9),
            )
            quaternion = ros_quaternion_from_mujoco(quaternion_wxyz)

            detected = DetectedObject3D()
            detected.object_id = resolved.object_id
            detected.label = resolved.label
            detected.confidence = 1.0
            detected.obb_pose.position.x = float(relative_position[0])
            detected.obb_pose.position.y = float(relative_position[1])
            detected.obb_pose.position.z = float(relative_position[2])
            detected.obb_pose.orientation.x = quaternion[0]
            detected.obb_pose.orientation.y = quaternion[1]
            detected.obb_pose.orientation.z = quaternion[2]
            detected.obb_pose.orientation.w = quaternion[3]
            detected.obb_size.x = resolved.size[0]
            detected.obb_size.y = resolved.size[1]
            detected.obb_size.z = resolved.size[2]
            msg.objects.append(detected)
        return msg
