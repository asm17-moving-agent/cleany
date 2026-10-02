"""Selected wrist RGB observation with exact image/info pairing.

YOLOE-seg redetects the projected instance at handoff and check.
Wrist RGB observations do not produce depth or measured 3D poses.
"""
from collections import OrderedDict
from copy import deepcopy
from pathlib import Path
import threading
import time
from uuid import uuid4

import numpy as np
from PIL import Image as PilImage
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data
from cleany_interfaces.srv import ObserveWristTarget
from cleany_perception.adapters.tf2_transform import rotation_matrix_from_quaternion
from cleany_perception.core.wrist_observation import (
    WristObservationConfig, project_box, bbox_iou, verify_mask,
)
from cleany_perception.core.ports import DetectorPort


class WristTargetNotDetected(ValueError):
    """A valid captured frame contains no associated target detection."""


class WristService:
    def __init__(self, node, detector: DetectorPort, *,
                 check_detector: DetectorPort | None = None,
                 mouse_check_detector: DetectorPort | None = None,
                 right_detector: DetectorPort | None = None,
                 right_check_detector: DetectorPort | None = None,
                 failure_image_directory: str = '',
                 config: WristObservationConfig = WristObservationConfig()):
        self.node, self.detector = node, detector
        self.check_detector = check_detector or detector
        self.mouse_check_detector = mouse_check_detector or self.check_detector
        self.right_detector = right_detector or detector
        self.right_check_detector = right_check_detector or self.check_detector
        self.failure_image_directory = Path(failure_image_directory).expanduser() if failure_image_directory else None
        self.config = config
        self.lock = threading.Condition()
        self.images = {arm: OrderedDict() for arm in ('left', 'right')}
        self.infos = {arm: OrderedDict() for arm in ('left', 'right')}
        self.reference = None
        self.subscriptions = []
        for arm in self.images:
            for message, suffix, cache in ((Image, 'image_raw', self.images), (CameraInfo, 'camera_info', self.infos)):
                self.subscriptions.append(node.create_subscription(
                    message, f'/{arm}_wrist_camera/{suffix}',
                    lambda msg, a=arm, c=cache: self.receive(c[a], msg), qos_profile_sensor_data,
                    callback_group=node._sensor_callback_group))
        self.service = node.create_service(ObserveWristTarget, '/perception/observe_wrist_target',
            self.execute, callback_group=node._action_callback_group)

    def receive(self, cache, message):
        key = message.header.stamp.sec*10**9 + message.header.stamp.nanosec
        if key <= 0:
            return
        with self.lock:
            ref = self.reference
            if ref is not None and cache is not self.images[ref['arm']] and cache is not self.infos[ref['arm']]:
                return
            cache[key] = message
            while len(cache) > 4:
                cache.popitem(last=False)
            self.lock.notify_all()

    def capture(self, arm, after):
        deadline = time.monotonic()+self.config.capture_timeout_seconds
        with self.lock:
            while time.monotonic() < deadline:
                keys = self.images[arm].keys() & self.infos[arm].keys()
                for stamp in sorted(keys, reverse=True):
                    age = (self.node.get_clock().now().nanoseconds-stamp)/1e9
                    if stamp > after and 0 <= age <= self.config.maximum_frame_age_seconds:
                        return self.images[arm][stamp], self.infos[arm][stamp], stamp
                self.lock.wait(timeout=0.05)
        raise ValueError('No fresh synchronized wrist RGB/CameraInfo')

    def close(self, *, wait: bool = False):
        with self.lock:
            self.reference = None

    @staticmethod
    def _rgb(msg, info, arm):
        expected_frame = f'{arm}_wrist_rgb_optical_frame'
        if (msg.header.frame_id != expected_frame or info.header != msg.header
                or (info.width, info.height) != (msg.width,msg.height)
                or msg.encoding not in ('rgb8','bgr8') or msg.step < msg.width*3
                or len(msg.data) != msg.step*msg.height
                or any(not np.isfinite(d) or abs(d)>1e-9 for d in info.d)
                or not np.isfinite(info.k).all() or info.k[0] <= 0 or info.k[4] <= 0):
            raise ValueError('Invalid rectified wrist RGB contract')
        rgb = np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:, :msg.width*3]
        rgb = rgb.reshape(msg.height,msg.width,3)
        return np.array(rgb if msg.encoding=='rgb8' else rgb[...,::-1],copy=True)

    def _reference_expired(self, ref):
        return time.monotonic()-ref.get('last_valid_at',ref['created']) > self.config.reference_ttl_seconds

    def _yoloe_mask(self, rgb, box, label, detector=None, fallback_detector=None):
        active_detector = detector or self.detector
        detections = active_detector.detect(rgb, '')
        expected_label = label.casefold().strip()
        def associated(items):
            if any(not np.isfinite(d.confidence) or not 0 <= d.confidence <= 1 for d in items):
                raise ValueError('Invalid wrist detection confidence')
            return [d for d in items
                    if f' {d.label.casefold().strip()} ' in f' {expected_label} '
                    and self.config.minimum_detection_confidence <= d.confidence <= 1
                    and bbox_iou(d.bbox, box) >= self.config.minimum_detection_iou]

        candidates = associated(detections)
        if not candidates and fallback_detector is not None and fallback_detector is not active_detector:
            fallback_detections = fallback_detector.detect(rgb, '')
            candidates = associated(fallback_detections)
            if candidates:
                self.node.get_logger().info(
                    f'WRIST YOLOE fallback associated {label} with projected box')
            else:
                detections = tuple(detections) + tuple(fallback_detections)
        if not candidates:
            if self.failure_image_directory is not None:
                self.failure_image_directory.mkdir(parents=True, exist_ok=True)
                PilImage.fromarray(rgb, mode='RGB').save(
                    self.failure_image_directory / f'wrist_missing_{time.time_ns()}.png')
            matching_labels = [d for d in detections
                               if f' {d.label.casefold().strip()} ' in f' {expected_label} ']
            strongest = max(
                ((d.confidence, bbox_iou(d.bbox, box)) for d in matching_labels),
                default=None)
            raise WristTargetNotDetected(
                f'Wrist YOLOE needs one projected {label}; found 0 '
                f'(class_detections={len(matching_labels)}, '
                f'best_confidence_iou={strongest})'
            )
        candidates.sort(key=lambda detection: detection.confidence, reverse=True)
        mask = candidates[0].segmentation_mask
        if mask is None or np.shape(mask) != rgb.shape[:2] or np.asarray(mask).dtype != np.bool_:
            raise ValueError('Wrist YOLOE instance mask is missing or mismatched')
        for candidate in candidates[1:]:
            other_mask = candidate.segmentation_mask
            if (other_mask is None or np.shape(other_mask) != rgb.shape[:2]
                    or np.asarray(other_mask).dtype != np.bool_
                    or bbox_iou(candidate.bbox, candidates[0].bbox) < 0.8):
                raise ValueError(f'Wrist YOLOE needs one projected {label}; found {len(candidates)}')
            union = np.count_nonzero(mask | other_mask)
            if not union or np.count_nonzero(mask & other_mask) / union < 0.8:
                raise ValueError(f'Wrist YOLOE needs one projected {label}; found {len(candidates)}')
        return mask


    def execute(self, request, response):
        with self.node._busy_lock:
            if self.node._busy:
                response.status = response.ERROR
                response.message = 'Perception busy'
                return response
            self.node._busy = True
        try:
            self.process(request, response)
            response.success = True
            response.status = response.OK
        except WristTargetNotDetected as error:
            response.success = False
            response.status = response.NOT_DETECTED
            response.message = str(error)
        except Exception as error:
            response.success = False
            response.status = response.ERROR
            response.message = str(error)
        finally:
            with self.node._busy_lock:
                self.node._busy = False
        return response

    def process(self, req, out):
        if req.arm not in self.images or req.operation not in (req.HANDOFF, req.CHECK, req.CLEAR):
            raise ValueError('Invalid wrist operation or arm')
        if req.operation != req.HANDOFF:
            ref = self.reference
            if (ref is None or ref['id'] != req.reference_id or ref['arm'] != req.arm
                    or ref['source'] != req.source_snapshot_id or ref['object'] != req.source_object_id):
                raise ValueError('Wrist reference identity mismatch')
            if req.operation == req.CLEAR:
                self.close()
                return
            if self._reference_expired(ref):
                raise ValueError('Wrist reference expired')
        else:
            # Failed replacement must not leave a usable old reference.
            self.close()
            if (not req.source_snapshot_id or not req.source_object_id or not req.label
                    or not np.isfinite(req.source_confidence)
                    or not self.config.minimum_detection_confidence <= req.source_confidence <= 1):
                raise ValueError('Invalid head observation provenance')
            prior_stamp = req.expected_pose.header.stamp
            prior_ns = prior_stamp.sec*10**9 + prior_stamp.nanosec
            prior_age = (self.node.get_clock().now().nanoseconds-prior_ns)/1e9
            if prior_ns <= 0 or not 0 <= prior_age <= self.config.maximum_head_prior_age_seconds:
                raise ValueError('Missing, stale or future head observation timestamp')
        if req.expected_pose.header.frame_id != 'base_link':
            raise ValueError('Expected object pose must be in base_link')
        msg, info, stamp = self.capture(req.arm, req.after_stamp_ns)
        rgb = self._rgb(msg, info, req.arm)
        expected_frame = f'{req.arm}_wrist_rgb_optical_frame'
        # Show the active sensor even if association/inference subsequently fails.
        if (getattr(self.node, '_debug_republish_count', 1) > 0
                and hasattr(self.node, '_publish_debug_image')):
            debug=deepcopy(msg); debug.encoding='rgb8'; debug.step=msg.width*3
            debug.data=rgb.tobytes(); self.node._publish_debug_image(debug)
        pose = req.expected_pose.pose
        q = pose.orientation
        rotation = rotation_matrix_from_quaternion(q.x,q.y,q.z,q.w)
        transform = self.node._transformer.lookup(expected_frame,'base_link',stamp)
        box = project_box((pose.position.x,pose.position.y,pose.position.z),rotation,
            (req.size.x,req.size.y,req.size.z),transform,np.array(info.k).reshape(3,3),
            msg.width,msg.height,self.config)
        # A missing target still carries the validated observation's provenance.
        # Consumers can retry after this capture without inventing a detection.
        out.source_snapshot_id = req.source_snapshot_id
        out.source_object_id = req.source_object_id
        out.header = deepcopy(msg.header)
        if req.operation == req.HANDOFF:
            mask = self._yoloe_mask(rgb, box, req.label,
                detector=self.right_detector if req.arm == 'right' else self.detector,
                fallback_detector=self.detector if req.arm == 'right' else None)
        else:
            if (tuple(info.k) != ref['k'] or rgb.shape != ref['shape']
                    or stamp <= ref['stamp']):
                raise ValueError('Wrist calibration/shape/timestamp changed')
            out.reference_id = ref['id']
            ref['stamp'] = stamp  # Consume failed checks too; retries need a new frame.
            mask = self._yoloe_mask(
                rgb, box, req.label,
                detector=(self.right_check_detector if req.arm == 'right'
                          else self.mouse_check_detector
                          if 'computer mouse' in req.label.casefold()
                          else self.check_detector),
                fallback_detector=self.check_detector if req.arm == 'right' else None)
        if np.shape(mask) != (msg.height, msg.width):
            raise ValueError('Wrist mask shape differs from captured image')
        verify_mask(mask,box,self.config)
        if req.operation == req.HANDOFF:
            with self.lock:
                self.reference = dict(id=uuid4().hex,arm=req.arm,source=req.source_snapshot_id,
                    object=req.source_object_id,created=time.monotonic(),stamp=stamp,
                    shape=rgb.shape,
                    k=tuple(info.k))
                other = 'right' if req.arm == 'left' else 'left'
                self.images[other].clear()
                self.infos[other].clear()
        out.reference_id = self.reference['id']
        out.source_snapshot_id=req.source_snapshot_id; out.source_object_id=req.source_object_id
        out.header=deepcopy(msg.header)
        out.mask.header=deepcopy(msg.header); out.mask.width=msg.width; out.mask.height=msg.height
        out.mask.encoding='mono8'; out.mask.step=msg.width
        out.mask.data=(np.asarray(mask,dtype=np.uint8)*255).tobytes()
        out.message = 'Wrist YOLOE-seg target consistency passed; no fresh depth or measured 3D pose'
        if getattr(self.node, '_debug_republish_count', 1) > 0:
            debug=deepcopy(msg); debug.encoding='rgb8'
            overlay=rgb.copy(); overlay[mask]=(overlay[mask]*0.5+np.array((0,255,0))*0.5).astype(np.uint8)
            debug.step=msg.width*3; debug.data=overlay.tobytes()
            if hasattr(self.node, '_publish_debug_image'):
                self.node._publish_debug_image(debug)
            else:
                self.node._debug_publisher.publish(debug)
