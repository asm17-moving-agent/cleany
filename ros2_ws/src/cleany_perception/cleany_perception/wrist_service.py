"""Selected wrist RGB, exact image/info pairing, shared learned models.

Handoff segments the projected head observation; optional redetection is explicit.
Checks track that wrist reference, never return synthetic depth/3D poses.
"""
from collections import OrderedDict
from copy import deepcopy
import threading
import time
from uuid import uuid4

import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data
from cleany_interfaces.srv import ObserveWristTarget
from cleany_interfaces.msg import WristTrackingStatus
from cleany_perception.adapters.tf2_transform import rotation_matrix_from_quaternion
from cleany_perception.core.wrist_observation import (
    WristObservationConfig, project_box, bbox_iou, verify_mask,
)
from cleany_perception.core.models import Detection2D
from cleany_perception.core.ports import DetectorPort, SegmenterPort, SequenceReferenceTrackerPort
from cleany_perception.core.continuous_tracking import ContinuousTracking, TrackingFrame


class WristService:
    def __init__(self, node, detector: DetectorPort, segmenter: SegmenterPort,
                 tracker: SequenceReferenceTrackerPort, *, require_redetection: bool = False,
                 continuous_tracking: bool = False,
                 config: WristObservationConfig = WristObservationConfig()):
        self.node, self.detector, self.segmenter, self.tracker = node, detector, segmenter, tracker
        self.require_redetection = require_redetection
        self.config = config
        self.continuous_tracking = continuous_tracking
        if continuous_tracking and not hasattr(tracker, 'start_stream'):
            raise ValueError('Continuous wrist observation requires a streaming tracker')
        self.worker = None
        self.tracking_publisher = node.create_publisher(WristTrackingStatus, '/perception/wrist_tracking_status', 10)
        self.lock = threading.Condition()
        self.images = {arm: OrderedDict() for arm in ('left', 'right')}
        self.infos = {arm: OrderedDict() for arm in ('left', 'right')}
        self.reference = None
        self.history = OrderedDict()
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
            cache[key] = message
            while len(cache) > 4:
                cache.popitem(last=False)
            ref = self.reference
            if (not self.continuous_tracking and ref is not None and cache is self.images[ref['arm']] and key > ref['stamp']
                    and message.header.frame_id == f"{ref['arm']}_wrist_rgb_optical_frame"):
                previous = next(reversed(self.history), ref['stamp'])
                if key-previous >= self.config.history_period_seconds*1e9:
                    self.history[key] = message
                    while len(self.history) > self.config.history_maximum_frames:
                        self.history.popitem(last=False)
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
        worker, self.worker = self.worker, None
        with self.lock:
            self.reference = None
            self.history.clear()
        if worker is not None:
            worker.close(timeout_seconds=5. if wait else 0.)

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

    def _start_tracking(self, ref):
        def next_frame(after):
            if self.reference is not ref or self._reference_expired(ref):
                raise ValueError('Wrist reference cleared or expired')
            msg, info, stamp = self.capture(ref['arm'], after)
            rgb = self._rgb(msg, info, ref['arm'])
            if tuple(info.k) != ref['k'] or rgb.shape != ref['rgb'].shape:
                raise ValueError('Streaming wrist calibration or RGB shape changed')
            return TrackingFrame(stamp, rgb, (msg, info))

        def report(result):
            if self.reference is not ref:
                return
            pixels = int(np.count_nonzero(result.mask))
            visible = self.config.minimum_mask_pixels <= pixels <= result.mask.size*self.config.maximum_mask_fraction
            age = (self.node.get_clock().now().nanoseconds-result.frame.stamp_ns)/1e9
            # Renew an active lease only from a fresh, advancing, visible
            # observation. Missing/stale/replayed results cannot keep it alive.
            with self.lock:
                if (self.reference is ref and not self._reference_expired(ref) and visible
                        and 0 <= age <= self.config.maximum_tracking_result_age_seconds
                        and result.frame.stamp_ns > ref.get('last_tracking_stamp',ref['stamp'])):
                    ref['last_valid_at'] = time.monotonic()
                    ref['last_tracking_stamp'] = result.frame.stamp_ns
            status = WristTrackingStatus(header=deepcopy(result.frame.metadata[0].header),
                arm=ref['arm'], reference_id=ref['id'], source_snapshot_id=ref['source'],
                source_object_id=ref['object'], valid=True, visible=visible,
                mask_pixels=pixels, image_pixels=result.mask.size,
                reason='visible' if visible else 'target_missing_or_invalid_mask_area')
            self.tracking_publisher.publish(status)
            self.node.get_logger().info(
                f'WRIST TRACK frame={result.processed_frames} capture_ns={result.frame.stamp_ns} '
                f'inference_sec={result.inference_seconds:.3f} '
                f'result_age_sec={(self.node.get_clock().now().nanoseconds-result.frame.stamp_ns)/1e9:.3f}')

        def failed(error):
            if self.reference is ref:
                self.tracking_publisher.publish(WristTrackingStatus(
                    arm=ref['arm'], reference_id=ref['id'], source_snapshot_id=ref['source'],
                    source_object_id=ref['object'], valid=False, visible=False, reason=str(error)))

        self.worker = ContinuousTracking(
            lambda: self.tracker.start_stream(ref['rgb'], ref['mask'],
                memory_frames=self.config.streaming_memory_frames,
                cpu_threads=self.config.streaming_cpu_threads),
            next_frame, ref['stamp'], report, failed)

    def execute(self, request, response):
        with self.node._busy_lock:
            if self.node._busy:
                response.message = 'Perception busy'
                return response
            self.node._busy = True
        try:
            self.process(request, response)
            response.success = True
        except Exception as error:
            response.success = False
            response.message = str(error)
            ref = self.reference
            if (request.operation == request.CHECK and ref is not None
                    and request.reference_id == ref['id'] and self.worker is not None):
                # A failed authorization check cannot leave useful work running
                # indefinitely while the robot holds and the mission is stopped.
                self.worker.close()
                self.worker = None
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
        tracked = None
        if req.operation == req.CHECK and self.continuous_tracking:
            if self.worker is None:
                raise ValueError('Missing continuous wrist tracker')
            started = time.monotonic()
            tracked = self.worker.wait_after(max(req.after_stamp_ns, ref['stamp']),
                timeout_seconds=self.config.tracking_check_timeout_seconds,
                now_ns=lambda: self.node.get_clock().now().nanoseconds,
                maximum_age_seconds=self.config.maximum_tracking_result_age_seconds)
            msg, info = tracked.frame.metadata
            stamp, rgb = tracked.frame.stamp_ns, tracked.frame.rgb
            self.node.get_logger().info(f'WRIST CHECK wait_sec={time.monotonic()-started:.3f} '
                f'capture_ns={stamp} after_ns={req.after_stamp_ns} processed={tracked.processed_frames}')
        else:
            msg, info, stamp = self.capture(req.arm, req.after_stamp_ns)
            rgb = self._rgb(msg, info, req.arm)
        expected_frame = f'{req.arm}_wrist_rgb_optical_frame'
        # Show the active sensor even if association/inference subsequently fails.
        if hasattr(self.node, '_publish_debug_image'):
            debug=deepcopy(msg); debug.encoding='rgb8'; debug.step=msg.width*3
            debug.data=rgb.tobytes(); self.node._publish_debug_image(debug)
        pose = req.expected_pose.pose
        q = pose.orientation
        rotation = rotation_matrix_from_quaternion(q.x,q.y,q.z,q.w)
        transform = self.node._transformer.lookup(expected_frame,'base_link',stamp)
        box = project_box((pose.position.x,pose.position.y,pose.position.z),rotation,
            (req.size.x,req.size.y,req.size.z),transform,np.array(info.k).reshape(3,3),
            msg.width,msg.height,self.config)
        if req.operation == req.HANDOFF:
            if self.require_redetection:
                detections = self.detector.detect(rgb, f'Detect the {req.label}.')
                candidates = [d for d in detections if d.label == req.label
                              and self.config.minimum_detection_confidence <= d.confidence <= 1
                              and bbox_iou(d.bbox,box) >= self.config.minimum_detection_iou]
                if len(candidates) != 1:
                    raise ValueError(f'Wrist handoff needs one associated {req.label}; found {len(candidates)}')
                detection = candidates[0]
            else:
                # Prompt provenance remains the head detection, not a fabricated
                # wrist semantic result. Geometry came from head RGB-D and TF.
                detection = Detection2D(req.label,req.source_confidence,box)
            masks = self.segmenter.segment(rgb,(detection,))
            if (len(masks)!=1 or not np.isfinite(masks[0].score)
                    or not self.config.minimum_segmentation_score <= masks[0].score <= 1):
                raise ValueError('Missing or low-quality wrist segmentation')
            mask = masks[0].mask
        elif tracked is not None:
            mask = tracked.mask
        else:
            if (tuple(info.k) != ref['k'] or rgb.shape != ref['rgb'].shape or stamp <= ref['stamp']):
                raise ValueError('Wrist calibration/shape/timestamp changed')
            with self.lock:
                support = [image for key, image in self.history.items() if ref['stamp'] < key < stamp]
            if len(support) > self.config.tracking_support_frames:
                indices = np.linspace(0, len(support)-1, self.config.tracking_support_frames, dtype=int)
                support = [support[index] for index in indices]
            frames = [ref['rgb']]
            for image in support:
                if ((image.width, image.height) != (msg.width, msg.height)
                        or image.encoding not in ('rgb8', 'bgr8') or image.step < image.width*3
                        or len(image.data) != image.step*image.height):
                    raise ValueError('Invalid intermediate wrist RGB frame')
                pixels = np.frombuffer(image.data, np.uint8).reshape(image.height, image.step)
                pixels = pixels[:, :image.width*3].reshape(image.height, image.width, 3)
                frames.append(np.array(pixels if image.encoding == 'rgb8' else pixels[..., ::-1], copy=True))
            frames.append(rgb)
            # Preserve the already validated SAM2 identity, not a newly guessed
            # object within the original projected box (which can include a jaw).
            mask = self.tracker.track_sequence(frames, ref['detection'], reference_mask=ref['mask'])
        if np.shape(mask) != (msg.height, msg.width):
            raise ValueError('Wrist mask shape differs from captured image')
        verify_mask(mask,box,self.config)
        if req.operation == req.HANDOFF:
            with self.lock:
                self.reference = dict(id=uuid4().hex,arm=req.arm,source=req.source_snapshot_id,
                    object=req.source_object_id,created=time.monotonic(),stamp=stamp,
                    rgb=rgb,mask=np.array(mask, copy=True),detection=detection,k=tuple(info.k))
            if self.continuous_tracking:
                self._start_tracking(self.reference)
        out.reference_id = self.reference['id']
        out.source_snapshot_id=req.source_snapshot_id; out.source_object_id=req.source_object_id
        out.header=deepcopy(msg.header)
        out.mask.header=deepcopy(msg.header); out.mask.width=msg.width; out.mask.height=msg.height
        out.mask.encoding='mono8'; out.mask.step=msg.width
        out.mask.data=(np.asarray(mask,dtype=np.uint8)*255).tobytes()
        out.message='Wrist RGB target consistency passed; original head label retained, no fresh depth or measured 3D pose'
        debug=deepcopy(msg); debug.encoding='rgb8'
        overlay=rgb.copy(); overlay[mask]=(overlay[mask]*0.5+np.array((0,255,0))*0.5).astype(np.uint8)
        debug.step=msg.width*3; debug.data=overlay.tobytes()
        if hasattr(self.node, '_publish_debug_image'):
            self.node._publish_debug_image(debug)
        else:
            self.node._debug_publisher.publish(debug)
