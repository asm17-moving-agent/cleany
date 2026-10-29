"""Observe and revalidate human-approved tasks without owning a Mission FSM."""

from collections import deque
import io
import json
import math
import os
from pathlib import Path
import time
from types import SimpleNamespace
from uuid import uuid4

from PIL import Image as PILImage, ImageDraw, ImageFont
from cleany_interfaces.srv import GetSceneSnapshot, RevalidateTarget
from cleany_skill_executor.core.automatic_goal import select_automatic_target
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image

from .camera import encode_image
from .operator_ros import OperatorROS
from .proposals import ProposalStore, fingerprint, scene_context, validate_fresh_target, write_json


class ProposalROS(OperatorROS):
    """Retain capture-matched observation images and validate snapshot identity.

    Subscriptions:
        /perception/debug_image_latched (sensor_msgs/msg/Image): Numbered capture;
            topic is configurable. QoS: reliable, transient local, depth 1.
    Services:
        /perception/get_scene_snapshot (GetSceneSnapshot): Read-only cache lookup.
        /perception/revalidate_target (RevalidateTarget): Fresh local detection
            and depth for the approved target, with frozen source semantics.
    """

    def __init__(self, root: Path, policy: dict):
        super().__init__()
        self.store, self.policy = ProposalStore(root), policy
        self.images = deque(maxlen=3)
        self.image_subscription = self.node.create_subscription(Image,
            policy['proposal_debug_image_topic'], self.images.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.lookup = self.node.create_client(GetSceneSnapshot, '/perception/get_scene_snapshot')
        self.target_observation = self.node.create_client(RevalidateTarget, '/perception/revalidate_target')

    @staticmethod
    def config():
        path = os.environ.get('CLEANY_MANIPULATION_COLLECTION_CONFIG')
        if not path:
            raise ValueError('실물 수거 프로파일이 설정되지 않았습니다.')
        return json.loads(Path(path).expanduser().read_text())

    def propose(self, cancelled, *, previous=None, skip=False, last_result=None):
        """Select a target, reusing the original observation when skipping."""
        if cancelled():
            raise InterruptedError('제안을 취소했습니다.')
        config = self.config()
        session = previous['tracking_session_id'] if previous else str(uuid4())
        if skip:
            if not previous or previous.get('state') != 'READY':
                raise ValueError('건너뛸 제안이 없습니다. 다시 제안받으세요.')
            cached = self.call(self.lookup, GetSceneSnapshot.Request(snapshot_id=previous['snapshot_id']))
            if not cached.found or cached.detections.snapshot_id != previous['snapshot_id']:
                raise ValueError('제안 관측이 만료되었습니다. 다시 제안받으세요.')
            scene = cached.detections
            validate_fresh_target(previous, scene, config, self.policy, time.time())
        else:
            scene = self.inspect_scene(cancelled, tracking_session_id=session,
                                       timeout=self.policy['proposal_inspection_timeout_sec'])
        now = time.time()
        captured = scene.header.stamp.sec + scene.header.stamp.nanosec / 1e9
        if not 0 <= now - captured < self.policy['proposal_max_age_sec']:
            raise ValueError('인식 영상이 오래되었습니다. 최신 영상으로 다시 제안받으세요.')
        expires_at = captured + self.policy['proposal_max_age_sec']
        if skip:
            expires_at = min(expires_at, previous['expires_at'])
        excluded = list(previous.get('excluded_tracks', [])) if previous and scene.tracking_epoch == previous['tracking_epoch'] else []
        if skip:
            excluded.append(previous['target']['track_id'])
        proposal = dict(id=str(uuid4()), state='NO_TARGET', planner='nearest_eligible',
            created_at=now, expires_at=expires_at,
            captured_at=captured, snapshot_id=scene.snapshot_id, tracking_session_id=session,
            tracking_epoch=scene.tracking_epoch, representative_frame=scene.representative_frame,
            config_fingerprint=fingerprint(config), operation_mode=config.get('operation_mode'),
            excluded_tracks=sorted(set(excluded)), last_result=last_result,
            scene_context=scene_context(scene), scene_object_count=len(scene.detections),
            reason='현재 관측에서 실행 가능한 제안이 없습니다. 작업 완료를 의미하지는 않습니다.')
        eligible = [item for item in scene.detections if item.tracking_state == 'TRACKED'
                    and item.position_valid and item.track_id not in excluded]
        try:
            selected = select_automatic_target(SimpleNamespace(snapshot_id=scene.snapshot_id, detections=eligible), config)
        except ValueError:
            selected = None
        if selected:
            item = next(item for item in eligible if item.object_id == selected.object_id)
            proposal.update(state='READY', skill_name=selected.skill_name, destination_id=selected.destination_id,
                reason='분류·신뢰도·깊이·크기·금지 목록·목적지 조건을 통과한 물체 중 로봇에 가장 가깝습니다.',
                target=dict(object_id=item.object_id, track_id=item.track_id, label=item.label,
                    detector_label=item.detector_label,
                    category=item.sorting_category,
                    confidence=float(item.confidence), distance_m=float(item.distance_m),
                    bbox=[item.x_min, item.y_min, item.x_max, item.y_max],
                    position=[item.representative_position.x, item.representative_position.y, item.representative_position.z]))
        jpeg = self.capture_image(scene, proposal.get('target'), cancelled)
        if jpeg is None and proposal['state'] == 'READY':
            raise ValueError('제안 관측과 일치하는 이미지를 받지 못했습니다. 다시 제안받으세요.')
        proposal['image_available'] = jpeg is not None
        if cancelled():
            raise InterruptedError('제안을 취소했습니다.')
        self.store.save(proposal, jpeg)
        return proposal

    def capture_image(self, scene, target, cancelled):
        deadline = time.monotonic() + 3.
        while not cancelled() and time.monotonic() < deadline:
            matching = [message for message in list(self.images) if message.header == scene.header]
            if matching:
                message = matching[-1]
                jpeg, width, height = encode_image(message)
                image = PILImage.open(io.BytesIO(jpeg)).convert('RGB')
                if target:
                    box = target['bbox']
                    draw = ImageDraw.Draw(image)
                    draw.rectangle((box[0]*width/message.width, box[1]*height/message.height,
                        box[2]*width/message.width, box[3]*height/message.height), outline='#00efb0', width=5)
                    # Detector debug labels may use pre-sort numbering. The thick
                    # overlay explicitly identifies the approved snapshot-local ID.
                    try:
                        font = ImageFont.truetype('DejaVuSans-Bold.ttf', 18)
                    except OSError:
                        font = ImageFont.load_default()
                    label = f"TARGET #{target['object_id']}"
                    left, top = box[0]*width/message.width, max(0., box[1]*height/message.height-26)
                    bounds = draw.textbbox((left+4, top+2), label, font=font)
                    draw.rectangle((left, top, bounds[2]+4, top+26), fill='#00efb0')
                    draw.text((left+4, top+2), label, fill='#002b21', font=font)
                output = io.BytesIO()
                image.save(output, format='JPEG', quality=90)
                return output.getvalue()
            time.sleep(.05)
        return None

    def revalidate(self, proposal, cancelled):
        audit = dict(proposal_id=proposal['id'], mode='local_target_frozen_semantics',
                     started_at=time.time(), approved_target=proposal['target'], attempts=[], success=False)
        try:
            if cancelled():
                raise InterruptedError('승인 실행을 취소했습니다.')
            config = self.config()
            cached = self.call(self.lookup, GetSceneSnapshot.Request(snapshot_id=proposal['snapshot_id']))
            if not cached.found or cached.detections.snapshot_id != proposal['snapshot_id']:
                raise ValueError('승인한 관측이 만료되었습니다. 다시 제안받으세요.')
            validate_fresh_target(proposal, cached.detections, config, self.policy, time.time())
            for attempt in range(self.policy.get('proposal_reobservation_attempts', 2)):
                requested_ns = time.time_ns()
                response = self._observe_target(proposal, cancelled, requested_ns)
                entry = dict(attempt=attempt + 1, requested_at_ns=requested_ns,
                             success=response.success, error_code=response.error_code, message=response.message)
                audit['attempts'].append(entry)
                if not response.success:
                    if (response.error_code in (response.ERROR_MASK, response.ERROR_RGBD_TIMEOUT)
                            and attempt + 1 < self.policy.get('proposal_reobservation_attempts', 2)):
                        continue
                    raise ValueError(f'{response.message} 다시 제안받으세요.')
                scene = response.detections
                entry['observation'] = self._observation_record(scene)
                captured_ns = scene.header.stamp.sec * 1_000_000_000 + scene.header.stamp.nanosec
                if (response.source_snapshot_id != proposal['snapshot_id']
                        or response.source_object_id != proposal['target']['object_id']):
                    raise ValueError('재확인 응답이 승인한 관측 또는 대상을 참조하지 않습니다.')
                if not requested_ns < captured_ns <= time.time_ns():
                    raise ValueError('재확인 영상의 촬영 시각이 유효하지 않습니다.')
                validate_fresh_target(proposal, scene, self.config(), self.policy, time.time())
                break
            cached = self.call(self.lookup, GetSceneSnapshot.Request(snapshot_id=proposal['snapshot_id']))
            if not cached.found or cached.detections.snapshot_id != proposal['snapshot_id']:
                raise ValueError('승인한 관측이 만료되었습니다. 다시 제안받으세요.')
            validate_fresh_target(proposal, cached.detections, self.config(), self.policy, time.time())
            if cancelled():
                raise InterruptedError('승인 실행을 취소했습니다.')
            audit['success'] = True
        except Exception as error:
            audit['message'] = str(error)
            raise
        finally:
            audit['finished_at'] = time.time()
            write_json(self.store.root / 'revalidations' / f"{proposal['id']}.json", audit)

    def _observe_target(self, proposal, cancelled, requested_ns):
        client = self.target_observation
        if not client.wait_for_service(timeout_sec=.5):
            raise RuntimeError('승인 대상 재확인 서비스가 없습니다. 인식기 업데이트를 확인하세요.')
        future = client.call_async(RevalidateTarget.Request(
            source_snapshot_id=proposal['snapshot_id'], source_object_id=proposal['target']['object_id'],
            tracking_session_id=proposal['tracking_session_id'], track_id=proposal['target']['track_id'],
            after_stamp_ns=requested_ns))
        deadline = time.monotonic() + self.policy['proposal_inspection_timeout_sec']
        try:
            while not future.done():
                if cancelled():
                    raise InterruptedError('승인 실행을 취소했습니다.')
                if time.time() >= proposal['expires_at'] or time.monotonic() >= deadline:
                    raise TimeoutError('승인 대상 재확인 시간이 만료되었습니다. 다시 제안받으세요.')
                time.sleep(.05)
            return future.result()
        finally:
            if not future.done():
                future.cancel()

    @staticmethod
    def _observation_record(scene):
        finite = lambda value: float(value) if math.isfinite(value) else None
        return dict(snapshot_id=scene.snapshot_id, tracking_session_id=scene.tracking_session_id,
            tracking_epoch=scene.tracking_epoch, representative_frame=scene.representative_frame,
            captured_at_ns=scene.header.stamp.sec * 1_000_000_000 + scene.header.stamp.nanosec,
            targets=[dict(object_id=item.object_id, track_id=item.track_id, tracking_state=item.tracking_state,
                label=item.label, detector_label=item.detector_label, category=item.sorting_category,
                confidence=finite(item.confidence), position_valid=item.position_valid,
                position=[finite(item.representative_position.x), finite(item.representative_position.y),
                          finite(item.representative_position.z)],
                bbox=[finite(item.x_min), finite(item.y_min), finite(item.x_max), finite(item.y_max)],
                median_depth_m=finite(item.median_depth_m), estimated_width_m=finite(item.estimated_width_m),
                estimated_height_m=finite(item.estimated_height_m)) for item in scene.detections])
