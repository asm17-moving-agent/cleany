"""Observe and revalidate human-approved tasks without owning a Mission FSM."""

from collections import deque
import io
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace
from uuid import uuid4

from PIL import Image as PILImage, ImageDraw, ImageFont
from cleany_interfaces.srv import GetSceneSnapshot
from cleany_skill_executor.core.automatic_goal import select_automatic_target
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image

from .camera import encode_image
from .operator_ros import OperatorROS
from .proposals import ProposalStore, fingerprint, scene_context, validate_fresh_target


class ProposalROS(OperatorROS):
    """Retain capture-matched observation images and validate snapshot identity.

    Subscriptions:
        /perception/debug_image_latched (sensor_msgs/msg/Image): Numbered capture;
            topic is configurable. QoS: reliable, transient local, depth 1.
    Services:
        /perception/get_scene_snapshot (GetSceneSnapshot): Read-only cache lookup.
    """

    def __init__(self, root: Path, policy: dict):
        super().__init__()
        self.store, self.policy = ProposalStore(root), policy
        self.images = deque(maxlen=3)
        self.image_subscription = self.node.create_subscription(Image,
            policy['proposal_debug_image_topic'], self.images.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.lookup = self.node.create_client(GetSceneSnapshot, '/perception/get_scene_snapshot')

    @staticmethod
    def config():
        path = os.environ.get('CLEANY_MANIPULATION_COLLECTION_CONFIG')
        if not path:
            raise ValueError('실물 수거 프로파일이 설정되지 않았습니다.')
        return json.loads(Path(path).expanduser().read_text())

    def propose(self, cancelled, *, previous=None, skip=False, last_result=None):
        config = self.config()
        session = previous['tracking_session_id'] if previous else str(uuid4())
        scene = self.inspect_scene(cancelled, tracking_session_id=session,
                                   timeout=self.policy['proposal_inspection_timeout_sec'])
        now = time.time()
        captured = scene.header.stamp.sec + scene.header.stamp.nanosec / 1e9
        if not 0 <= now - captured < self.policy['proposal_max_age_sec']:
            raise ValueError('인식 영상이 오래되었습니다. 최신 영상으로 다시 제안받으세요.')
        excluded = list(previous.get('excluded_tracks', [])) if previous and scene.tracking_epoch == previous['tracking_epoch'] else []
        if skip and previous:
            if scene.tracking_epoch != previous['tracking_epoch']:
                raise ValueError('추적 기준이 변경되어 건너뛸 대상을 확인할 수 없습니다.')
            excluded.append(previous['target']['track_id'])
        proposal = dict(id=str(uuid4()), state='NO_TARGET', planner='nearest_eligible',
            created_at=now, expires_at=captured + self.policy['proposal_max_age_sec'],
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
        if time.time() >= proposal['expires_at']:
            raise ValueError('제안이 만료되었습니다. 다시 제안받으세요.')
        scene = self.inspect_scene(cancelled, tracking_session_id=proposal['tracking_session_id'],
                                   timeout=self.policy['proposal_inspection_timeout_sec'])
        validate_fresh_target(proposal, scene, self.config(), self.policy, time.time())
        cached = self.call(self.lookup, GetSceneSnapshot.Request(snapshot_id=proposal['snapshot_id']))
        if (not cached.found or cached.detections.snapshot_id != proposal['snapshot_id']
                or time.time() >= proposal['expires_at']):
            raise ValueError('승인한 관측이 만료되었습니다. 다시 제안받으세요.')
        if cancelled():
            raise InterruptedError('승인 실행을 취소했습니다.')
