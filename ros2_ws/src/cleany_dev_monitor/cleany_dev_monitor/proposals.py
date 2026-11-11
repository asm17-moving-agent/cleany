"""Immutable operator proposals and approval checks, independent of ROS."""

import hashlib
import json
import math
from pathlib import Path
import tempfile
import time
from uuid import UUID


DEFAULTS = {
    'proposal_max_age_sec': 90.0,
    'proposal_position_tolerance_m': 0.015,
    'proposal_minimum_bbox_iou': 0.7,
    'proposal_debug_image_topic': '/perception/debug_image_latched',
    'proposal_inspection_timeout_sec': 180.0,
    'proposal_execution_timeout_sec': 1200.0,
    'proposal_reobservation_attempts': 2,
}


def policy_settings(config: dict) -> dict:
    values = {key: config.get(key, default) for key, default in DEFAULTS.items()}
    for key in DEFAULTS:
        if key.endswith('_topic'):
            if not isinstance(values[key], str) or not values[key].startswith('/'):
                raise ValueError('제안 이미지 토픽은 절대 경로여야 합니다.')
        elif not isinstance(values[key], (int, float)) or not math.isfinite(values[key]) or values[key] <= 0:
            raise ValueError(f'Invalid proposal setting: {key}')
    if values['proposal_minimum_bbox_iou'] > 1:
        raise ValueError('Proposal bbox IoU must not exceed one')
    attempts = values['proposal_reobservation_attempts']
    if isinstance(attempts, bool) or not isinstance(attempts, int) or not 1 <= attempts <= 3:
        raise ValueError('Proposal reobservation attempts must be an integer from one to three')
    return values


def identifier(value: str) -> str:
    return str(UUID(value))


def fingerprint(config: dict) -> str:
    return hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest()


def scene_context(scene) -> list[dict]:
    return [dict(track_id=item.track_id, label=item.label, category=item.sorting_category,
                 position=[item.representative_position.x, item.representative_position.y,
                           item.representative_position.z])
            for item in scene.detections if item.position_valid and item.tracking_state == 'TRACKED']


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
        json.dump(value, stream, allow_nan=False)
        temporary = Path(stream.name)
    temporary.replace(path)


class ProposalStore:
    """Approval jobs are the durable consumption ledger, including failed jobs."""

    def __init__(self, root: Path):
        self.root = root
        self.directory = root / 'proposals'
        self.pointer = root / 'current-proposal.json'

    def invalidate(self) -> None:
        write_json(self.pointer, {})

    def save(self, proposal: dict, jpeg: bytes | None = None) -> None:
        proposal_id = identifier(proposal['id'])
        self.directory.mkdir(parents=True, exist_ok=True)
        if jpeg:
            (self.directory / f'{proposal_id}.jpg').write_bytes(jpeg)
        write_json(self.directory / f'{proposal_id}.json', proposal)
        write_json(self.pointer, {'id': proposal_id})

    def load(self, proposal_id: str) -> dict:
        return json.loads((self.directory / f'{identifier(proposal_id)}.json').read_text())

    def consumer(self, proposal_id: str) -> str:
        for path in (self.root / 'jobs').glob('*.json'):
            job = json.loads(path.read_text())
            if job.get('action') in ('approve', 'skip') and job.get('args', {}).get('proposal_id') == proposal_id:
                return job['id']
        return ''

    def current(self) -> dict | None:
        if not self.pointer.exists():
            return None
        proposal_id = json.loads(self.pointer.read_text()).get('id')
        if not proposal_id:
            return None
        value = self.load(proposal_id)
        consumer = self.consumer(proposal_id)
        if consumer:
            value.update(state='CONSUMED', consumed_by=consumer)
        elif value['state'] == 'READY' and time.time() >= value['expires_at']:
            value['state'] = 'EXPIRED'
        return value

    def claimable(self, proposal_id: str) -> dict:
        proposal = self.current()
        if proposal is None or proposal['id'] != identifier(proposal_id):
            raise ValueError('제안이 변경되었습니다. 최신 제안을 확인하세요.')
        if proposal['state'] != 'READY':
            raise ValueError('만료되었거나 이미 처리한 제안입니다. 다시 제안받으세요.')
        return proposal


def validate_fresh_target(proposal: dict, scene, config: dict, policy: dict, now: float) -> None:
    """Validate only the approved target; execution planning checks obstacles."""
    if not now < proposal['expires_at'] or now < proposal['created_at']:
        raise ValueError('승인 대기 시간이 지났습니다. 최신 장면으로 다시 제안받으세요.')
    if fingerprint(config) != proposal['config_fingerprint']:
        raise ValueError('로봇 동작 설정이 변경되었습니다. 다시 제안받으세요.')
    if (scene.tracking_session_id != proposal['tracking_session_id']
            or scene.tracking_epoch != proposal['tracking_epoch']):
        raise ValueError('물체 추적 기준이 변경되었습니다. 다시 제안받으세요.')
    target = proposal['target']
    matches = [item for item in scene.detections
               if item.track_id == target['track_id'] and item.tracking_state == 'TRACKED']
    if len(matches) != 1:
        raise ValueError('승인한 물체를 다시 확인하지 못했습니다. 다시 제안받으세요.')
    item = matches[0]
    position = (item.representative_position.x, item.representative_position.y, item.representative_position.z)
    box = (item.x_min, item.y_min, item.x_max, item.y_max)
    old = target['bbox']
    intersection = max(0., min(box[2], old[2])-max(box[0], old[0])) * max(0., min(box[3], old[3])-max(box[1], old[1]))
    union = (box[2]-box[0])*(box[3]-box[1]) + (old[2]-old[0])*(old[3]-old[1]) - intersection
    if not item.position_valid or not all(math.isfinite(v) for v in (*position, *box)):
        raise ValueError('승인 물체의 최신 위치 또는 탐지 박스가 유효하지 않습니다.')
    if scene.representative_frame != proposal['representative_frame']:
        raise ValueError('물체 위치의 좌표계가 변경되었습니다. 다시 제안받으세요.')
    displacement = math.dist(position, target['position'])
    tolerance = policy['proposal_position_tolerance_m']
    if displacement > tolerance:
        raise ValueError(f'승인 물체의 위치가 {displacement*1000:.1f}mm 변경되었습니다 '
                         f'(허용 {tolerance*1000:.1f}mm). 다시 제안받으세요.')
    if box[2] <= box[0] or box[3] <= box[1] or union <= 0:
        raise ValueError('승인 물체의 최신 탐지 박스가 유효하지 않습니다.')
    iou = intersection / union
    if iou < policy['proposal_minimum_bbox_iou']:
        raise ValueError(f'승인 물체의 박스 겹침이 {iou:.3f}로 낮아졌습니다 '
                         f"(최소 {policy['proposal_minimum_bbox_iou']:.3f}). 다시 제안받으세요.")
    if item.sorting_category != target['category'] or item.label != target['label']:
        raise ValueError('재확인 응답이 승인한 이름 또는 분류를 유지하지 않았습니다. 다시 제안받으세요.')
    # Apply the same depth, classification, size and destination policy again.
    from cleany_skill_executor.core.automatic_goal import select_automatic_target
    from types import SimpleNamespace
    selected = select_automatic_target(SimpleNamespace(snapshot_id=scene.snapshot_id, detections=[item]), config)
    if selected.skill_name != proposal['skill_name'] or selected.destination_id != proposal['destination_id']:
        raise ValueError('승인한 행동이나 목적지가 변경되었습니다.')
