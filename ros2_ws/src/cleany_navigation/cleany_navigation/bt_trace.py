"""범위가 제한된 ROS 독립 Nav2 런타임 BT 설명과 관측입니다."""
from __future__ import annotations

from collections import deque
from copy import deepcopy
from hashlib import sha256
import time
from uuid import uuid4
from xml.etree import ElementTree as ET

MAX_NODES = 256
MAX_XML_BYTES = 131072
STATUSES = {'IDLE', 'RUNNING', 'SUCCESS', 'FAILURE'}
CONTROL = {'Sequence', 'ReactiveSequence', 'PipelineSequence', 'Fallback',
           'ReactiveFallback', 'RecoveryNode', 'RoundRobin', 'Parallel'}


def describe_tree(xml: str, source: str) -> tuple[str, dict]:
    """Nav2가 로드하기 전에 모든 인스턴스에 이름을 붙이며 포트나 구조는 바꾸지 않습니다."""
    if len(xml.encode()) > MAX_XML_BYTES:
        raise ValueError('BT XML exceeds observation limit')
    root = ET.fromstring(xml)
    trees = root.findall('BehaviorTree')
    selected = root.get('main_tree_to_execute')
    candidates = [t for t in trees if not selected or t.get('ID') == selected]
    if len(candidates) != 1 or len(candidates[0]) != 1:
        raise ValueError('BT must select exactly one root')
    if any(e.tag in ('SubTree', 'include') for e in root.iter()):
        raise ValueError('SubTree/include require expanded instance metadata')
    nodes: list[dict] = []

    def walk(element: ET.Element, parent: str | None, path: tuple[int, ...]) -> None:
        if len(nodes) >= MAX_NODES or len(path) > 32:
            raise ValueError('BT topology exceeds observation limit')
        label = element.get('name', element.tag)
        instance = 'bt_' + '_'.join(map(str, path)) + '__' + label
        element.set('name', instance)
        nodes.append(dict(id=instance, parent=parent, order=path[-1], type=element.tag,
                          label=label, kind='control' if element.tag in CONTROL else
                          'decorator' if len(element) else 'leaf',
                          ports={k: v for k, v in element.attrib.items() if k != 'name'}))
        for i, child in enumerate(element):
            walk(child, instance, (*path, i))

    walk(candidates[0][0], None, (0,))
    runtime_xml = ET.tostring(root, encoding='unicode')
    return runtime_xml, dict(schema=1, tree_id=candidates[0].get('ID'), source=source,
                             xml_sha256=sha256(runtime_xml.encode()).hexdigest(), nodes=nodes)


class TreeTrace:
    """현재 전체 snapshot과 크기 제한 이벤트 이력이며 미관측은 IDLE과 다릅니다.

    표준 Nav2 이벤트에는 goal UUID가 없습니다. 목표 연결은 명시적으로 수신 시점의
    컨텍스트일 뿐 각 이벤트가 해당 목표에 속한다는 근거는 아닙니다.
    """
    def __init__(self, manifest: dict):
        self.manifest = deepcopy(manifest)
        self.boot_id = str(uuid4())
        self.epoch = 0
        self.sequence = 0
        self.goal_id: str | None = None
        self.goal_status: int | None = None
        self.active = False
        self.states = {n['id']: 'UNKNOWN' for n in manifest['nodes']}
        self.observed = False
        self.incomplete = False
        self.reason = '아직 실행 상태를 관측하지 못했습니다.'
        self.history: deque = deque(maxlen=128)
        self.repeated: dict[tuple[str, str, str], float] = {}
        self.last_ros: float | None = None

    def reset(self, reason: str) -> None:
        self.epoch += 1
        self.states = dict.fromkeys(self.states, 'UNKNOWN')
        self.observed = False
        self.incomplete = False
        self.reason = reason
        self.history.clear()
        self.repeated.clear()

    def lifecycle(self, active: bool) -> None:
        if self.active != active:
            self.reset('실행 상태 수신 대기' if active else 'Nav2가 비활성 상태입니다.')
        self.active = active
        if not active:
            self.goal_id = None
            self.goal_status = None

    def goal(self, goal_id: str, status: int) -> None:
        if status not in (1, 2, 3, 4, 5, 6):
            return
        if status in (1, 2, 3) and goal_id != self.goal_id:
            self.reset('새 주행의 실행 상태 수신 대기')
            self.goal_id = goal_id
        if goal_id == self.goal_id:
            self.goal_status = status

    def apply(self, changes: list[dict], ros_time: float, received_at: float | None = None) -> None:
        now = time.time() if received_at is None else received_at
        if self.last_ros is not None and ros_time < self.last_ros:
            self.reset('시뮬레이션 시계가 역행했습니다.')
            self.goal_id = None
            self.goal_status = None
        self.last_ros = ros_time
        # 외부 또는 사용자 정의 트리를 설정된 구조로 간주하지 않습니다.
        if any(c['node'] not in self.states or c['status'] not in STATUSES for c in changes):
            self.reset('설정 XML과 다른 노드 또는 알 수 없는 상태를 수신했습니다.')
            self.incomplete = True
            return
        for change in changes:
            previous = self.states[change['node']]
            if previous != 'UNKNOWN' and previous != change.get('previous'):
                self.incomplete = True
                self.reason = '일부 상태 전이를 놓쳤습니다. 마지막 관측값을 표시합니다.'
            self.states[change['node']] = change['status']
            self.sequence += 1
            self.observed = True
            if not self.incomplete:
                self.reason = ''
            pair = (change['node'], change.get('previous', ''), change['status'])
            # 모든 실패와 action 전이를 보존합니다. 빠른 success/idle 조건 반복은
            # 현재 상태가 아닌 이력에서만 합칩니다.
            node = next(n for n in self.manifest['nodes'] if n['id'] == change['node'])
            important = change['status'] == 'FAILURE' or node['type'] in ('ComputePathToPose', 'FollowPath', 'YieldWait')
            if important or now-self.repeated.get(pair, -float('inf')) >= 1:
                self.history.append(dict(change, sequence=self.sequence, received_at=now,
                                         goal_context=self.goal_id, epoch=self.epoch))
                self.repeated[pair] = now

    def snapshot(self) -> dict:
        return dict(self.manifest, boot_id=self.boot_id, epoch=self.epoch,
                    sequence=self.sequence, goal_id=self.goal_id, goal_status=self.goal_status,
                    active=self.active, observed=self.observed, incomplete=self.incomplete,
                    reason=self.reason, states=dict(self.states), history=list(self.history),
                    goal_association='receipt_context_only; BehaviorTreeLog has no goal UUID')
