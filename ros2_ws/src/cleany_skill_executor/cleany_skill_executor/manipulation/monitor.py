"""Read-only mock event projection for BehaviorTree Viewer 0.1.2.

This is a visualization adapter, not a BT executor. Only FULLTREE and STATUS
requests are implemented; hooks, blackboards and robot commands are unsupported.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import struct
import uuid
import xml.etree.ElementTree as ET

from .models import Stage
from .steps import STAGE_STEPS, STEP_BY_ID


STAGE_NODES = {
    Stage.VALIDATING.value: 'ValidateGoal',
    Stage.PREPARING_TARGET.value: 'PrepareTarget',
    Stage.APPROACHING.value: 'ApproachObject',
    Stage.GRASPING.value: 'GraspObject',
    Stage.LIFTING.value: 'LiftObject',
    Stage.TRANSPORTING.value: 'CarryObject',
    Stage.PLACING.value: 'OpenGripperAtDestination',
    Stage.RETURNING_ARM.value: 'ReturnArm',
    Stage.VERIFYING_PLACEMENT.value: 'VerifyPlacedObject',
}
IDLE, RUNNING, SUCCESS, FAILURE = range(4)


@dataclass(frozen=True)
class Progress:
    execution_id: str
    accepted_at_ns: int
    revision: int
    stage: str
    last_completed_stage: str = ''
    has_result: bool = False
    status: str = ''
    failed_stage: str = ''
    record_state: str = 'ACTIVE'
    execution_profile: str = 'mock'
    stop_confirmed: bool = False
    substage: str = ''
    completed_substages: tuple[str, ...] = ()
    failed_substage: str = ''


class MonitorProjection:
    def __init__(self, xml_path: str | Path) -> None:
        root = ET.parse(xml_path).getroot()
        trees = root.findall('BehaviorTree')
        if len(trees) != 1 or trees[0].get('ID') != 'CollectTrashSkill':
            raise ValueError('Expected one CollectTrashSkill preview tree')
        self.nodes = list(trees[0].iter())[1:]
        self.by_tag = {node.tag: index for index, node in enumerate(self.nodes, 1)
                       if node.tag not in ('Sequence', 'Fallback')}
        required = {*STEP_BY_ID, 'FinalizeSuccess', 'StopAndAssess',
                    'FinalizeFailure', 'AlwaysFailure'}
        if not required.issubset(self.by_tag) or len(self.nodes) > 65535:
            raise ValueError('Preview tree does not match the mock server stages')
        outer = trees[0].find('Fallback')
        if outer is None or len(outer) != 2 or any(node.tag != 'Sequence' for node in outer):
            raise ValueError('Expected Fallback and two Sequence nodes')
        for index, node in enumerate(self.nodes, 1):
            node.set('_uid', str(index))
        self.controls = [int(node.get('_uid')) for node in (outer, *outer)]
        self.groups = [node for node in self.nodes
                       if node.tag == 'Sequence' and int(node.get('_uid')) not in self.controls]
        self.xml = ET.tostring(root, encoding='utf-8')
        self.tree_id = uuid.uuid4().bytes
        self.statuses = {index: IDLE for index in range(1, len(self.nodes) + 1)}
        self.latest: Progress | None = None

    def update(self, event: Progress) -> bool:
        if event.execution_profile != 'mock':
            return False
        if self.latest is not None:
            if event.execution_id == self.latest.execution_id:
                if event.revision <= self.latest.revision:
                    return False
            elif event.accepted_at_ns <= self.latest.accepted_at_ns:
                return False
        self.latest = event
        self.statuses = dict.fromkeys(self.statuses, IDLE)
        outer, normal, failure = self.controls
        self.statuses[outer] = RUNNING
        self.statuses[normal] = RUNNING
        stages = list(STAGE_NODES)
        detailed = bool(event.substage or event.completed_substages or event.failed_substage)
        if detailed:
            for step in event.completed_substages:
                if step in STEP_BY_ID:
                    self._set(step, SUCCESS)
        elif event.last_completed_stage in stages:
            for stage in stages[:stages.index(event.last_completed_stage) + 1]:
                for step in STAGE_STEPS[Stage(stage)]:
                    self._set(step.node, SUCCESS)
        if event.record_state in ('INTERRUPTED', 'RECORDING_FAILED'):
            # No evidence of successful stopping or finalization after a crash.
            self.statuses[outer] = self.statuses[normal] = FAILURE
            if event.stage in STAGE_NODES:
                self._set(event.substage if event.substage in STEP_BY_ID
                          else STAGE_NODES[event.stage], FAILURE)
        elif event.has_result and event.status == 'SUCCESS':
            self._set('FinalizeSuccess', SUCCESS)
            self.statuses[outer] = self.statuses[normal] = SUCCESS
        elif event.has_result:
            self.statuses[outer] = self.statuses[normal] = FAILURE
            self.statuses[failure] = FAILURE
            if event.failed_substage in STEP_BY_ID:
                self._set(event.failed_substage, FAILURE)
            elif (event.failed_stage in STAGE_NODES and not detailed and
                    not (event.status == 'CANCELED' and
                         event.failed_stage == event.last_completed_stage)):
                self._set(STAGE_NODES[event.failed_stage], FAILURE)
            if event.stop_confirmed:
                self._set('StopAndAssess', SUCCESS)
            elif event.status == 'FATAL':
                self._set('StopAndAssess', FAILURE)
            self._set('FinalizeFailure', SUCCESS)
            self._set('AlwaysFailure', FAILURE)
        elif event.stage == Stage.STOPPING.value:
            self.statuses[normal] = FAILURE
            self.statuses[failure] = RUNNING
            self._set('StopAndAssess', RUNNING)
        elif event.stage == Stage.FINALIZING.value:
            # The Result has not arrived yet; do not claim success.
            if event.last_completed_stage == Stage.VERIFYING_PLACEMENT.value:
                self._set('FinalizeSuccess', RUNNING)
            else:
                self.statuses[normal] = FAILURE
                self.statuses[failure] = RUNNING
                self._set('FinalizeFailure', RUNNING)
        elif event.stage in STAGE_NODES:
            if event.substage in STEP_BY_ID:
                if event.substage not in event.completed_substages:
                    self._set(event.substage, RUNNING)
            elif event.stage != event.last_completed_stage and not detailed:
                self._set(STAGE_NODES[event.stage], RUNNING)
        # Project child progress onto the visual groups; no BT nodes are ticked.
        for group in reversed(self.groups):
            children = [self.statuses[int(node.get('_uid'))] for node in group]
            status = (FAILURE if FAILURE in children else RUNNING if RUNNING in children
                      else SUCCESS if children and all(value == SUCCESS for value in children)
                      else IDLE)
            self.statuses[int(group.get('_uid'))] = status
        return True

    def _set(self, tag: str, status: int) -> None:
        self.statuses[self.by_tag[tag]] = status

    def reply(self, frames: list[bytes]) -> list[bytes]:
        header = frames[0] if frames else b''
        payload = b''
        if len(frames) == 1 and len(header) == 6 and header[0] == 2:
            if header[1] == ord('T'):
                payload = self.xml
            elif header[1] == ord('S'):
                payload = b''.join(struct.pack('<HB', uid, status)
                                   for uid, status in self.statuses.items())
        return [header + self.tree_id, payload]
