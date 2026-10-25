"""Sensor-backed desk work: preserve observation identity and semantic permission."""
from __future__ import annotations

import math
from typing import Any, Callable
from uuid import uuid4

from .result import FailureCode, ModuleResult, ResultStatus
from .runtime_models import SceneObject, SceneSnapshot, TaskProposal


class DeskPlanner:
    """Choose one observed task; never remove objects after an actuator result."""

    def __init__(self, destination: str):
        self.destination = destination
        self.results: dict[str, ModuleResult] = {}

    def start(self, command: object) -> str:
        scene, records = command
        skipped = {record.object_id for record in records if record.status == 'SKIPPED'}
        completed = {record.object_id for record in records if record.status == 'OK'}
        obj = next((obj for obj in scene.objects if obj.object_id not in skipped), None)
        identifier = str(uuid4())
        if obj and obj.object_id in completed:
            result = ModuleResult.blocked(FailureCode.PLACE_FAIL,
                message='Previously collected object is still observed on the desk')
        else:
            result = ModuleResult.success(TaskProposal(
                obj.disposition if obj else 'done', scene.snapshot_id,
                obj.object_id if obj else '',
                self.destination if obj and obj.disposition == 'collect_trash' else '',
                obj.wire_object_id if obj else None))
        self.results = {identifier: result}
        return identifier

    def poll(self, identifier: str) -> ModuleResult | None:
        return self.results.get(identifier)

    def cancel(self, identifier: str) -> None:
        pass

    def ready(self) -> tuple[bool, str]:
        return True, ''

    def stopped(self) -> bool:
        return True


class SensorDesk:
    """Nonblocking InspectScene adapter with late-acceptance cancellation.

    Transport uses the existing detector-only action and returns its ROS result.
    A successful empty observation may finish a desk; failed or stale observations
    never become empty scenes. Snapshot-local IDs are distinct from track IDs.
    """

    def __init__(self, transport: Any, clock: Callable[[], float],
                 ros_now_ns: Callable[[], int], *, destination: str,
                 maximum_age: float = 5., minimum_confidence: float = .25):
        self.transport, self.clock, self.ros_now_ns = transport, clock, ros_now_ns
        self.maximum_age, self.minimum_confidence = maximum_age, minimum_confidence
        self.perception = self
        self.planner = DeskPlanner(destination)
        self.session = ''
        self.operation = ''
        self.pending = self.handle = self.terminal = None
        self.result = None
        self.cancel_requested = False
        self.seen: set[str] = set()
        self.collected: set[str] = set()
        self.identities: dict[tuple[str, int], str] = {}
        self.epoch = ''

    def begin(self) -> None:
        if not self.stopped():
            raise RuntimeError('Cannot reset an active inspection')
        self.session = str(uuid4())
        self.seen.clear()
        self.collected.clear()
        self.identities.clear()
        self.epoch = ''

    def ready(self) -> tuple[bool, str]:
        return (True, '') if self.transport.ready() else (False, 'PERCEPTION_NOT_READY')

    def stopped(self) -> bool:
        self._maintain()
        return self.pending is None and self.terminal is None

    def start(self, command: object) -> str:
        if not self.stopped():
            raise RuntimeError('Inspection already active')
        self.operation = str(uuid4())
        self.result, self.cancel_requested, self.handle = None, False, None
        self.started_ns = self.ros_now_ns()
        self.pending = self.transport.send(self.session)
        return self.operation

    def cancel(self, identifier: str) -> None:
        if identifier == self.operation:
            self.cancel_requested = True
            self._maintain()

    def poll(self, identifier: str) -> ModuleResult | None:
        self._maintain()
        return self.result if identifier == self.operation else None

    def _maintain(self) -> None:
        try:
            if self.pending is not None and self.pending.done():
                future, self.pending = self.pending, None
                self.handle = future.result()
                if self.handle.accepted:
                    self.terminal = self.handle.get_result_async()
                else:
                    self.result = ModuleResult.failed(FailureCode.PERCEPTION_FAIL,
                                                       message='Inspection goal rejected')
            if self.cancel_requested and self.handle is not None and self.terminal is not None:
                self.handle.cancel_goal_async()
                self.handle = None
            if self.terminal is not None and self.terminal.done():
                future, self.terminal = self.terminal, None
                response = future.result()
                if self.cancel_requested:
                    self.result = ModuleResult(False, ResultStatus.CANCELLED)
                elif response.status != 4 or not response.result.success:
                    self.result = ModuleResult.failed(FailureCode.PERCEPTION_FAIL,
                                                       message=response.result.message)
                else:
                    self.result = ModuleResult.success(self._snapshot(response.result.detections))
        except (ValueError, AttributeError, TypeError) as error:
            self.result = ModuleResult.failed(FailureCode.PERCEPTION_FAIL, message=str(error))

    def _snapshot(self, array: Any) -> SceneSnapshot:
        stamp = array.header.stamp.sec * 10**9 + array.header.stamp.nanosec
        age = (self.ros_now_ns() - stamp) / 1e9
        if (not array.snapshot_id or array.snapshot_id in self.seen
                or not self.started_ns <= stamp <= self.ros_now_ns()
                or not 0 <= age <= self.maximum_age
                or array.tracking_session_id != self.session or not array.tracking_epoch
                or self.epoch and array.tracking_epoch != self.epoch):
            raise ValueError('Stale observation or changed tracking identity')
        self.epoch = array.tracking_epoch
        objects, ids, tracks = [], set(), set()
        for detection in array.detections:
            if not 0 < detection.object_id <= 0xffffffff or detection.object_id in ids:
                raise ValueError('Duplicate or invalid snapshot-local object ID')
            ids.add(detection.object_id)
            identity = detection.track_id
            valid_track = detection.tracking_state == 'TRACKED' and bool(identity)
            if valid_track and identity in tracks:
                raise ValueError('Duplicate tracked object')
            tracks.add(identity)
            key = f'{self.epoch}/{identity}' if valid_track else f'{array.snapshot_id}/{detection.object_id}'
            permitted = (valid_track and detection.sorting_category == 'trash'
                and bool(detection.sorting_reason.strip()) and detection.distance_valid
                and math.isfinite(detection.distance_m) and detection.distance_m > 0
                and math.isfinite(detection.confidence)
                and self.minimum_confidence <= detection.confidence <= 1.)
            objects.append(SceneObject(key, 'collect_trash' if permitted else 'human_review', detection.object_id))
            self.identities[array.snapshot_id, detection.object_id] = key
        for missing in array.missing_objects:
            key = f'{self.epoch}/{missing.track_id}'
            if key not in self.collected and key not in {obj.object_id for obj in objects}:
                objects.append(SceneObject(key, 'human_review'))
        self.seen.add(array.snapshot_id)
        return SceneSnapshot(array.snapshot_id, self.clock() - age, tuple(objects),
                             f'perception://snapshots/{array.snapshot_id}', source='isaac')

    def manipulation_succeeded(self, goal: dict) -> None:
        key = self.identities.get((goal['snapshot_id'], goal['object_id']))
        if key:
            self.collected.add(key)
