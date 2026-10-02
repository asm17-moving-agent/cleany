"""Timed mock evidence. No robot commands or simulator ground truth are used."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from pathlib import Path
from typing import Any

import yaml

from .models import Error, Goal, ObjectState, Placement, Stage
from .ports import Evidence
from .steps import STAGE_STEPS, STEP_BY_ID


@dataclass(frozen=True)
class MockConfig:
    stage_duration_sec: float = 0.5
    stop_duration_sec: float = 0.1
    preparation_timeout_sec: float = 5.0
    verification_timeout_sec: float = 5.0
    motion_timeout_sec: float = 10.0
    stop_timeout_sec: float = 1.0
    observation_max_age_sec: float = 30.0
    selected_arm: str = 'left'
    destination_id: str = 'mock_trash_bin'
    snapshots: dict[str, Any] = field(default_factory=lambda: {
        'mock-snapshot-001': {'age_sec': 0.0, 'object_ids': [1, 2, 3]},
    })
    scenarios: dict[str, Any] = field(default_factory=lambda: {'success': {}})

    def __post_init__(self) -> None:
        for name in (
            'stage_duration_sec', 'stop_duration_sec', 'preparation_timeout_sec',
            'verification_timeout_sec', 'motion_timeout_sec', 'stop_timeout_sec',
            'observation_max_age_sec',
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not self.selected_arm.strip() or not self.destination_id.strip():
            raise ValueError('mock arm and destination IDs are required')
        for snapshot in self.snapshots.values():
            if not math.isfinite(float(snapshot['age_sec'])) or snapshot['age_sec'] < 0:
                raise ValueError('snapshot age must be finite and nonnegative')
        for scenario in self.scenarios.values():
            for step, error in scenario.get('substage_errors', {}).items():
                if step not in STEP_BY_ID:
                    raise ValueError(f'unknown substage: {step}')
                Error(error)
            for stage, error in scenario.get('stage_errors', {}).items():
                Stage(stage)
                Error(error)
            for stage, duration in scenario.get('stage_durations_sec', {}).items():
                Stage(stage)
                if not math.isfinite(duration) or duration <= 0:
                    raise ValueError('scenario durations must be finite and positive')
            if scenario.get('fault_stage'):
                Stage(scenario['fault_stage'])
                if Error(scenario.get('fault_error', 'HARDWARE_ERROR')) not in (
                    Error.HARDWARE_ERROR, Error.E_STOP,
                ):
                    raise ValueError('fault_error must be HARDWARE_ERROR or E_STOP')

    @classmethod
    def load(cls, path: str) -> MockConfig:
        with Path(path).open() as stream:
            return cls(**yaml.safe_load(stream))

    def timeout(self, stage: Stage) -> float:
        if stage in (Stage.VALIDATING, Stage.PREPARING_TARGET):
            return self.preparation_timeout_sec
        if stage == Stage.VERIFYING_PLACEMENT:
            return self.verification_timeout_sec
        if stage == Stage.STOPPING:
            return self.stop_timeout_sec
        return self.motion_timeout_sec


class MockAdapter:
    def __init__(self, config: MockConfig, scenario: str = 'success') -> None:
        self.config = config
        if scenario not in config.scenarios:
            raise ValueError(f'unknown mock scenario: {scenario}')
        self.scenario = config.scenarios[scenario]
        self.stage: Stage | None = None
        self.goal: Goal | None = None
        self.started_at = 0.0
        self.commands: list[Stage] = []
        self._step_index = 0

    def begin(self, stage: Stage, goal: Goal, now: float) -> None:
        self.stage, self.goal, self.started_at = stage, goal, now
        self.commands.append(stage)
        self._step_index = 0

    def stop(self, now: float) -> None:
        self.stage, self.started_at = Stage.STOPPING, now
        self.commands.append(Stage.STOPPING)

    def duration(self) -> float:
        default = (self.config.stop_duration_sec if self.stage == Stage.STOPPING
                   else self.config.stage_duration_sec)
        return self.scenario.get('stage_durations_sec', {}).get(self.stage.value, default)

    def fault(self, now: float) -> Error | None:
        if (self.stage is not None and self.scenario.get('fault_stage') == self.stage.value
                and now - self.started_at >= self.duration() / 2):
            return Error(self.scenario.get('fault_error', 'HARDWARE_ERROR'))
        return None

    def poll(self, now: float) -> Evidence | None:
        if self.stage is None:
            return None
        steps = STAGE_STEPS.get(self.stage, ())
        threshold = self.duration() * (self._step_index + 1) / len(steps) if steps else self.duration()
        if now - self.started_at < threshold:
            return None
        if steps:
            current = steps[self._step_index]
            error = Error(self.scenario.get('substage_errors', {}).get(current.node, 'NONE'))
            if error != Error.NONE:
                return Evidence(error=error, substage=current.node,
                                object_state=ObjectState.UNKNOWN if self.stage in (
                                    Stage.GRASPING, Stage.LIFTING, Stage.PLACING) else None,
                                message=f'Mock {current.label} failed')
            # Invalid observation must block before any reconstruction or planning.
            if self.stage == Stage.PREPARING_TARGET and self._step_index == 0:
                evidence = self._stage_evidence(now)
                if evidence.error != Error.NONE:
                    return replace(evidence, substage=current.node)
            if self._step_index < len(steps) - 1:
                self._step_index += 1
                following = steps[self._step_index]
                return Evidence(stage_complete=False, substage=following.node,
                                completed_substage=current.node,
                                message=f'Substage completed: {current.node}; starting {following.node}')
            evidence = self._stage_evidence(now)
            confirmed = not (self.stage == Stage.PLACING and
                             evidence.object_state != ObjectState.LEFT_GRIPPER)
            return replace(evidence, substage=current.node,
                           completed_substage=current.node if evidence.error == Error.NONE and confirmed else '')
        return self._stage_evidence(now)

    def _stage_evidence(self, now: float) -> Evidence:
        if self.stage == Stage.STOPPING:
            confirmed = self.scenario.get('stop_confirmed', True)
            return Evidence(error=Error.NONE if confirmed else Error.STOP_UNCONFIRMED,
                            stop_confirmed=confirmed, message='Mock stop observation')
        error = Error(self.scenario.get('stage_errors', {}).get(self.stage.value, 'NONE'))
        if self.stage == Stage.VALIDATING:
            return Evidence(error=error, stop_confirmed=error != Error.BACKEND_NOT_READY,
                            message='Mock readiness observation')
        if self.stage == Stage.PREPARING_TARGET:
            snapshot = self.config.snapshots.get(self.goal.snapshot_id)
            if error == Error.NONE:
                if not snapshot or self.goal.object_id not in snapshot['object_ids']:
                    error = Error.TARGET_UNAVAILABLE
                elif snapshot['age_sec'] + now - self.started_at > self.config.observation_max_age_sec:
                    error = Error.STALE_TARGET
                elif self.goal.destination_id != self.config.destination_id:
                    error = Error.DESTINATION_UNAVAILABLE
            return Evidence(error=error, selected_arm=self.config.selected_arm if error == Error.NONE
                            else None, message='Mock target and path preparation')
        if self.stage in (Stage.GRASPING, Stage.LIFTING):
            return Evidence(error=error, object_state=ObjectState.HELD if error == Error.NONE
                            else ObjectState.UNKNOWN, message='Mock contact observation')
        if self.stage == Stage.TRANSPORTING and error == Error.GRASP_LOST:
            return Evidence(error=error, object_state=ObjectState.UNKNOWN,
                            message='Mock held-state observation lost')
        if self.stage == Stage.PLACING:
            released = self.scenario.get('release_evidence', True) and error == Error.NONE
            return Evidence(error=error, object_state=ObjectState.LEFT_GRIPPER if released
                            else ObjectState.UNKNOWN, message='Mock post-release observation')
        if self.stage == Stage.RETURNING_ARM:
            verified = error != Error.NONE and self.scenario.get('independent_placement', False)
            return Evidence(error=error, arm_recovered=error == Error.NONE,
                            stop_confirmed=error == Error.NONE,
                            placement_state=Placement.CONFIRMED if verified else None,
                            object_state=ObjectState.LEFT_GRIPPER if verified else None,
                            message='Mock arm recovery observation')
        if self.stage == Stage.VERIFYING_PLACEMENT:
            state = (Placement.CONFIRMED if error == Error.NONE else Placement.NOT_CONFIRMED
                     if error == Error.PLACEMENT_NOT_CONFIRMED else Placement.UNKNOWN)
            return Evidence(error=error, placement_state=state,
                            object_state=ObjectState.LEFT_GRIPPER if error == Error.NONE else None,
                            message='Mock independent bin observation')
        return Evidence(error=error, message=f'Mock {self.stage.value} completion')
