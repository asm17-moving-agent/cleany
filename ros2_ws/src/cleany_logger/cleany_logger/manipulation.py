"""Bounded execution observations; no action/service clients or safety decisions."""

from collections import OrderedDict
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any


@dataclass
class ObservedExecution:
    record: dict[str, Any]
    received_at: float
    last_seen_at: float = 0.
    intervals: list[dict[str, Any]] = field(default_factory=list)
    partial: bool = False
    clock_regressed: bool = False


class ManipulationObserver:
    def __init__(self, execution_limit: int = 20, interval_limit: int = 64) -> None:
        if (type(execution_limit) is not int or not 1 <= execution_limit <= 50
                or type(interval_limit) is not int or not 1 <= interval_limit <= 128):
            raise ValueError("Manipulation observation limits must be 1..50 executions and 1..128 intervals")
        self.execution_limit = execution_limit
        self.interval_limit = interval_limit
        self.executions: OrderedDict[str, ObservedExecution] = OrderedDict()
        self.evicted = 0

    def ingest(self, record: dict[str, Any], received_at: float, *, heartbeat: bool = False) -> bool:
        execution_id = record.get("execution_id")
        revision = record.get("revision")
        updated = record.get("updated_at_ns")
        if (not isinstance(execution_id, str) or not execution_id
                or type(revision) is not int or revision < 0
                or type(updated) is not int or updated <= 0):
            raise ValueError("Invalid manipulation execution identity or timestamp")
        observed = self.executions.get(execution_id)
        if observed and revision <= observed.record["revision"]:
            if (heartbeat and record == observed.record and not record.get("has_result")
                    and record.get("record_state") == "ACTIVE"):
                observed.last_seen_at = received_at
                return True
            return False  # Retained/repeated progress cannot replace newer evidence.
        gap = revision > 0 if observed is None else revision != observed.record["revision"] + 1
        if observed is None:
            observed = ObservedExecution(deepcopy(record), received_at, partial=gap)
            self.executions[execution_id] = observed
            if len(self.executions) > self.execution_limit:
                self.executions.popitem(last=False)
                self.evicted += 1
        else:
            observed.partial |= gap
            observed.clock_regressed |= updated < observed.record["updated_at_ns"]

        stage, substage = record.get("stage", ""), record.get("substage", "")
        previous = observed.intervals[-1] if observed.intervals else None
        changed = previous is None or (previous["stage"], previous["substage"]) != (stage, substage)
        if changed:
            if previous is not None and previous["end_ns"] is None:
                previous["end_ns"] = updated if updated >= previous["start_ns"] else None
                previous["partial"] |= gap or observed.clock_regressed
            observed.intervals.append(dict(stage=stage, substage=substage, start_ns=updated,
                                           end_ns=None, partial=gap or observed.clock_regressed,
                                           received_at=received_at))
            if len(observed.intervals) > self.interval_limit:
                observed.intervals.pop(0)
                observed.partial = True
        elif gap or observed.clock_regressed:
            previous["partial"] = True
        if record.get("has_result") or record.get("record_state") != "ACTIVE":
            current = observed.intervals[-1]
            current["end_ns"] = updated if updated >= current["start_ns"] else None
        observed.record = deepcopy(record)
        observed.received_at = received_at
        observed.last_seen_at = received_at
        return True

    def snapshot(self) -> dict[str, Any]:
        runs = [dict(record=value.record, received_at=value.received_at, last_seen_at=value.last_seen_at,
                     intervals=value.intervals, partial=value.partial,
                     clock_regressed=value.clock_regressed)
                for value in self.executions.values()]
        runs.sort(key=lambda run: run["record"].get("accepted_at_ns", 0), reverse=True)
        return deepcopy(dict(executions=runs, evicted=self.evicted,
                             execution_limit=self.execution_limit, interval_limit=self.interval_limit))
