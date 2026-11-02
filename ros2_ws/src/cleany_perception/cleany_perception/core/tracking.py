"""Conservative fixed-base 3D association. No simulator identities or ROS types."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
import time
from typing import Callable, Sequence
from uuid import uuid4

Point = tuple[float, float, float]


class TrackingState(str, Enum):
    TRACKED = 'TRACKED'
    AMBIGUOUS = 'AMBIGUOUS'
    INVALID = 'INVALID'
    UNTRACKED = 'UNTRACKED'


@dataclass(frozen=True)
class TrackingConfig:
    maximum_distance_m: float = .03
    ambiguity_margin_m: float = .01
    session_ttl_seconds: float = 600.
    maximum_sessions: int = 32
    # Explicit aliases identify types only, never disposal permission.
    label_aliases: tuple[tuple[str, str], ...] = (
        ('paper cup', 'cup'), ('disposable paper cup', 'cup'),
        ('computer mouse', 'mouse'), ('wireless mouse', 'mouse'),
    )

    def __post_init__(self) -> None:
        if (not all(math.isfinite(x) and x > 0 for x in (
                self.maximum_distance_m, self.ambiguity_margin_m, self.session_ttl_seconds))
                or self.ambiguity_margin_m > self.maximum_distance_m
                or self.maximum_sessions < 1):
            raise ValueError('Invalid tracking configuration')
        if (any(len(pair) != 2 or not all(value.strip() for value in pair) for pair in self.label_aliases)
                or len({pair[0] for pair in self.label_aliases}) != len(self.label_aliases)):
            raise ValueError('Tracking aliases require distinct source=type pairs')


@dataclass(frozen=True)
class TrackReference:
    tracking_session_id: str
    tracking_epoch: str
    snapshot_id: str
    object_id: int
    track_id: str
    label: str


@dataclass(frozen=True)
class TrackedDetection:
    track_id: str
    state: TrackingState
    position: Point | None


@dataclass(frozen=True)
class TrackingOutput:
    session_id: str
    epoch: str
    detections: tuple[TrackedDetection, ...]
    missing: tuple[TrackReference, ...] = ()


@dataclass
class _Track:
    reference: TrackReference
    position: Point
    label: str


@dataclass
class _Session:
    epoch: str
    updated_at: float
    tracks: dict[str, _Track] = field(default_factory=dict)
    last_snapshot: str = ''
    last_output: TrackingOutput | None = None
    # Invalid depth can hide a previously untracked object. Keep that
    # uncertainty until the caller starts a new session, rather than minting IDs.
    uncertain_labels: set[str] = field(default_factory=set)
    uncertain_references: dict[str, TrackReference] = field(default_factory=dict)


class ObjectTracker:
    def __init__(self, config: TrackingConfig = TrackingConfig(),
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.config, self.clock = config, clock
        self.sessions: dict[str, _Session] = {}
        self.aliases = dict(config.label_aliases)

    def label_key(self, label: str) -> str:
        key = ' '.join(label.lower().split())
        return self.aliases.get(key, key)

    def update(self, session_id: str, snapshot_id: str,
               labels: Sequence[str], positions: Sequence[Point | None]) -> TrackingOutput:
        if not snapshot_id or len(labels) != len(positions):
            raise ValueError('Snapshot and parallel labels/positions required')
        if any(not label.strip() for label in labels):
            raise ValueError('Empty tracking label')
        for point in positions:
            if point is not None and (len(point) != 3 or not all(math.isfinite(v) for v in point)):
                raise ValueError('Tracking points must be finite 3D positions')
        if not session_id:
            return TrackingOutput('', '', tuple(TrackedDetection('', TrackingState.UNTRACKED, p)
                                                 for p in positions))
        now = self.clock()
        self.sessions = {key: value for key, value in self.sessions.items()
                         if now - value.updated_at < self.config.session_ttl_seconds}
        if session_id not in self.sessions:
            if len(self.sessions) >= self.config.maximum_sessions:
                del self.sessions[min(self.sessions, key=lambda key: self.sessions[key].updated_at)]
            self.sessions[session_id] = _Session(uuid4().hex, now)
        session = self.sessions[session_id]
        if snapshot_id == session.last_snapshot:
            assert session.last_output is not None
            return session.last_output
        keys = [self.label_key(label) for label in labels]
        tracks = list(session.tracks.values())
        distances = {(i, j): math.dist(point, track.position)
                     for i, point in enumerate(positions) if point is not None
                     for j, track in enumerate(tracks) if keys[i] == track.label}

        def unique_nearest(options: list[tuple[float, int]]) -> int | None:
            options.sort()
            if not options or options[0][0] > self.config.maximum_distance_m:
                return None
            if len(options) > 1 and options[1][0] - options[0][0] < self.config.ambiguity_margin_m:
                return None
            return options[0][1]

        forward = {i: unique_nearest([(d, j) for (a, j), d in distances.items() if a == i])
                   for i in range(len(labels))}
        reverse = {j: unique_nearest([(d, i) for (i, b), d in distances.items() if b == j])
                   for j in range(len(tracks))}
        assigned = {i: j for i, j in forward.items() if j is not None and reverse[j] == i}
        seen: set[str] = set()
        output = []
        for i, (label, point) in enumerate(zip(labels, positions)):
            track_id = ''
            if point is None:
                state = TrackingState.INVALID
                session.uncertain_labels.add(keys[i])
            elif i in assigned:
                track_id = tracks[assigned[i]].reference.track_id
                state = TrackingState.TRACKED
            else:
                # A missing or unmatched prior object may have moved here.
                # Never create a fresh ID that bypasses its failure history.
                unresolved = any(t.label == keys[i] and j not in assigned.values()
                                 for j, t in enumerate(tracks))
                crowded = any(k != i and keys[k] == keys[i] and p is not None
                              and math.dist(point, p) < self.config.ambiguity_margin_m
                              for k, p in enumerate(positions))
                invalid_peer = any(keys[k] == keys[i] and p is None
                                   for k, p in enumerate(positions))
                near_previous = any(a == i and distance <= self.config.maximum_distance_m
                                    for (a, _), distance in distances.items())
                if unresolved or crowded or invalid_peer or near_previous or keys[i] in session.uncertain_labels:
                    state = TrackingState.AMBIGUOUS
                    session.uncertain_labels.add(keys[i])
                else:
                    track_id, state = uuid4().hex, TrackingState.TRACKED
            if state in (TrackingState.INVALID, TrackingState.AMBIGUOUS) and not any(
                    track.label == keys[i] for track in session.tracks.values()):
                session.uncertain_references.setdefault(keys[i],
                    TrackReference(session_id, session.epoch, snapshot_id, i + 1, '', label))
            if track_id:
                assert point is not None
                reference = TrackReference(session_id, session.epoch, snapshot_id, i + 1, track_id, label)
                session.tracks[track_id] = _Track(reference, point, keys[i])
                seen.add(track_id)
            output.append(TrackedDetection(track_id, state, point))
        missing = tuple(t.reference for tid, t in session.tracks.items() if tid not in seen)
        missing += tuple(ref for ref in session.uncertain_references.values()
                         if ref.snapshot_id != snapshot_id)
        result = TrackingOutput(session_id, session.epoch, tuple(output), missing)
        session.updated_at, session.last_snapshot, session.last_output = now, snapshot_id, result
        return result
