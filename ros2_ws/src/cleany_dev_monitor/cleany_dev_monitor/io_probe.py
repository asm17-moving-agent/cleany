"""Read-only manipulation timing analysis and bounded evidence storage. No ROS."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import queue
import struct
import threading
import time


def load_config(path: str | Path) -> dict:
    config = json.loads(Path(path).read_text())
    for key in ('window_s', 'sample_interval_s', 'state_timeout_s', 'startup_grace_s',
                'resource_interval_s', 'motor_detail_interval_s', 'max_recording_mib'):
        value = config[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'Invalid {key}')
    if config['sample_interval_s'] < .05 or config['resource_interval_s'] < .5:
        raise ValueError('Sampling intervals would add excessive observer overhead')
    if type(config['writer_queue_size']) is not int or not 1 <= config['writer_queue_size'] <= 65536:
        raise ValueError('Invalid writer queue size')
    topics = set()
    for stream in config['streams']:
        if (stream['topic'] in topics or not stream['topic'].startswith('/')
                or stream['mode'] not in ('always', 'left', 'right')
                or stream['kind'] not in ('joint', 'motor', 'image', 'info')
                or not math.isfinite(stream['hz']) or stream['hz'] <= 0):
            raise ValueError('Invalid or duplicate stream')
        topics.add(stream['topic'])
    if not topics:
        raise ValueError('No streams configured')
    return config


def header_stamp_ns(payload: bytes) -> int | None:
    """Read only the CDR1 Header prefix; never deserialize image pixels."""
    if len(payload) < 12 or payload[:2] not in (b'\x00\x00', b'\x00\x01'):
        return None
    sec, nanos = struct.unpack_from('<iI' if payload[1] else '>iI', payload, 4)
    if sec < 0 or nanos >= 1_000_000_000 or (sec == 0 and nanos == 0):
        return None
    return sec * 1_000_000_000 + nanos


@dataclass
class Stream:
    spec: dict
    times: deque = field(default_factory=deque)
    count: int = 0
    last: float | None = None
    stamp_ns: int | None = None
    age_ms: float | None = None
    max_gap_ms: float = 0.


class ProbeState:
    def __init__(self, config: dict):
        self.config = config
        self.streams = {s['topic']: Stream(s) for s in config['streams']}
        self.wrists: dict[str, tuple[bool, float, float]] = {}
        self.execution: dict = {}
        self.execution_seen = -math.inf
        self.command: dict = {}
        self.command_seen = -math.inf
        self.execution_key = None
        self.execution_first_seen = 0.

    def observe_execution(self, record: dict, elapsed: float, start_wall_ns: int) -> dict:
        current = self.execution
        relevant = (not current or record['accepted_at_ns'] > current['accepted_at_ns'] or
                    (record['execution_id'] == current['execution_id'] and record['revision'] >= current['revision']))
        heartbeat = False
        if relevant:
            key = (record['execution_id'], record['revision'])
            if key != self.execution_key:
                self.execution_key, self.execution_first_seen = key, elapsed
            else:
                heartbeat = (elapsed-self.execution_first_seen >= .5 and
                             record.get('record_state') == 'ACTIVE' and not record.get('has_result'))
            self.execution, self.execution_seen = record, elapsed
        return dict(kind='execution', t=elapsed, relevant=relevant, heartbeat_observed=heartbeat,
                    historical=record['updated_at_ns'] < start_wall_ns and not heartbeat, record=record)

    def enabled(self, arm: str, value: bool, elapsed: float) -> None:
        old = self.wrists.get(arm)
        changed = elapsed if old is None or value != old[0] else old[2]
        self.wrists[arm] = (value, elapsed, changed)

    def mode(self, spec: dict, elapsed: float) -> tuple[str, float]:
        if spec['mode'] == 'always':
            return 'ACTIVE', 0.
        state = self.wrists.get(spec['mode'])
        if state is None or elapsed - state[1] > self.config['state_timeout_s']:
            return 'UNKNOWN', elapsed
        return ('ACTIVE' if state[0] else 'OFF'), state[2]

    def receive(self, topic: str, elapsed: float, payload_bytes: int,
                stamp_ns: int | None, ros_now_ns: int) -> dict:
        stream = self.streams[topic]
        gap = None if stream.last is None else (elapsed - stream.last) * 1000
        mode, changed = self.mode(stream.spec, elapsed)
        active_gap = gap if mode == 'ACTIVE' and stream.last is not None and stream.last >= changed else None
        if active_gap is not None:
            stream.max_gap_ms = max(stream.max_gap_ms, active_gap)
        stream.count += 1
        stream.times.append(elapsed)
        while stream.times and stream.times[0] <= elapsed - self.config['window_s']:
            stream.times.popleft()
        duplicate = stamp_ns is not None and stamp_ns == stream.stamp_ns
        stream.last, stream.stamp_ns = elapsed, stamp_ns
        stream.age_ms = None if stamp_ns is None else (ros_now_ns - stamp_ns) / 1e6
        return dict(kind='arrival', t=elapsed, topic=topic, sequence=stream.count,
                    payload_bytes=payload_bytes, gap_ms=gap, stamp_ns=stamp_ns,
                    stamp_age_ms=stream.age_ms, duplicate_stamp=duplicate,
                    active_gap_ms=active_gap, capture_state=mode)

    def stage(self, elapsed: float) -> dict:
        if self.command and (not self.execution or self.command['execution_id'] != self.execution.get('execution_id')):
            phase = self.command.get('phase', 'UNKNOWN')
            if elapsed-self.command_seen > self.config['state_timeout_s'] or (
                    self.command.get('state') == 'ACTIVE' and not self.command.get('owner_alive')):
                phase = 'STALE_REQUEST'
            return dict(execution_id=self.command['execution_id'], stage='REQUEST:'+phase, substage=phase)
        if not self.execution:
            return dict(execution_id='', stage='NO_EXECUTION', substage='')
        row = {k: self.execution.get(k, '') for k in ('execution_id', 'stage', 'substage', 'selected_arm')}
        if self.execution.get('has_result') or self.execution.get('record_state') != 'ACTIVE':
            row['stage'] = 'TERMINAL'
        elif elapsed - self.execution_seen > self.config['state_timeout_s']:
            row['stage'] = 'STALE_EXECUTION'
        return row

    def sample(self, elapsed: float, timer_lag_ms: float) -> dict:
        result = []
        for topic, stream in self.streams.items():
            while stream.times and stream.times[0] <= elapsed - self.config['window_s']:
                stream.times.popleft()
            mode, changed = self.mode(stream.spec, elapsed)
            age = None if stream.last is None else (elapsed-stream.last)*1000
            if mode == 'OFF':
                status = 'OFF'
            elif mode == 'UNKNOWN':
                status = 'UNKNOWN'
            elif elapsed - changed < self.config['startup_grace_s']:
                status = 'WARMUP'
            else:
                status = 'NO_DATA' if stream.last is None or stream.last < changed else 'OBSERVED'
            result.append(dict(topic=topic, count=len(stream.times),
                hz=len(stream.times)/self.config['window_s'] if elapsed >= self.config['window_s'] else None,
                target_hz=stream.spec['hz'], status=status, last_receipt_age_ms=age,
                stamp_age_ms=stream.age_ms,
                current_source_age_ms=(stream.age_ms+age) if stream.age_ms is not None and age is not None else None))
        return dict(kind='window', t=elapsed, timer_lag_ms=timer_lag_ms,
                    **self.stage(elapsed), streams=result)


class EvidenceWriter:
    """Bounded nonblocking callback handoff. Any loss invalidates completeness."""
    def __init__(self, path: Path, capacity: int, max_bytes: int):
        self.queue: queue.Queue = queue.Queue(capacity)
        self.file = path.open('x')
        self.max_bytes = max_bytes
        self.bytes_written = self.dropped = self.queue_peak = 0
        self.error: str | None = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def put(self, value: dict) -> None:
        if self.error:
            self.dropped += 1
            return
        try:
            self.queue.put_nowait(value)
            self.queue_peak = max(self.queue_peak, self.queue.qsize())
        except queue.Full:
            self.dropped += 1

    def _run(self) -> None:
        flushed = time.monotonic()
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    value = self.queue.get(timeout=.2)
                except queue.Empty:
                    self.file.flush()
                    continue
                line = json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')) + '\n'
                size = len(line.encode())
                if self.bytes_written + size > self.max_bytes:
                    self.dropped += 1
                    raise OSError('Recording size limit reached')
                self.file.write(line)
                self.bytes_written += size
                if time.monotonic() - flushed >= 1:
                    self.file.flush()
                    flushed = time.monotonic()
        except Exception as exc:
            self.error = str(exc)
        finally:
            self.file.close()

    def close(self) -> dict:
        self.stop.set()
        self.thread.join(timeout=10)
        if self.thread.is_alive():
            self.error = 'Writer did not drain before shutdown'
        return dict(dropped=self.dropped, queued=self.queue.qsize(), error=self.error,
                    bytes_written=self.bytes_written, queue_peak=self.queue_peak,
                    complete=not self.dropped and not self.error and self.queue.empty())
