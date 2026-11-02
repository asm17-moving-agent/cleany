"""Bounded JSONL decoding and device-clock lag detection."""
import json
import math


class Stream:
    def __init__(self):
        self.buffer = bytearray()
        self.sequence = -1
        self.device_us = -1
        self.offset = None

    def feed(self, data: bytes):
        self.buffer.extend(data)
        if len(self.buffer) > 131072:
            self.buffer.clear()
            raise ValueError('USB backlog exceeds limit')
        lines = self.buffer.split(b'\n')
        self.buffer = bytearray(lines.pop())
        result = []
        for raw in lines:
            if not raw.strip():
                continue
            try:
                record = json.loads(raw)
                if not isinstance(record, dict):
                    raise ValueError('record must be an object')
                result.append(record)
            except (ValueError, UnicodeError):
                # A corrupt line invalidates the batch, even if valid lines follow.
                raise ValueError('invalid JSONL') from None
        return result

    def frame(self, record, received: float, max_lag: float = .12):
        if record.get('type') != 'frame' or record.get('zones') != 64 or record.get('hz') != 15 or 'cnh' in record or 'cnh_error' in record:
            raise ValueError('expected 64-zone 15Hz distance frame')
        keys = ('distance_mm', 'sigma_mm', 'target_status', 'target_count', 'valid')
        if any(not isinstance(record.get(k), list) or len(record[k]) != 64 for k in keys):
            raise ValueError('invalid frame arrays')
        for key in keys:
            if any(type(v) not in (int, float, bool) or not math.isfinite(v) for v in record[key]):
                raise ValueError('nonfinite or nonnumeric frame data')
        seq, stamp, read = (record.get(k) for k in ('seq', 'time_us', 'read_us'))
        if any(type(v) is not int or v < 0 for v in (seq, stamp, read)) or read > 100000:
            raise ValueError('invalid frame timing')
        if seq <= self.sequence or stamp <= self.device_us:
            raise ValueError('replayed or restarted frame')
        if any(type(v) is not bool for v in record['valid']):
            raise ValueError('valid flags must be boolean')
        if any(type(v) is not int or not 0 <= v <= 255 for key in ('target_status', 'target_count') for v in record[key]):
            raise ValueError('invalid target metadata')
        if any(v != (n > 0 and st in (5, 9)) for v, n, st in zip(record['valid'], record['target_count'], record['target_status'])):
            raise ValueError('inconsistent target validity')
        offset = received-stamp/1e6
        if self.offset is None:
            self.offset = offset
        self.offset = min(self.offset, offset)
        if offset-self.offset > max_lag:
            raise ValueError('delayed USB frame')
        self.sequence, self.device_us = seq, stamp
        # time_us is read completion, not exposure: include read time and period.
        acquisition = stamp/1e6+self.offset-read/1e6-1/15
        return acquisition
