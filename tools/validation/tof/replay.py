#!/usr/bin/env python3
"""Audit recorded firmware frames without opening serial devices or driving ROS."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from cleany_tof.core import Settings, observe
from cleany_tof.transport import Stream


def audit(path: Path) -> dict:
    settings = Settings()
    states, statuses, old_coverage = Counter(), Counter(), 0
    frames = available = prefix = 0
    farthest = 0.
    stream = Stream()
    for line in path.open(encoding='utf-8'):
        record = json.loads(line)
        if record.get('type') != 'frame':
            continue
        if record.get('zones') != 64 or record.get('hz') != 15:
            if frames:
                raise ValueError('configuration changed within the recorded session')
            prefix += 1
            continue
        # Device intervals supply the offline arrival clock; this validates wire
        # content/order, not a new measurement of live USB latency.
        stream.frame(record, record['time_us']/1e6)
        o = observe([v/1000 for v in record['distance_mm']], [v/1000 for v in record['sigma_mm']],
                    record['target_status'], record['target_count'], record['valid'], settings)
        frames += 1
        available += o.available
        old_coverage += bool(o.ranges.all())
        farthest = max(farthest, max((d/1000 for d, v in zip(record['distance_mm'], record['valid']) if v), default=0.))
        states.update(o.counts())
        statuses.update(map(str, record['target_status']))
    if not frames:
        raise ValueError('no 64-zone 15Hz frames')
    return dict(source_name=path.name, source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                kind='offline classification; no new hardware or mounted coverage proof',
                frames=frames, excluded_pre_session_frames=prefix, maximum_firmware_valid_distance_m=farthest,
                zone_states=dict(states),
                target_status=dict(statuses), available_observations=available,
                old_all_zone_coverage_frames=old_coverage,
                note='NO_RETURN is unknown; no independent RGB-D coverage was recorded')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.input)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
