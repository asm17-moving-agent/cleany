"""Render the audited stage breakdown and independent FK benchmark together."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    read = lambda path: json.loads(path.read_text())
    records = read(args.source/'executions.json')['executions']
    payload = dict(analysis=read(args.output/'analysis.json'),
                   benchmark=read(args.output/'fk-benchmark.json'),
                   verification=read(args.output/'verification.json'),
                   accepted={r['goal']['execution_id']: r['accepted_at_ns']/1e9 for r in records})
    template = Path(__file__).with_name('execution_bottleneck_report.html').read_text()
    # Embedded data is inert JSON, including any strings originating in logs.
    encoded = json.dumps(payload, ensure_ascii=False).replace('<', '\\u003c')
    (args.output/'report.html').write_text(template.replace('__REPORT_DATA__', encoded))


if __name__ == '__main__':
    main()
