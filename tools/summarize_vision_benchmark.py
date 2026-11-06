#!/usr/bin/env python3
"""Summarize OCID pipeline metrics with explicit denominators and warm timing."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from yoloe_trt import sha256
from record_jetson_telemetry import _statistics


def memory_baseline(path: Path) -> dict[str, int] | None:
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'memory':
                memory = event['meminfo']
                return {
                    'system_used_bytes': memory['MemTotal'] - memory['MemAvailable'],
                    'swap_used_bytes': memory['SwapTotal'] - memory['SwapFree'],
                }
    return None


def summarize_results(results: list[dict[str, Any]]) -> dict[str, Any]:
    warm = [item for item in results if item['run_kind'] == 'warm']
    successful = [item for item in warm if item['success']]
    object_scenes = [item for item in results if item['ground_truth_instance_count'] > 0]
    predictions = sum(item['detection_count'] for item in results)
    truth = sum(item['ground_truth_instance_count'] for item in results)
    matches = sum(item['class_agnostic_matches_iou_50'] for item in results)
    return {
        'sample_count': len(results),
        'pipeline_completed_count': sum(item['success'] for item in results),
        'object_scene_count': len(object_scenes),
        'object_scene_completed_count': sum(item['success'] for item in object_scenes),
        'empty_scene_count': len(results) - len(object_scenes),
        'empty_scene_with_detection_count': sum(
            item['ground_truth_instance_count'] == 0 and item['detection_count'] > 0
            for item in results
        ),
        'phase2_attempt_count': sum('phase2_seconds' in item for item in results),
        'phase2_success_count': sum(item.get('phase2_success', False) for item in results),
        'grasp_attempt_count': sum('anygrasp_seconds' in item for item in results),
        'grasp_success_count': sum(item.get('grasp_success', False) for item in results),
        'detection_gt_count': truth, 'detection_count': predictions,
        'class_agnostic_iou50_match_count': matches,
        'class_agnostic_iou50_precision': matches / predictions if predictions else None,
        'class_agnostic_iou50_recall': matches / truth if truth else None,
        'warm_all_seconds': {
            key: _statistics([item[key] for item in warm if key in item])
            for key in ('phase1_seconds', 'phase2_seconds', 'anygrasp_seconds', 'e2e_seconds')
        },
        'warm_success_seconds': {
            key: _statistics([item[key] for item in successful if key in item])
            for key in ('phase1_seconds', 'phase2_seconds', 'anygrasp_seconds', 'e2e_seconds')
        },
        'cold_e2e_seconds': next((item['e2e_seconds'] for item in results
                                  if item['run_kind'] == 'cold'), None),
        'failures': [
            {'sample_id': item['sample']['sample_id'],
             'gt_count': item['ground_truth_instance_count'],
             'reason': item.get('grasp_message', item.get('phase2_message', 'no detection'))}
            for item in results if not item['success']
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rebuild-dir', type=Path)
    args = parser.parse_args()
    metrics = args.run_dir / 'metrics.json'
    telemetry = args.run_dir / 'telemetry_summary.json'
    result = {
        'schema': 'cleany.vision_deployment_summary/v1',
        'metrics_sha256': sha256(metrics),
        'telemetry_summary_sha256': sha256(telemetry),
        'pipeline': summarize_results(json.loads(metrics.read_text())['results']),
        'telemetry': json.loads(telemetry.read_text()),
        'memory_baseline': memory_baseline(args.run_dir / 'telemetry.jsonl'),
    }
    if args.rebuild_dir is not None:
        manifest = args.rebuild_dir / 'rebuild_manifest.json'
        comparison_path = args.rebuild_dir / 'comparison.json'
        comparison = json.loads(comparison_path.read_text())
        result['rebuild'] = {
            'manifest': json.loads(manifest.read_text()),
            'manifest_sha256': sha256(manifest),
            'comparison_sha256': sha256(comparison_path),
            'comparison': {key: value for key, value in comparison.items() if key != 'results'},
            'failed_parity_sample_ids': [
                item['sample_id'] for item in comparison['results'] if not item['passed']
            ],
            'deployment': 'not_automatically_deployed',
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
