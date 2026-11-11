from pathlib import Path
import json
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from summarize_vision_benchmark import memory_baseline, summarize_results  # noqa: E402


def test_summary_separates_empty_scene_completion_from_object_success() -> None:
    common = {
        'run_kind': 'warm', 'success': True, 'detection_count': 1,
        'class_agnostic_matches_iou_50': 0, 'e2e_seconds': 1.0,
    }
    result = summarize_results([
        {**common, 'ground_truth_instance_count': 0},
        {**common, 'ground_truth_instance_count': 2,
         'class_agnostic_matches_iou_50': 1, 'e2e_seconds': 3.0},
    ])
    assert result['pipeline_completed_count'] == 2
    assert result['object_scene_completed_count'] == result['object_scene_count'] == 1
    assert result['empty_scene_with_detection_count'] == 1
    assert result['class_agnostic_iou50_precision'] == 0.5
    assert result['class_agnostic_iou50_recall'] == 0.5
    assert result['warm_success_seconds']['e2e_seconds']['mean'] == 2.0
    assert result['grasp_attempt_count'] == 0
    assert summarize_results([])['class_agnostic_iou50_precision'] is None


def test_memory_baseline_preserves_swap_already_used_before_inference(tmp_path: Path) -> None:
    path = tmp_path / 'telemetry.jsonl'
    path.write_text('\n'.join(json.dumps(event) for event in [
        {'kind': 'tegrastats'},
        {'kind': 'memory', 'meminfo': {
            'MemTotal': 16000, 'MemAvailable': 12000,
            'SwapTotal': 8000, 'SwapFree': 7000,
        }},
        {'kind': 'memory', 'meminfo': {}},
    ]))
    assert memory_baseline(path) == {'system_used_bytes': 4000, 'swap_used_bytes': 1000}
