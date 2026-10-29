import importlib.util
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / 'tools/validation/compare_yoloe_models.py'
SPEC = importlib.util.spec_from_file_location('compare_yoloe_models', SCRIPT)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def test_duplicate_predictions_cannot_inflate_target_recovery():
    targets = [{'labels': ['case'], 'bbox': [0., 0., 10., 10.]}]
    rows = [{'label': 'case', 'confidence': .9, 'bbox': [0., 0., 10., 10.]},
            {'label': 'box', 'confidence': .8, 'bbox': [0., 0., 10., 10.]}]
    assert len(module.match_targets(targets, rows, classified=False)) == 1
    assert len(module.match_targets(targets, rows, classified=True)) == 1
    assert module.unique_boxes(rows) == 1


def test_one_prediction_cannot_cover_two_targets_and_wrong_names_are_separate():
    targets = [{'labels': ['case'], 'bbox': [0., 0., 10., 10.]}] * 2
    rows = [{'label': 'box', 'confidence': .9, 'bbox': [0., 0., 10., 10.]}]
    assert len(module.match_targets(targets, rows, classified=False)) == 1
    assert module.match_targets(targets, rows, classified=True) == []
