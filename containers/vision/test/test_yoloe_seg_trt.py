import json
from pathlib import Path
import sys

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'tools'))
sys.path.insert(0, str(REPO / 'ros2_ws/src/cleany_perception'))

from yoloe_trt import DEFAULT_LOCK, DEFAULT_PROFILE, export_version, load_inputs
from validate_yoloe_trt import acceptance, bbox_iou, mask_iou, matching, activate, replay_head_labels
from cleany_perception.model_contract import check_segmentation_shapes


def test_locked_profile_uses_the_exact_34_class_order():
    lock = load_inputs(DEFAULT_LOCK, DEFAULT_PROFILE)
    assert len(lock['classes']) == 34
    assert lock['classes'][:4] == ['cup', 'bottle', 'beverage can', 'drink carton']
    assert lock['classes'][-4:] == ['wallet', 'keys', 'eyeglasses', 'personal bag']
    assert lock['export']['quantize'] == 16
    assert lock['export']['nms'] is False


def test_failed_export_records_manifest_and_never_overwrites_inputs(tmp_path):
    source = tmp_path / 'source.pt'
    source.write_bytes(b'wrong hash')
    encoder = tmp_path / 'encoder.ts'
    encoder.write_bytes(b'encoder')
    with pytest.raises(ValueError, match='SHA-256'):
        export_version(source, encoder, tmp_path / 'versions')
    version = next((tmp_path / 'versions').iterdir())
    manifest = json.loads((version / 'manifest.json').read_text())
    assert manifest['status'] == 'failed'
    assert 'SHA-256' in (version / 'export.log').read_text()
    assert source.read_bytes() == b'wrong hash'
    assert not (tmp_path / 'versions/active').exists()


def test_segmentation_contract_requires_prototypes_and_coefficients():
    inputs = {'images': [1, 3, 640, 640]}
    check_segmentation_shapes(inputs, {'output0': [1, 300, 38], 'output1': [1, 32, 160, 160]}, 640, 34)
    with pytest.raises(ValueError, match='coefficients'):
        check_segmentation_shapes(inputs, {'output0': [1, 300, 6], 'output1': [1, 32, 160, 160]}, 640, 34)


def row(label='cup', box=(0., 0., 20., 20.), confidence=.8):
    return {'label': label, 'bbox': list(box), 'confidence': confidence}


def test_matching_is_one_to_one_and_diagnostic_tolerances_are_preserved():
    assert len(matching([row(), row()], [row()])) == 1
    assert matching([row()], [row('bottle')]) == []
    assert matching([row()], [row(box=(2., 0., 22., 20.))]) == [(0, 0)]
    assert matching([row()], [row(box=(2., 0., 22., 20.))], strict=True) == []
    assert matching([row()], [row(confidence=.82)], strict=True) == []
    assert bbox_iou([0., 0., 10., 10.], [10., 10., 20., 20.]) == 0.
    assert mask_iou(np.ones((2, 2), bool), np.eye(2, dtype=bool)) == .5


def test_gate_cannot_pass_empty_data_poor_masks_or_slow_engine():
    timing = {'reference': [10.] * 3, 'candidate': [8.] * 3}
    assert acceptance(100, 98, [.96] * 98, timing)['passed']
    assert not acceptance(0, 0, [], timing)['passed']
    assert not acceptance(100, 97, [1.] * 97, timing)['passed']
    assert not acceptance(100, 100, [.94] * 100, timing)['passed']
    assert not acceptance(100, 100, [1.] * 100, {'reference': [10.] * 3, 'candidate': [10.] * 3})['passed']
    assert not acceptance(100, 100, [1.] * 100, timing, strict_rebuild=False)['passed']


def test_saved_head_replay_normalizes_cup_and_preserves_mask():
    from types import SimpleNamespace
    from cleany_perception.core.models import BoundingBox2D, Detection2D
    mask = np.zeros((20, 30), dtype=bool)
    mask[2:10, 3:12] = True
    detection = Detection2D('cup', .8, BoundingBox2D(3., 2., 12., 10.), segmentation_mask=mask)
    adapter = SimpleNamespace(detect=lambda *args: [detection])
    result = replay_head_labels(adapter, [('one', np.zeros((20, 30, 3), np.uint8),
                                          {'camera': 'head', 'sample_id': 'saved-head'})])
    assert result['passed']
    assert result['outcomes'][0]['label'] == 'cup (paper cup)'
    assert result['outcomes'][0]['geometry_preserved']


def test_saved_head_replay_keeps_ambiguous_targets_rejected():
    from types import SimpleNamespace
    from cleany_perception.core.models import BoundingBox2D, Detection2D
    mask = np.ones((20, 30), dtype=bool)
    cup = Detection2D('cup', .8, BoundingBox2D(3., 2., 12., 10.), segmentation_mask=mask)
    pen = Detection2D('pen', .8, BoundingBox2D(15., 2., 25., 10.), segmentation_mask=mask)
    adapter = SimpleNamespace(detect=lambda *args: [cup, cup, pen])
    result = replay_head_labels(adapter, [('one', np.zeros((20, 30, 3), np.uint8),
                                          {'camera': 'head', 'sample_id': 'saved-head'})])
    assert result['passed']
    rejected = result['outcomes'][0]
    assert 'found 2' in rejected['canonical_rejection']
    assert rejected['normalized_rejection'] == rejected['canonical_rejection']
    assert rejected['same_association']


def test_activation_refuses_failed_comparison_and_keeps_previous_symlink(tmp_path):
    old = tmp_path / 'old'
    old.mkdir()
    new = tmp_path / 'new'
    new.mkdir()
    (new / 'manifest.json').write_text('{}')
    engine = new / 'yoloe-26s-seg-real34.engine'
    engine.write_bytes(b'engine')
    from cleany_perception.model_contract import sha256
    (tmp_path / 'active').symlink_to('old')
    with pytest.raises(ValueError, match='Failed comparison'):
        activate(new, {'passed': False, 'candidate_sha256': sha256(engine)}, old)
    assert (tmp_path / 'active').readlink() == Path('old')
