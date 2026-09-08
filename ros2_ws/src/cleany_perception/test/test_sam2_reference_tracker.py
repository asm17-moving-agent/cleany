from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from cleany_perception.adapters.sam2_reference_tracker import Sam2ReferenceTracker
from cleany_perception.core.models import BoundingBox2D, Detection2D, InspectionFailure


class Predictor:
    def __init__(self, output='valid'):
        self.output, self.calls, self.folders = output, [], []

    def init_state(self, directory, **kwargs):
        self.folders.append(directory)
        self.calls.append(('init', kwargs))
        assert [p.name for p in sorted(Path(directory).iterdir())] == ['00000.jpg', '00001.jpg']
        assert np.asarray(Image.open(Path(directory)/'00000.jpg')).mean() < 1
        assert np.asarray(Image.open(Path(directory)/'00001.jpg')).mean() > 250
        return object()

    def add_new_points_or_box(self, state, **kwargs):
        self.calls.append(('seed', kwargs))

    def propagate_in_video(self, state):
        logits = np.ones((1, 1, 20, 30), np.float32)
        yield 0, [1], logits
        if self.output == 'missing':
            return
        if self.output == 'nonfinite':
            logits[0, 0, 0, 0] = np.nan
        yield 1, ([2] if self.output == 'identity' else [1]), logits


@pytest.mark.parametrize('output', ['valid', 'missing', 'nonfinite', 'identity'])
def test_bounded_two_frame_public_api_and_cleanup(tmp_path, output):
    checkpoint = tmp_path/'model.pt'
    checkpoint.touch()
    predictor = Predictor(output)
    adapter = Sam2ReferenceTracker('config', str(checkpoint), 'cpu', lambda *args: predictor)
    detection = Detection2D('cup', .315, BoundingBox2D(3., 2., 9., 8.))
    source = np.zeros((20, 30, 3), np.uint8)
    current = np.full_like(source, 255)
    if output == 'valid':
        result = adapter.track(source, detection, current)
        assert result.shape == (20, 30) and result.dtype == np.bool_
    else:
        with pytest.raises(InspectionFailure):
            adapter.track(source, detection, current)
    assert not any(Path(folder).exists() for folder in predictor.folders)
    assert predictor.calls[1][1]['frame_idx'] == 0
    assert predictor.calls[1][1]['obj_id'] == 1
    np.testing.assert_array_equal(predictor.calls[1][1]['box'], [3., 2., 9., 8.])
    assert not source.any() and current.min() == 255


def test_sequence_uses_intermediate_frames_and_returns_only_last_mask(tmp_path):
    class SequencePredictor(Predictor):
        def init_state(self, directory, **kwargs):
            paths = sorted(Path(directory).iterdir())
            assert len(paths) == 4
            assert [int(np.asarray(Image.open(path)).mean()) for path in paths] == [0, 50, 100, 255]
            self.folders.append(directory)
            return object()

        def propagate_in_video(self, state):
            for index in range(4):
                yield index, [1], np.full((1, 1, 20, 30), 1. if index == 3 else -1., np.float32)

    checkpoint = tmp_path/'model.pt'
    checkpoint.touch()
    predictor = SequencePredictor()
    adapter = Sam2ReferenceTracker('config', str(checkpoint), 'cpu', lambda *a: predictor)
    frames = [np.full((20, 30, 3), value, np.uint8) for value in (0, 50, 100, 255)]
    detection = Detection2D('cup', .8, BoundingBox2D(3., 2., 9., 8.))
    assert adapter.track_sequence(frames, detection).all()
    assert not any(Path(folder).exists() for folder in predictor.folders)
    with pytest.raises(InspectionFailure, match='2..16'):
        adapter.track_sequence(frames*5, detection)


def test_validated_reference_mask_is_used_instead_of_reselecting_from_box(tmp_path):
    class MaskPredictor(Predictor):
        def add_new_points_or_box(self, *args, **kwargs):
            pytest.fail('A validated seed must not be replaced by a box')

        def add_new_mask(self, state, *, frame_idx, obj_id, mask):
            assert frame_idx == 0 and obj_id == 1
            np.testing.assert_array_equal(mask, seed)

    checkpoint = tmp_path/'model.pt'
    checkpoint.touch()
    adapter = Sam2ReferenceTracker('config', str(checkpoint), 'cpu', lambda *a: MaskPredictor())
    frames = [np.zeros((20, 30, 3), np.uint8), np.full((20, 30, 3), 255, np.uint8)]
    seed = np.zeros((20, 30), bool)
    seed[3:10, 4:12] = True
    detection = Detection2D('cup', .8, BoundingBox2D(3., 2., 9., 8.))
    assert adapter.track_sequence(frames, detection, reference_mask=seed).all()
    for invalid in (np.zeros_like(seed), seed.astype(np.uint8), seed[:10]):
        with pytest.raises(InspectionFailure, match='seed mask'):
            adapter.track_sequence(frames, detection, reference_mask=invalid)


def test_stream_initializes_once_encodes_only_new_frames_and_bounds_memory(tmp_path):
    torch = pytest.importorskip('torch')
    class StreamPredictor:
        max_obj_ptrs_in_encoder, num_maskmem, memory_temporal_stride_for_eval = 16, 7, 1
        image_size = 16
        def __init__(self):
            self.inits, self.seeds, self.indices = 0, 0, []
        def init_state(self, directory, **kwargs):
            self.inits += 1
            return dict(images=[torch.zeros(3, 16, 16)], num_frames=1,
                output_dict_per_obj={0: dict(cond_frame_outputs={0: 'seed'}, non_cond_frame_outputs={})},
                frames_tracked_per_obj={0: {}}, cached_features={}, temp_output_dict_per_obj={})
        def add_new_mask(self, state, **kwargs):
            self.seeds += 1
        def propagate_in_video_preflight(self, state):
            pass
        def propagate_in_video(self, state, *, start_frame_idx, max_frame_num_to_track):
            assert max_frame_num_to_track == 0
            self.indices.append(start_frame_idx)
            assert sorted(state['images']) == [0, start_frame_idx]
            state['output_dict_per_obj'][0]['non_cond_frame_outputs'][start_frame_idx] = True
            state['frames_tracked_per_obj'][0][start_frame_idx] = True
            yield start_frame_idx, [1], torch.ones(1, 1, 20, 30)
    checkpoint = tmp_path/'model.pt'
    checkpoint.touch()
    predictor = StreamPredictor()
    adapter = Sam2ReferenceTracker('config', str(checkpoint), 'cpu', lambda *a: predictor)
    rgb, mask = np.zeros((20, 30, 3), np.uint8), np.ones((20, 30), bool)
    original_threads = torch.get_num_threads()
    session = adapter.start_stream(rgb, mask, memory_frames=16, cpu_threads=2)
    assert torch.get_num_threads() == 2
    for _ in range(40):
        assert session.track(rgb).all()
    assert predictor.inits == predictor.seeds == 1
    assert predictor.indices == list(range(1, 41))
    assert len(session.state['output_dict_per_obj'][0]['non_cond_frame_outputs']) == 16
    assert len(session.state['frames_tracked_per_obj'][0]) == 16
    assert session.state['output_dict_per_obj'][0]['cond_frame_outputs'] == {0: 'seed'}
    session.close()
    assert torch.get_num_threads() == original_threads and adapter._streams == 0
    with pytest.raises(ValueError, match='memory'):
        adapter.start_stream(rgb, mask, memory_frames=1)
    assert torch.get_num_threads() == original_threads and adapter._streams == 0
    with pytest.raises(ValueError, match='Closed'):
        session.track(rgb)
