import numpy as np

from cleany_perception.detector import Detection, parse_boxes


def test_parse_boxes_maps_class_ids_to_labels():
    boxes = [(10.0, 20.0, 30.0, 40.0)]
    scores = [0.9]
    class_ids = [41]
    names = {41: 'cup', 39: 'bottle'}

    dets = parse_boxes(boxes, scores, class_ids, names)

    assert dets == [Detection(label='cup', score=0.9, x1=10.0, y1=20.0, x2=30.0, y2=40.0)]


def test_parse_boxes_handles_multiple_detections():
    boxes = [(0.0, 0.0, 5.0, 5.0), (1.0, 2.0, 3.0, 4.0)]
    scores = [0.5, 0.8]
    class_ids = [39, 60]
    names = {39: 'bottle', 60: 'dining table'}

    dets = parse_boxes(boxes, scores, class_ids, names)

    assert [d.label for d in dets] == ['bottle', 'dining table']
    assert [d.score for d in dets] == [0.5, 0.8]


def test_parse_boxes_falls_back_to_stringified_id_for_unknown_class():
    dets = parse_boxes([(0.0, 0.0, 1.0, 1.0)], [0.3], [999], {41: 'cup'})

    assert dets[0].label == '999'


def test_parse_boxes_empty_input_returns_empty_list():
    assert parse_boxes([], [], [], {}) == []


def test_parse_boxes_casts_values_to_float():
    dets = parse_boxes([(1, 2, 3, 4)], [1], [41], {41: 'cup'})

    d = dets[0]
    assert isinstance(d.score, float)
    assert (d.x1, d.y1, d.x2, d.y2) == (1.0, 2.0, 3.0, 4.0)


def test_parse_boxes_without_masks_leaves_mask_none():
    dets = parse_boxes([(0.0, 0.0, 1.0, 1.0)], [0.5], [41], {41: 'cup'})

    assert dets[0].mask is None


def test_parse_boxes_attaches_boolean_masks_per_detection():
    masks = [np.array([[0.0, 1.0], [1.0, 0.0]]), np.zeros((2, 2))]
    dets = parse_boxes(
        [(0.0, 0.0, 1.0, 1.0), (0.0, 0.0, 2.0, 2.0)],
        [0.5, 0.6],
        [41, 39],
        {41: 'cup', 39: 'bottle'},
        masks,
    )

    assert dets[0].mask.dtype == bool
    assert dets[0].mask.tolist() == [[False, True], [True, False]]
    assert not dets[1].mask.any()


def test_detection_equality_ignores_mask():
    with_mask = Detection('cup', 0.9, 0.0, 0.0, 1.0, 1.0, mask=np.ones((2, 2), dtype=bool))
    without_mask = Detection('cup', 0.9, 0.0, 0.0, 1.0, 1.0)

    assert with_mask == without_mask
