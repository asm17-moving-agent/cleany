import io
import json
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image, JpegImagePlugin

from cleany_perception.adapters.gemini_detector import (
    GeminiClassifier,
    GeminiDetector,
    parse_gemini_classifications,
    parse_gemini_detections,
)
from cleany_perception.core.models import (
    BoundingBox2D, Detection2D, FailureKind, InspectionFailure,
)


def _jpeg_pixels(payload, size):
    with Image.open(io.BytesIO(payload)) as image:
        assert image.format == 'JPEG'
        assert image.size == size
        assert image.mode == 'RGB'
        assert JpegImagePlugin.get_sampling(image) == 0
        image.load()
        return np.asarray(image)


def test_gemini_sorting_semantics_survive_parser_and_missing_is_not_guessed():
    item = dict(label='cup', confidence=.9, box_2d=[100,100,200,200],
                sorting_category='lost_item')
    result = parse_gemini_detections(json.dumps([item]), 640, 480)[0]
    assert result.sorting_category == 'lost_item'

    item.pop('sorting_category')
    assert parse_gemini_detections(json.dumps([item]), 640, 480)[0].sorting_category == ''
    item['sorting_category'] = 'recycle'
    with pytest.raises(InspectionFailure):
        parse_gemini_detections(json.dumps([item]), 640, 480)


def test_gemini_classifies_yoloe_instances_without_changing_geometry():
    mask = np.ones((20, 30), dtype=np.bool_)
    detection = Detection2D('cup', .82, BoundingBox2D(2, 3, 12, 15),
                            segmentation_mask=mask)
    requests = []
    classifier = GeminiClassifier('test-model', response_provider=lambda image, prompt, schema: (
        requests.append((image, prompt, schema)) or json.dumps({'objects': [{
            'id': 1, 'label': 'used paper cup', 'sorting_category': 'trash',
        }]})))
    classified = classifier.classify(np.zeros((20, 30, 3), np.uint8), (detection,))
    assert len(requests) == 1
    pixels = _jpeg_pixels(requests[0][0], (30, 20))
    # The originally black input must still contain the annotated box.
    assert pixels[10, 2, :2].min() > 200
    assert requests[0][2]['required'] == ['objects']
    item_schema = requests[0][2]['properties']['objects']['items']
    assert set(item_schema['properties']) == {'id', 'label', 'sorting_category'}
    assert set(item_schema['required']) == {'id', 'label', 'sorting_category'}
    assert classified[0].label == 'cup'
    assert classified[0].detector_label == 'cup'
    assert classified[0].sorting_category == 'trash'
    assert classified[0].confidence == .82
    assert classified[0].bbox is detection.bbox
    assert classified[0].segmentation_mask is mask


def test_gemini_classification_retries_one_incomplete_mapping():
    detections = (
        Detection2D('cup', .8, BoundingBox2D(1, 1, 9, 9)),
        Detection2D('computer mouse', .7, BoundingBox2D(10, 1, 18, 9)),
    )
    prompts = []

    def provider(_image, prompt, _schema):
        prompts.append(prompt)
        objects = [{'id': 1, 'label': 'paper cup',
                    'sorting_category': 'trash'}]
        if len(prompts) == 2:
            objects.append({'id': 2, 'label': 'computer mouse',
                            'sorting_category': 'lost_item'})
        return json.dumps({'objects': objects})

    classifier = GeminiClassifier('test-model', response_provider=provider)
    result = classifier.classify(np.zeros((20, 30, 3), np.uint8), detections)
    assert len(prompts) == 2 and 'exactly 2 entries' in prompts[1]
    assert [item.label for item in result] == ['cup', 'computer mouse']
    assert result[0].bbox is detections[0].bbox


@pytest.mark.parametrize('semantic,expected', [
    ('Remote Control', 'remote control'), ('remote', 'remote control'),
    ('unknown', 'unknown object'),
])
def test_semantic_type_can_correct_detector_without_replacing_mask(semantic, expected):
    mask = np.ones((20, 30), dtype=bool)
    detection = Detection2D('power bank', .82, BoundingBox2D(2, 3, 12, 15),
                            segmentation_mask=mask)
    payload = json.dumps({'objects': [dict(id=1, label=semantic,
        sorting_category='lost_item')]})
    result, = parse_gemini_classifications(payload, (detection,))
    assert result.label == expected
    assert result.detector_label == 'power bank'
    assert result.bbox is detection.bbox and result.segmentation_mask is mask
    assert result.confidence == detection.confidence
    assert result.sorting_category == ('review' if expected == 'unknown object' else 'lost_item')


def test_classification_custom_aliases_do_not_change_detector_identity():
    detection = Detection2D('bottle', .8, BoundingBox2D(1, 1, 9, 9))
    classifier = GeminiClassifier('test-model', label_aliases=['controller=remote control'],
        response_provider=lambda *_: json.dumps({'objects': [dict(id=1, label='controller',
            sorting_category='lost_item')]}))
    result, = classifier.classify(np.zeros((20, 30, 3), np.uint8), (detection,))
    assert result.label == 'remote control' and result.detector_label == 'bottle'


def test_gemini_classification_stops_after_two_incomplete_mappings():
    calls = []
    classifier = GeminiClassifier('test-model', response_provider=lambda *_: (
        calls.append(None) or json.dumps({'objects': []})))
    with pytest.raises(InspectionFailure, match='every YOLOE instance'):
        classifier.classify(
            np.zeros((20, 30, 3), np.uint8),
            (Detection2D('cup', .8, BoundingBox2D(1, 1, 9, 9)),))
    assert len(calls) == 2


@pytest.mark.parametrize('objects', [
    [],
    [{'id': 2, 'label': 'cup', 'sorting_category': 'trash'}],
    [{'id': 1, 'label': 'cup'}],
    [{'id': 1, 'label': 'cup', 'sorting_category': ''}],
    [{'id': 1, 'label': 'cup', 'sorting_category': 'recycle'}],
])
def test_gemini_classification_rejects_unmatched_or_incomplete_ids(objects):
    detection = Detection2D('cup', .82, BoundingBox2D(2, 3, 12, 15))
    with pytest.raises(InspectionFailure) as raised:
        parse_gemini_classifications(json.dumps({'objects': objects}), (detection,))
    assert raised.value.kind == FailureKind.DETECTOR_RESPONSE


def test_flash_lite_prepare_initializes_client_without_inference(monkeypatch):
    monkeypatch.setenv('CLEANY_TEST_GEMINI_KEY', 'unit-test-placeholder')
    calls = []
    detector = GeminiDetector('gemini-3.1-flash-lite',
        api_key_environment='CLEANY_TEST_GEMINI_KEY',
        client_factory=lambda key, timeout: calls.append(timeout) or object())
    detector.prepare()
    detector.prepare()
    assert calls == [30.0]


@pytest.mark.parametrize('error_code', [503, 504])
def test_gemini_retries_transient_service_and_deadline_failures(monkeypatch, error_code):
    monkeypatch.setenv('CLEANY_TEST_GEMINI_KEY', 'unit-test-placeholder')
    delays = []
    monkeypatch.setattr('cleany_perception.adapters.gemini_detector.time.sleep', delays.append)
    detector = GeminiDetector('gemini-3.1-flash-lite',
        api_key_environment='CLEANY_TEST_GEMINI_KEY',
        client_factory=lambda key, timeout: object())
    calls = []

    def request(*args):
        calls.append(None)
        if len(calls) < 3:
            error = RuntimeError('model busy')
            error.code = error_code
            raise error
        return '{}'

    detector._request_generate_content = request
    assert detector._request(b'png', 'classify') == '{}'
    assert len(calls) == 3 and delays == [1, 2]

    calls.clear()

    def unavailable(*args):
        calls.append(None)
        error = RuntimeError('unavailable')
        error.code = 404
        raise error

    detector._request_generate_content = unavailable
    with pytest.raises(RuntimeError, match='unavailable'):
        detector._request(b'png', 'classify')
    assert len(calls) == 1


@pytest.mark.parametrize('error_code, expected_calls', [(503, 3), (504, 3), (401, 1), (403, 1)])
def test_gemini_api_retry_is_bounded_and_preserves_authentication_failures(
        monkeypatch, error_code, expected_calls):
    monkeypatch.setenv('CLEANY_TEST_GEMINI_KEY', 'unit-test-placeholder')
    monkeypatch.setattr('cleany_perception.adapters.gemini_detector.time.sleep', lambda _: None)
    detector = GeminiDetector('test-model', api_key_environment='CLEANY_TEST_GEMINI_KEY',
                              client_factory=lambda *_: object())
    calls = []
    def unavailable(*args):
        calls.append(None)
        error = RuntimeError('API failed')
        error.code = error_code
        raise error
    detector._request_generate_content = unavailable
    with pytest.raises(RuntimeError, match='API failed'):
        detector._request(b'png', 'classify')
    assert len(calls) == expected_calls


def test_gemini_prepare_fails_without_key_before_advertising_ready(monkeypatch):
    monkeypatch.delenv('CLEANY_TEST_GEMINI_KEY', raising=False)
    detector = GeminiDetector('gemini-3.1-flash-lite',
        api_key_environment='CLEANY_TEST_GEMINI_KEY')
    with pytest.raises(InspectionFailure, match='CLEANY_TEST_GEMINI_KEY is not set'):
        detector.prepare()


def test_gemini_detector_encodes_image_and_parses_normalized_bbox():
    requests = []

    def provider(image_bytes, prompt, schema):
        requests.append((image_bytes, prompt, schema))
        return json.dumps(
            {
                'objects': [
                    {
                        'label': 'can',
                        'confidence': 0.9,
                        'box_2d': [100, 200, 600, 800],
                    }
                ]
            }
        )

    detector = GeminiDetector('test-model', response_provider=provider)
    rgb = np.zeros((100, 200, 3), dtype=np.uint8)

    detections = detector.detect(rgb, 'find a can')

    assert len(detections) == 1
    assert detections[0].label == 'can'
    assert detections[0].bbox.x_min == pytest.approx(40.0)
    assert detections[0].bbox.y_min == pytest.approx(10.0)
    assert detections[0].bbox.x_max == pytest.approx(160.0)
    assert detections[0].bbox.y_max == pytest.approx(60.0)
    _jpeg_pixels(requests[0][0], (200, 100))
    assert 'find a can' in requests[0][1]
    assert requests[0][2]['required'] == ['objects']


def test_gemini_generate_content_sends_jpeg_with_matching_mime(monkeypatch):
    monkeypatch.setenv('CLEANY_TEST_GEMINI_KEY', 'test-key')
    calls = []
    client = SimpleNamespace(models=SimpleNamespace(generate_content=lambda **kwargs: (
        calls.append(kwargs) or SimpleNamespace(text='{"objects": []}'))))
    detector = GeminiDetector('test-model', api_key_environment='CLEANY_TEST_GEMINI_KEY',
                              client_factory=lambda *_: client)
    detector._types = SimpleNamespace(
        Part=SimpleNamespace(from_bytes=lambda **kwargs: SimpleNamespace(**kwargs)),
        GenerateContentConfig=lambda **kwargs: SimpleNamespace(**kwargs))
    rgb = np.full((48, 64, 3), (230, 30, 10), dtype=np.uint8)

    assert detector.detect(rgb, 'classify') == ()

    part = calls[0]['contents'][1]
    assert part.mime_type == 'image/jpeg'
    pixels = _jpeg_pixels(part.data, (64, 48))
    np.testing.assert_allclose(pixels.astype(float), rgb, atol=3)


@pytest.mark.parametrize('quality', [0, 101, 95.5, True])
def test_gemini_rejects_invalid_jpeg_quality(quality):
    with pytest.raises(ValueError, match='JPEG quality'):
        GeminiDetector('test-model', jpeg_quality=quality)


def test_gemini_parser_accepts_robotics_interaction_bbox_array():
    detections = parse_gemini_detections(
        json.dumps(
            [
                {
                    'label': 'box',
                    'confidence': 0.8,
                    'y': 100,
                    'x': 250,
                    'y2': 700,
                    'x2': 750,
                }
            ]
        ),
        width=200,
        height=100,
    )

    assert len(detections) == 1
    assert detections[0].label == 'box'
    assert detections[0].bbox.x_min == pytest.approx(50.0)
    assert detections[0].bbox.y_min == pytest.approx(10.0)
    assert detections[0].bbox.x_max == pytest.approx(150.0)
    assert detections[0].bbox.y_max == pytest.approx(70.0)


def test_robotics_detector_uses_interactions_and_deletes_upload(monkeypatch):
    monkeypatch.setenv('CLEANY_TEST_GEMINI_KEY', 'test-key')
    calls = {}

    class FakeFiles:
        def upload(self, **kwargs):
            calls['upload'] = kwargs
            calls['image_bytes'] = kwargs['file'].getvalue()
            calls['filename'] = kwargs['file'].name
            return SimpleNamespace(
                uri='https://example.invalid/file.jpg',
                mime_type=kwargs['config']['mime_type'],
                name='files/cleany-test',
            )

        def delete(self, **kwargs):
            calls['delete'] = kwargs

    class FakeInteractions:
        def create(self, **kwargs):
            calls['interaction'] = kwargs
            return SimpleNamespace(
                output_text=json.dumps(
                    [
                        {
                            'label': 'can',
                            'confidence': 0.9,
                            'y': 100,
                            'x': 200,
                            'y2': 600,
                            'x2': 800,
                        }
                    ]
                )
            )

    fake_client = SimpleNamespace(
        files=FakeFiles(),
        interactions=FakeInteractions(),
    )
    detector = GeminiDetector(
        'gemini-robotics-er-2-preview',
        api_key_environment='CLEANY_TEST_GEMINI_KEY',
        client_factory=lambda _key, _timeout: fake_client,
    )

    detections = detector.detect(
        np.zeros((100, 200, 3), dtype=np.uint8),
        'find the can',
    )

    assert len(detections) == 1
    assert calls['upload']['config']['mime_type'] == 'image/jpeg'
    assert calls['filename'].endswith('.jpg')
    _jpeg_pixels(calls['image_bytes'], (200, 100))
    assert calls['interaction']['model'] == (
        'gemini-robotics-er-2-preview'
    )
    assert calls['interaction']['input'][0]['type'] == 'image'
    assert calls['interaction']['input'][0]['mime_type'] == 'image/jpeg'
    assert calls['interaction']['response_format']['schema']['type'] == (
        'array'
    )
    assert calls['delete'] == {'name': 'files/cleany-test'}


@pytest.mark.parametrize(
    'payload',
    [
        'not-json',
        '{}',
        '{"objects": [1]}',
        (
            '{"objects": [{"label": "", "confidence": 1, '
            '"box_2d": [0, 0, 1, 1]}]}'
        ),
        (
            '{"objects": [{"label": "box", "confidence": 2, '
            '"box_2d": [0, 0, 1, 1]}]}'
        ),
        (
            '{"objects": [{"label": "box", "confidence": 1, '
            '"box_2d": [0, 0, 1001, 1]}]}'
        ),
        (
            '{"objects": [{"label": "box", "confidence": 1, '
            '"box_2d": [5, 5, 5, 6]}]}'
        ),
    ],
)
def test_gemini_parser_rejects_invalid_responses(payload):
    with pytest.raises(InspectionFailure) as raised:
        parse_gemini_detections(payload, width=100, height=100)

    assert raised.value.kind == FailureKind.DETECTOR_RESPONSE


def test_gemini_detector_maps_provider_exception_to_api_failure():
    def provider(_image, _prompt, _schema):
        raise RuntimeError('network unavailable')

    detector = GeminiDetector('test-model', response_provider=provider)

    with pytest.raises(InspectionFailure) as raised:
        detector.detect(np.zeros((10, 10, 3), dtype=np.uint8), 'find')

    assert raised.value.kind == FailureKind.DETECTOR_API


def test_gemini_detector_reports_missing_api_key(monkeypatch):
    monkeypatch.delenv('CLEANY_TEST_GEMINI_KEY', raising=False)
    detector = GeminiDetector(
        'test-model',
        api_key_environment='CLEANY_TEST_GEMINI_KEY',
    )

    with pytest.raises(InspectionFailure) as raised:
        detector.detect(np.zeros((10, 10, 3), dtype=np.uint8), 'find')

    assert raised.value.kind == FailureKind.DETECTOR_API
