import json
from types import SimpleNamespace

import numpy as np
import pytest

from cleany_perception.adapters.gemini_detector import (
    GeminiDetector,
    parse_gemini_detections,
)
from cleany_perception.core.models import FailureKind, InspectionFailure


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
    assert requests[0][0].startswith(b'\x89PNG')
    assert 'find a can' in requests[0][1]
    assert requests[0][2]['required'] == ['objects']


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
            return SimpleNamespace(
                uri='https://example.invalid/file.png',
                mime_type='image/png',
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
    assert calls['upload']['config']['mime_type'] == 'image/png'
    assert calls['interaction']['model'] == (
        'gemini-robotics-er-2-preview'
    )
    assert calls['interaction']['input'][0]['type'] == 'image'
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
