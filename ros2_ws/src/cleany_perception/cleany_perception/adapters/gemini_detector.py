from __future__ import annotations

import io
import json
import math
import os
from collections.abc import Callable, Sequence
from dataclasses import replace
from typing import Any

import numpy as np
from PIL import Image as PilImage
from PIL import ImageDraw

from cleany_perception.core.models import (
    BoundingBox2D,
    Detection2D,
    FailureKind,
    InspectionFailure,
    RgbArray,
)


ResponseProvider = Callable[[bytes, str, dict[str, Any]], str]
ClientFactory = Callable[[str, float], Any]


_RESPONSE_SCHEMA: dict[str, Any] = {
    'type': 'OBJECT',
    'properties': {
        'objects': {
            'type': 'ARRAY',
            'items': {
                'type': 'OBJECT',
                'properties': {
                    'label': {'type': 'STRING'},
                    'confidence': {'type': 'NUMBER'},
                    'sorting_category': {'type': 'STRING', 'enum': ['trash', 'lost_item', 'review']},
                    'sorting_reason': {'type': 'STRING'},
                    'box_2d': {
                        'type': 'ARRAY',
                        'items': {'type': 'NUMBER'},
                        'minItems': 4,
                        'maxItems': 4,
                    },
                },
                'required': ['label', 'confidence', 'box_2d', 'sorting_category', 'sorting_reason'],
            },
        }
    },
    'required': ['objects'],
}

_INTERACTION_RESPONSE_SCHEMA: dict[str, Any] = {
    'type': 'array',
    'items': {
        'type': 'object',
        'properties': {
            'label': {'type': 'string'},
            'confidence': {'type': 'number'},
            'sorting_category': {'type': 'string', 'enum': ['trash', 'lost_item', 'review']},
            'sorting_reason': {'type': 'string'},
            'y': {'type': 'number'},
            'x': {'type': 'number'},
            'y2': {'type': 'number'},
            'x2': {'type': 'number'},
        },
        'required': ['label', 'confidence', 'y', 'x', 'y2', 'x2', 'sorting_category', 'sorting_reason'],
    },
}

_CLASSIFICATION_SCHEMA: dict[str, Any] = {
    'type': 'OBJECT',
    'properties': {'objects': {'type': 'ARRAY', 'items': {
        'type': 'OBJECT',
        'properties': {
            'id': {'type': 'INTEGER'},
            'label': {'type': 'STRING'},
            'sorting_category': {'type': 'STRING', 'enum': ['trash', 'lost_item', 'review']},
            'sorting_reason': {'type': 'STRING'},
        },
        'required': ['id', 'label', 'sorting_category', 'sorting_reason'],
    }}},
    'required': ['objects'],
}

_CLASSIFICATION_INTERACTION_SCHEMA: dict[str, Any] = {
    'type': 'object',
    'properties': {'objects': {'type': 'array', 'items': {
        'type': 'object',
        'properties': {
            'id': {'type': 'integer'},
            'label': {'type': 'string'},
            'sorting_category': {'type': 'string', 'enum': ['trash', 'lost_item', 'review']},
            'sorting_reason': {'type': 'string'},
        },
        'required': ['id', 'label', 'sorting_category', 'sorting_reason'],
    }}},
    'required': ['objects'],
}


def parse_gemini_classifications(
    response_text: str, detections: Sequence[Detection2D],
) -> tuple[Detection2D, ...]:
    """Attach semantics by exact YOLOE instance ID; never replace geometry."""
    try:
        payload = json.loads(response_text)
        items = payload['objects']
    except (TypeError, ValueError, KeyError) as error:
        raise InspectionFailure(FailureKind.DETECTOR_RESPONSE,
                                f'Gemini classification is invalid JSON: {error}') from error
    if not isinstance(items, list) or len(items) != len(detections):
        raise InspectionFailure(FailureKind.DETECTOR_RESPONSE,
                                'Gemini must classify every YOLOE instance exactly once')
    by_id = {}
    for item in items:
        if not isinstance(item, dict):
            raise InspectionFailure(FailureKind.DETECTOR_RESPONSE,
                                    'Gemini classification entry must be an object')
        identifier = item.get('id')
        label = item.get('label')
        category = item.get('sorting_category')
        reason = item.get('sorting_reason')
        if (type(identifier) is not int or identifier < 1 or identifier > len(detections)
                or identifier in by_id or not isinstance(label, str) or not label.strip()
                or category not in ('trash', 'lost_item', 'review')
                or not isinstance(reason, str) or not reason.strip()):
            raise InspectionFailure(FailureKind.DETECTOR_RESPONSE,
                                    'Gemini returned missing, duplicate, or invalid instance semantics')
        by_id[identifier] = (label.strip(), category, reason.strip())
    if len(by_id) != len(detections):
        raise InspectionFailure(FailureKind.DETECTOR_RESPONSE,
                                'Gemini classification omitted a YOLOE instance')
    return tuple(replace(detection, label=by_id[index][0],
                         sorting_category=by_id[index][1],
                         sorting_reason=by_id[index][2])
                 for index, detection in enumerate(detections, 1))


def parse_gemini_detections(
    response_text: str,
    width: int,
    height: int,
) -> tuple[Detection2D, ...]:
    try:
        payload = json.loads(response_text)
    except (TypeError, json.JSONDecodeError) as error:
        raise InspectionFailure(
            FailureKind.DETECTOR_RESPONSE,
            f'Gemini response is not valid JSON: {error}',
        ) from error
    if isinstance(payload, list):
        items = payload
    elif isinstance(payload, dict) and isinstance(
        payload.get('objects'), list
    ):
        items = payload['objects']
    else:
        raise InspectionFailure(
            FailureKind.DETECTOR_RESPONSE,
            'Gemini response must be an array or contain an objects array',
        )

    detections = []
    for index, item in enumerate(items):
        if not isinstance(item, dict):
            raise InspectionFailure(
                FailureKind.DETECTOR_RESPONSE,
                f'Gemini object {index} must be a JSON object',
            )
        label = item.get('label')
        confidence = item.get('confidence')
        box = item.get('box_2d')
        if box is None and all(
            key in item for key in ('y', 'x', 'y2', 'x2')
        ):
            box = [item['y'], item['x'], item['y2'], item['x2']]
        if not isinstance(label, str) or not label.strip():
            raise InspectionFailure(
                FailureKind.DETECTOR_RESPONSE,
                f'Gemini object {index} has an invalid label',
            )
        if isinstance(confidence, bool) or not isinstance(
            confidence, (int, float)
        ):
            raise InspectionFailure(
                FailureKind.DETECTOR_RESPONSE,
                f'Gemini object {index} has an invalid confidence',
            )
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise InspectionFailure(
                FailureKind.DETECTOR_RESPONSE,
                f'Gemini object {index} confidence is outside [0, 1]',
            )
        if not isinstance(box, list) or len(box) != 4:
            raise InspectionFailure(
                FailureKind.DETECTOR_RESPONSE,
                f'Gemini object {index} box_2d must have four values',
            )
        if any(
            isinstance(value, bool) or not isinstance(value, (int, float))
            for value in box
        ):
            raise InspectionFailure(
                FailureKind.DETECTOR_RESPONSE,
                f'Gemini object {index} box_2d must be numeric',
            )
        normalized = [float(value) for value in box]
        if not all(math.isfinite(value) for value in normalized):
            raise InspectionFailure(
                FailureKind.DETECTOR_RESPONSE,
                f'Gemini object {index} box_2d must be finite',
            )
        if not all(0.0 <= value <= 1000.0 for value in normalized):
            raise InspectionFailure(
                FailureKind.DETECTOR_RESPONSE,
                f'Gemini object {index} box_2d is outside [0, 1000]',
            )
        y_min, x_min, y_max, x_max = normalized
        try:
            bbox = BoundingBox2D(
                x_min=x_min * width / 1000.0,
                y_min=y_min * height / 1000.0,
                x_max=x_max * width / 1000.0,
                y_max=y_max * height / 1000.0,
            )
            detections.append(
                Detection2D(
                    label=label.strip(),
                    confidence=float(confidence),
                    bbox=bbox,
                    sorting_category=item.get('sorting_category', ''),
                    sorting_reason=item.get('sorting_reason', ''),
                )
            )
        except ValueError as error:
            raise InspectionFailure(
                FailureKind.DETECTOR_RESPONSE,
                f'Gemini object {index} is invalid: {error}',
            ) from error
    return tuple(detections)


class GeminiDetector:
    def __init__(
        self,
        model: str,
        api_key_environment: str = 'GEMINI_API_KEY',
        timeout_seconds: float = 30.0,
        response_provider: ResponseProvider | None = None,
        client_factory: ClientFactory | None = None,
    ) -> None:
        if not model:
            raise ValueError('Gemini model must not be empty')
        if not api_key_environment:
            raise ValueError(
                'Gemini API key environment name must not be empty'
            )
        if timeout_seconds <= 0.0:
            raise ValueError('Gemini timeout must be positive')
        self._model = model
        self._api_key_environment = api_key_environment
        self._timeout_seconds = timeout_seconds
        self._response_provider = response_provider
        self._client_factory = client_factory
        self._client = None
        self._types = None

    def prepare(self) -> None:
        """Validate local credentials/client setup, without an API inference call."""
        if self._response_provider is not None:
            return
        api_key = os.environ.get(self._api_key_environment, '').strip()
        if not api_key:
            raise InspectionFailure(
                FailureKind.DETECTOR_API, f'{self._api_key_environment} is not set')
        self._get_client(api_key)

    def detect(
        self,
        rgb: RgbArray,
        query: str,
    ) -> Sequence[Detection2D]:
        image = np.asarray(rgb)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise InspectionFailure(
                FailureKind.DETECTOR_RESPONSE,
                'Gemini detector requires an HxWx3 uint8 RGB image',
            )
        prompt = (
            f'{query.strip()}\n' if query.strip() else ''
        ) + (
            'Detect every requested visible object. Return each bounding box '
            'as [ymin, xmin, ymax, xmax] coordinates normalized to 0-1000. '
            'Return a concise label and confidence in [0, 1]. Do not return '
            'segmentation masks. Also classify each object for this supervised '
            'table-clearing simulation: sorting_category must be trash, lost_item, '
            'or review, with a short sorting_reason based on visible evidence. '
            'Discarded disposable material is trash; reusable personal belongings '
            'are lost_item. Use review for uncertainty, hazardous objects, or '
            'unclear disposability. Do not infer ownership as fact.'
        )
        buffer = io.BytesIO()
        PilImage.fromarray(image, mode='RGB').save(buffer, format='PNG')
        try:
            if self._response_provider is not None:
                response_text = self._response_provider(
                    buffer.getvalue(),
                    prompt,
                    _RESPONSE_SCHEMA,
                )
            else:
                response_text = self._request(buffer.getvalue(), prompt)
        except InspectionFailure:
            raise
        except Exception as error:
            raise InspectionFailure(
                FailureKind.DETECTOR_API,
                f'Gemini request failed: {error}',
            ) from error
        return parse_gemini_detections(
            response_text,
            width=image.shape[1],
            height=image.shape[0],
        )

    def _request(self, image_bytes: bytes, prompt: str,
                 schema: dict[str, Any] = _RESPONSE_SCHEMA,
                 interaction_schema: dict[str, Any] = _INTERACTION_RESPONSE_SCHEMA) -> str:
        api_key = os.environ.get(self._api_key_environment, '')
        if not api_key:
            raise InspectionFailure(
                FailureKind.DETECTOR_API,
                f'{self._api_key_environment} is not set',
            )
        client = self._get_client(api_key)
        if self._model.startswith('gemini-robotics-er-'):
            return self._request_interaction(client, image_bytes, prompt,
                                             interaction_schema)
        return self._request_generate_content(client, image_bytes, prompt, schema)

    def _get_client(self, api_key: str):
        if self._client is not None:
            return self._client
        if self._client_factory is not None:
            self._client = self._client_factory(
                api_key,
                self._timeout_seconds,
            )
            return self._client
        try:
            from google import genai
            from google.genai import types
        except ImportError as error:
            raise InspectionFailure(
                FailureKind.DETECTOR_API,
                'google-genai is not installed',
            ) from error
        self._types = types
        self._client = genai.Client(
            api_key=api_key,
            http_options=types.HttpOptions(
                timeout=int(self._timeout_seconds * 1000.0)
            ),
        )
        return self._client

    def _request_generate_content(
        self,
        client: Any,
        image_bytes: bytes,
        prompt: str,
        schema: dict[str, Any] = _RESPONSE_SCHEMA,
    ) -> str:
        if self._types is None:
            from google.genai import types

            self._types = types
        types = self._types
        response = client.models.generate_content(
            model=self._model,
            contents=[
                prompt,
                types.Part.from_bytes(
                    data=image_bytes,
                    mime_type='image/png',
                ),
            ],
            config=types.GenerateContentConfig(
                temperature=0.0,
                response_mime_type='application/json',
                response_schema=schema,
            ),
        )
        response_text = getattr(response, 'text', None)
        if not isinstance(response_text, str) or not response_text.strip():
            raise InspectionFailure(
                FailureKind.DETECTOR_RESPONSE,
                'Gemini returned an empty response',
            )
        return response_text

    def _request_interaction(
        self,
        client: Any,
        image_bytes: bytes,
        prompt: str,
        schema: dict[str, Any] = _INTERACTION_RESPONSE_SCHEMA,
    ) -> str:
        image_file = io.BytesIO(image_bytes)
        image_file.name = 'cleany_rgb_snapshot.png'
        uploaded_file = None
        try:
            uploaded_file = client.files.upload(
                file=image_file,
                config={
                    'mime_type': 'image/png',
                    'display_name': 'cleany RGB snapshot',
                },
            )
            interaction = client.interactions.create(
                model=self._model,
                input=[
                    {
                        'type': 'image',
                        'uri': uploaded_file.uri,
                        'mime_type': uploaded_file.mime_type,
                    },
                    {'type': 'text', 'text': prompt},
                ],
                generation_config={
                    'temperature': 0.0,
                    'thinking_level': 'medium',
                },
                response_format={
                    'type': 'text',
                    'mime_type': 'application/json',
                    'schema': schema,
                },
            )
            response_text = getattr(interaction, 'output_text', None)
            if not isinstance(response_text, str) or not response_text.strip():
                raise InspectionFailure(
                    FailureKind.DETECTOR_RESPONSE,
                    'Gemini interaction returned an empty response',
                )
            return response_text
        finally:
            uploaded_name = getattr(uploaded_file, 'name', None)
            if isinstance(uploaded_name, str) and uploaded_name:
                try:
                    client.files.delete(name=uploaded_name)
                except Exception:
                    pass


class GeminiClassifier(GeminiDetector):
    """Classify numbered YOLOE instances without re-detecting their geometry."""

    def classify(self, rgb: RgbArray, detections: Sequence[Detection2D],
                 query: str = '') -> tuple[Detection2D, ...]:
        if not detections:
            return ()
        image = np.asarray(rgb)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise InspectionFailure(FailureKind.DETECTOR_RESPONSE,
                                    'Gemini classifier requires an HxWx3 uint8 RGB image')
        annotated = PilImage.fromarray(image, mode='RGB')
        draw = ImageDraw.Draw(annotated)
        for index, detection in enumerate(detections, 1):
            box = detection.bbox
            draw.rectangle((box.x_min, box.y_min, box.x_max, box.y_max),
                           outline=(255, 255, 0), width=3)
            draw.text((box.x_min, max(0, box.y_min - 12)), str(index),
                      fill=(255, 255, 0), stroke_width=2, stroke_fill=(0, 0, 0))
        buffer = io.BytesIO()
        annotated.save(buffer, format='PNG')
        candidates = '; '.join(
            f'{index}: {detection.label}'
            for index, detection in enumerate(detections, 1))
        prompt = (
            (f'{query.strip()}\n' if query.strip() else '')
            + 'The image contains numbered YOLOE instance boxes. Classify only '
            'these numbered objects; do not detect new objects or return boxes. '
            'For each ID return a specific visible-object label. Include the YOLOE '
            'candidate class phrase in the label when visual evidence confirms it; '
            'otherwise name the visible object accurately. Return sorting_category '
            '(trash, lost_item, or review), and a brief sorting_reason grounded '
            'in visual evidence. Discarded disposable material is trash; reusable '
            'personal belongings are lost_item. Use review for uncertainty, '
            'hazardous objects, or unclear disposability. Do not infer ownership '
            f'as fact. YOLOE coarse candidates: {candidates}.'
        )
        try:
            if self._response_provider is not None:
                response_text = self._response_provider(
                    buffer.getvalue(), prompt, _CLASSIFICATION_SCHEMA)
            else:
                response_text = self._request(
                    buffer.getvalue(), prompt, _CLASSIFICATION_SCHEMA,
                    _CLASSIFICATION_INTERACTION_SCHEMA)
        except InspectionFailure:
            raise
        except Exception as error:
            raise InspectionFailure(FailureKind.DETECTOR_API,
                                    f'Gemini classification failed: {error}') from error
        return parse_gemini_classifications(response_text, detections)
