from __future__ import annotations

import io
import json
import math
import os
from collections.abc import Callable, Sequence
from typing import Any

import numpy as np
from PIL import Image as PilImage

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
                    'box_2d': {
                        'type': 'ARRAY',
                        'items': {'type': 'NUMBER'},
                        'minItems': 4,
                        'maxItems': 4,
                    },
                },
                'required': ['label', 'confidence', 'box_2d'],
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
            'y': {'type': 'number'},
            'x': {'type': 'number'},
            'y2': {'type': 'number'},
            'x2': {'type': 'number'},
        },
        'required': ['label', 'confidence', 'y', 'x', 'y2', 'x2'],
    },
}


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
            'segmentation masks.'
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

    def _request(self, image_bytes: bytes, prompt: str) -> str:
        api_key = os.environ.get(self._api_key_environment, '')
        if not api_key:
            raise InspectionFailure(
                FailureKind.DETECTOR_API,
                f'{self._api_key_environment} is not set',
            )
        client = self._get_client(api_key)
        if self._model.startswith('gemini-robotics-er-'):
            return self._request_interaction(client, image_bytes, prompt)
        return self._request_generate_content(client, image_bytes, prompt)

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
                response_schema=_RESPONSE_SCHEMA,
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
                    'schema': _INTERACTION_RESPONSE_SCHEMA,
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
