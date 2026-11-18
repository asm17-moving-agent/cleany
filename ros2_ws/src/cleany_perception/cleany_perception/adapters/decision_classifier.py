"""Bounded REST classification of numbered YOLOE instances using RGB evidence."""
from __future__ import annotations

import base64
from collections.abc import Callable, Sequence
from dataclasses import asdict
import hashlib
import io
import json
import math
import os
from pathlib import Path
import time
import urllib.error
import urllib.request
from uuid import uuid4

import numpy as np
from PIL import Image, ImageDraw

from cleany_perception.core.decision_semantics import (
    apply_decision_response, build_questions, decode_decision_response,
    load_decision_prompt, normalize_object_types, object_types_hash,
)
from cleany_perception.core.models import Detection2D, FailureKind, InspectionFailure, RgbArray
from cleany_perception.object_labels import DEFAULT_OBJECT_LABEL_ALIASES, ObjectLabelNormalizer


DECISIONS_ENDPOINT = 'https://generativelanguage.googleapis.com/v1beta/decisions'
DecisionTransport = Callable[[dict, str, float], bytes]


def default_decision_prompt_file() -> Path:
    """Resolve installed configuration, or source configuration outside ROS."""
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return Path(__file__).resolve().parents[2] / 'config' / 'decision_prompt.yaml'
    return Path(get_package_share_directory('cleany_perception')) / 'config' / 'decision_prompt.yaml'


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        # A redirect must not create hidden attempts or forward the API key.
        return None


def post_decision(body: dict, api_key: str, timeout_seconds: float) -> bytes:
    """Perform exactly one HTTP attempt; retry policy belongs to the classifier."""
    request = urllib.request.Request(
        DECISIONS_ENDPOINT, data=json.dumps(body, allow_nan=False).encode('utf-8'),
        headers={'Content-Type': 'application/json', 'x-goog-api-key': api_key}, method='POST',
    )
    with urllib.request.build_opener(_NoRedirect()).open(request, timeout=timeout_seconds) as response:
        return response.read()


class DecisionClassifier:
    """Ask independent type, sorting and visual-risk questions in one request.

    Deadline covers annotation, encoding, transport and response validation.
    Detector confidence and geometry are preserved; semantic scores are separate.
    """

    def __init__(self, *, model: str = 'gemini-decision-1',
                 object_types: Sequence[str], prompt_file: str | Path,
                 label_aliases: Sequence[str] = DEFAULT_OBJECT_LABEL_ALIASES,
                 jpeg_quality: int = 95, api_key_environment: str = 'GEMINI_API_KEY',
                 timeout_seconds: float = 30.0,
                 progress: Callable[[dict], None] | None = None,
                 audit: Callable[[dict], None] | None = None,
                 cancelled: Callable[[], bool] | None = None,
                 transport: DecisionTransport = post_decision,
                 clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], None] = time.sleep) -> None:
        self._model = model.removeprefix('models/')
        if self._model != 'gemini-decision-1':
            raise ValueError(f'Unsupported Decision model: {self._model}')
        if (not math.isfinite(timeout_seconds) or timeout_seconds <= 0
                or type(jpeg_quality) is not int or not 1 <= jpeg_quality <= 100):
            raise ValueError('Decision timeout and JPEG quality must be positive and valid')
        self._object_types = normalize_object_types(object_types, ObjectLabelNormalizer(label_aliases))
        self._prompt = load_decision_prompt(prompt_file)
        self._jpeg_quality = jpeg_quality
        self._api_key_environment = api_key_environment
        self._timeout_seconds = timeout_seconds
        self._progress = progress or (lambda event: None)
        self._audit = audit or (lambda event: None)
        self._cancelled = cancelled or (lambda: False)
        self._transport = transport
        self._clock = clock
        self._sleep = sleep

    def prepare(self) -> None:
        if not os.environ.get(self._api_key_environment, '').strip():
            raise InspectionFailure(FailureKind.DETECTOR_API,
                                    f'{self._api_key_environment} is not set')

    def _remaining(self, deadline: float) -> float:
        if self._cancelled():
            raise InspectionFailure(FailureKind.CANCELLED, 'Decision request canceled')
        remaining = deadline - self._clock()
        if remaining <= 0:
            raise InspectionFailure(FailureKind.DETECTOR_API,
                                    'Decision classification total deadline exceeded')
        return remaining

    def _wait(self, seconds: float, deadline: float) -> None:
        end = self._clock() + seconds
        while self._clock() < end:
            self._remaining(deadline)
            self._sleep(min(0.05, end - self._clock()))
        self._remaining(deadline)

    def _request(self, body: dict, request_id: str, deadline: float) -> dict:
        self.prepare()
        key = os.environ[self._api_key_environment].strip()
        began = self._clock()
        for attempt in range(1, 4):
            timeout = min(self._remaining(deadline), self._timeout_seconds / 3)
            event = dict(event='gemini_request', model=self._model, request_id=request_id,
                         attempt=attempt, max_attempts=3, timeout_sec=timeout,
                         remaining_sec=self._remaining(deadline))
            self._progress(dict(event, state='started'))
            try:
                raw = self._transport(body, key, timeout)
                self._remaining(deadline)
                response = decode_decision_response(raw)
            except InspectionFailure:
                self._progress(dict(event, state='failed'))
                raise
            except Exception as error:
                self._remaining(deadline)
                code = getattr(error, 'code', None)
                transient = code in (408, 429, 500, 502, 503, 504) or isinstance(
                    error, (TimeoutError, ConnectionError, urllib.error.URLError))
                # HTTPError subclasses URLError, but ordinary HTTP failures are permanent.
                if isinstance(error, urllib.error.HTTPError):
                    transient = code in (408, 429, 500, 502, 503, 504)
                delay = float(2 ** (attempt - 1))
                retry = transient and attempt < 3 and self._remaining(deadline) > delay
                self._progress(dict(event, state='retrying' if retry else 'failed',
                                    error_code=code, error_type=type(error).__name__,
                                    retry_in_sec=delay if retry else 0))
                if not retry:
                    # Transport messages may contain credentials; expose only type and status.
                    raise InspectionFailure(FailureKind.DETECTOR_API,
                        f'Decision API failed ({type(error).__name__}, HTTP {code})') from None
                self._wait(delay, deadline)
            else:
                self._progress(dict(event, state='succeeded', elapsed_sec=self._clock() - began))
                return response
        raise AssertionError('Unreachable Decision retry state')

    def classify(self, rgb: RgbArray,
                 detections: Sequence[Detection2D]) -> tuple[Detection2D, ...]:
        if not detections:
            return ()
        began = self._clock()
        deadline = began + self._timeout_seconds
        self._remaining(deadline)
        image = np.asarray(rgb)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise InspectionFailure(FailureKind.DETECTOR_RESPONSE,
                                    'Decision requires an HxWx3 uint8 RGB image')
        try:
            questions = build_questions(len(detections), self._prompt, self._object_types)
        except ValueError as error:
            raise InspectionFailure(FailureKind.DETECTOR_RESPONSE, str(error)) from error
        annotated = Image.fromarray(image.copy())
        draw = ImageDraw.Draw(annotated)
        for i, detection in enumerate(detections, 1):
            box = detection.bbox
            draw.rectangle((box.x_min, box.y_min, box.x_max, box.y_max),
                           outline=(255, 255, 0), width=3)
            draw.text((box.x_min, max(0, box.y_min - 12)), str(i),
                      fill=(255, 255, 0), stroke_width=2, stroke_fill=(0, 0, 0))
        buffer = io.BytesIO()
        annotated.save(buffer, format='JPEG', quality=self._jpeg_quality, subsampling=0)
        image_bytes = buffer.getvalue()
        body = dict(model=self._model, input=[
            dict(type='image', mime_type='image/jpeg',
                 data=base64.b64encode(image_bytes).decode('ascii')),
            dict(type='text', text=self._prompt.context),
        ], questions=questions)
        request_id = str(uuid4())
        metadata = dict(request_id=request_id, model=self._model,
                        image_sha256=hashlib.sha256(image_bytes).hexdigest(),
                        prompt_version=self._prompt.version, prompt_sha256=self._prompt.sha256,
                        object_types_sha256=object_types_hash(self._object_types),
                        object_count=len(detections), question_count=len(questions))
        self._audit(dict(metadata, event='decision_request'))
        try:
            self._remaining(deadline)
            response = self._request(body, request_id, deadline)
            classified = apply_decision_response(response, questions, detections,
                                                request_id=request_id, model=self._model)
            self._remaining(deadline)
        except InspectionFailure as error:
            self._audit(dict(metadata, event='decision_failure', failure_kind=error.kind.value))
            raise
        self._audit(dict(metadata, event='decision_assessments',
            elapsed_sec=self._clock() - began, usage=response.get('usage'),
            objects=[dict(assessment=asdict(d.semantic_assessment), label=d.label,
                          final_sorting_category=d.sorting_category) for d in classified]))
        return classified
