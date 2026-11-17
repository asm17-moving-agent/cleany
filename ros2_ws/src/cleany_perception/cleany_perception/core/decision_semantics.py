"""Pure construction and validation of independent Decision questions."""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
import hashlib
import json
import math
from pathlib import Path
from string import Formatter
from typing import Any

import yaml

from cleany_perception.core.models import (
    ChoiceAssessment, Detection2D, FailureKind, InspectionFailure, SemanticAssessment,
)
from cleany_perception.object_labels import ObjectLabelNormalizer


@dataclass(frozen=True)
class DecisionPrompt:
    version: str
    sha256: str
    context: str
    instructions: tuple[tuple[str, str], ...]
    sorting_criteria: tuple[tuple[str, str], ...]
    risk_criteria: tuple[tuple[str, str], ...]
    unknown_description: str
    default_object_types: tuple[str, ...]


def _text(value: Any) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError('Decision prompt values must be non-empty strings')
    return value


def load_decision_prompt(path: str | Path) -> DecisionPrompt:
    raw = Path(path).read_bytes()
    data = yaml.safe_load(raw)
    if not isinstance(data, dict) or data.get('schema_version') != 1:
        raise ValueError('Unsupported Decision prompt schema')
    instructions = data['instructions']
    if not isinstance(instructions, dict) or set(instructions) != {
            'object_type', 'sorting_category', 'visual_risk'}:
        raise ValueError('Decision requires type, sorting and visual-risk instructions')
    for template in instructions.values():
        fields = [(name, spec, conversion) for _, name, spec, conversion
                  in Formatter().parse(_text(template)) if name is not None]
        if not fields or any(field != ('id', '', None) for field in fields):
            raise ValueError('Decision instructions must contain only the {id} placeholder')

    def criteria(key: str, expected: set[str]) -> tuple[tuple[str, str], ...]:
        value = data[key]
        if not isinstance(value, dict) or set(value) != expected:
            raise ValueError(f'Invalid Decision {key}')
        return tuple((k, _text(v)) for k, v in sorted(value.items()))

    return DecisionPrompt(
        version=_text(data['version']), sha256=hashlib.sha256(raw).hexdigest(),
        context=_text(data['context']), instructions=tuple(instructions.items()),
        sorting_criteria=criteria('sorting_criteria', {'trash', 'lost_item', 'review'}),
        risk_criteria=criteria('risk_criteria', {'no_visible_issue', 'visible_risk', 'uncertain'}),
        unknown_description=_text(data['unknown_description']),
        default_object_types=tuple(_text(v) for v in data['default_object_types']),
    )


def normalize_object_types(values: Sequence[str],
                           normalize: ObjectLabelNormalizer) -> tuple[str, ...]:
    if isinstance(values, str) or not all(isinstance(v, str) for v in values):
        raise ValueError('decision_object_types must be a list of names')
    normalized = tuple(normalize(v) for v in values)
    if len(set(normalized)) != len(normalized):
        raise ValueError('Duplicate Decision object types after normalization')
    ordinary = tuple(v for v in normalized if v != 'unknown object')
    if not 1 <= len(ordinary) <= 254:
        raise ValueError('Decision requires 1 to 254 ordinary object types')
    result = tuple(sorted((*ordinary, 'unknown object')))
    if any(len(v) > 128 or len(v.encode('utf-8')) > 512
           or any(ord(c) < 32 or ord(c) == 127 or (c.isspace() and c != ' ') for c in v)
           for v in result):
        raise ValueError('Decision object type violates option-key limits')
    return result


def object_types_hash(values: Sequence[str]) -> str:
    return hashlib.sha256(json.dumps(list(values), ensure_ascii=False).encode()).hexdigest()


def build_questions(count: int, prompt: DecisionPrompt,
                    object_types: Sequence[str]) -> dict[str, dict]:
    if not 0 <= count <= 333:
        raise ValueError('Decision supports at most 333 objects with three questions each')
    types = {name: (prompt.unknown_description if name == 'unknown object' else name)
             for name in object_types}
    criteria = {'object_type': types, 'sorting_category': dict(prompt.sorting_criteria),
                'visual_risk': dict(prompt.risk_criteria)}
    return {f'{kind}_{i}': {'type': 'choice', 'instructions': template.format(id=i),
                          'criteria': dict(criteria[kind])}
            for i in range(1, count + 1) for kind, template in prompt.instructions}


def decode_decision_response(raw: bytes) -> dict:
    """Reject duplicate keys before JSON object construction loses them."""
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f'Duplicate Decision response key: {key}')
            result[key] = value
        return result

    try:
        result = json.loads(raw, object_pairs_hook=unique)
        if not isinstance(result, dict):
            raise ValueError('Decision response must be an object')
        return result
    except (ValueError, UnicodeError) as error:
        raise InspectionFailure(FailureKind.DETECTOR_RESPONSE,
                                'Invalid Decision response JSON') from error


def _choice(answer: Any, options: Mapping[str, str]) -> ChoiceAssessment:
    if not isinstance(answer, Mapping) or answer.get('type') != 'choice':
        raise ValueError('Decision answer must have type choice')
    choice = answer.get('choice')
    if not isinstance(choice, str) or choice not in options:
        raise ValueError('Invalid Decision choice')

    def probability(value):
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not 0 <= value <= 1):
            raise ValueError('Invalid Decision probability or confidence')
        return float(value)

    confidence = probability(answer.get('confidence'))
    probabilities = answer.get('probabilities')
    if not isinstance(probabilities, Mapping) or set(probabilities) != set(options):
        raise ValueError('Decision probability keys do not match the choices')
    scores = tuple((key, probability(value)) for key, value in sorted(probabilities.items()))
    if not math.isclose(sum(value for _, value in scores), 1.0, abs_tol=0.001000001):
        raise ValueError('Decision choice probabilities must sum to one')
    return ChoiceAssessment(choice, confidence, scores)


def apply_decision_response(response: Mapping, questions: Mapping,
                            detections: Sequence[Detection2D], *,
                            request_id: str, model: str) -> tuple[Detection2D, ...]:
    """Apply review vetoes atomically without changing detector geometry or scores."""
    try:
        answers = response.get('answers')
        if not isinstance(answers, Mapping) or set(answers) != set(questions):
            raise ValueError('Decision response IDs do not match requested questions')
        result = []
        for i, detection in enumerate(detections, 1):
            def read(kind):
                key = f'{kind}_{i}'
                return _choice(answers[key], questions[key]['criteria'])

            kind, category, risk = (read(k) for k in (
                'object_type', 'sorting_category', 'visual_risk'))
            triggers = tuple(reason for condition, reason in (
                (category.choice == 'review', 'model_review'),
                (kind.choice == 'unknown object', 'unknown_object'),
                (risk.choice == 'visible_risk', 'visible_risk'),
            ) if condition)
            assessment = SemanticAssessment(request_id, i, model, kind, category, risk, triggers)
            result.append(replace(
                detection, label=kind.choice,
                detector_label=detection.detector_label or detection.label,
                sorting_category='review' if triggers else category.choice,
                # Legacy wire compatibility: a typed policy code, not generated reasoning.
                sorting_reason='decision:' + ('+'.join(triggers) or category.choice),
                semantic_assessment=assessment,
            ))
        return tuple(result)
    except (KeyError, TypeError, ValueError) as error:
        raise InspectionFailure(FailureKind.DETECTOR_RESPONSE,
                                f'Invalid Decision answers: {error}') from error


def semantic_object_mappings(detections: Sequence[Detection2D]) -> list[dict]:
    """Connect final distance-ranked numbers to original prompt numbers."""
    return [dict(object_id=i, request_id=d.semantic_assessment.request_id,
                 prompt_object_id=d.semantic_assessment.prompt_object_id)
            for i, d in enumerate(detections, 1) if d.semantic_assessment is not None]
