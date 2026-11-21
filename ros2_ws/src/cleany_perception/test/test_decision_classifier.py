import base64
from dataclasses import replace
import io
import json
from pathlib import Path
import urllib.error

import numpy as np
from PIL import Image, JpegImagePlugin
import pytest
import yaml

from cleany_perception.adapters.decision_classifier import DecisionClassifier
from cleany_perception.adapters.gemini_detector import GeminiDetector, GeminiClassifier
from cleany_perception.adapters.yoloe_gemini import make_semantic_classifier, YoloeGeminiDetector
from cleany_perception.core.decision_semantics import (
    build_questions, decode_decision_response, load_decision_prompt,
    normalize_object_types, semantic_object_mappings,
)
from cleany_perception.core.models import BoundingBox2D, Detection2D, FailureKind, InspectionFailure
from cleany_perception.core.object_ranking import rank_detections_by_distance
from cleany_perception.core.target_revalidation import match_approved_target
from cleany_perception.object_labels import ObjectLabelNormalizer
from cleany_perception.snapshot_cache import CachedDetectionSnapshot, DetectionSnapshotCache
from cleany_skill_executor.core.sorting import classify_model


pytestmark = pytest.mark.usefixtures("gemini_privacy_model")


PROMPT = Path(__file__).parents[1] / 'config/decision_prompt.yaml'


def response_for(body, *, kind='cup', category='trash', risk='no_visible_issue'):
    answers = {}
    for key, question in body['questions'].items():
        selected = kind if key.startswith('object_type_') else (
            category if key.startswith('sorting_category_') else risk)
        answers[key] = dict(type='choice', choice=selected, confidence=1.,
                           probabilities={k: float(k == selected) for k in question['criteria']})
    return dict(answers=answers, usage={'total_output_tokens': 0})


@pytest.fixture
def scene(monkeypatch):
    monkeypatch.setenv('DECISION_TEST_KEY', 'private-test-key')
    rgb = np.zeros((40, 60, 3), dtype=np.uint8)
    mask = np.ones(rgb.shape[:2], dtype=np.bool_)
    detection = Detection2D('misleading-detector-label', .82, BoundingBox2D(2, 3, 20, 30),
                            segmentation_mask=mask, median_depth_m=.5,
                            estimated_width_m=.02, estimated_height_m=.03)
    return rgb, detection


def classifier(**kwargs):
    return DecisionClassifier(object_types=['cup', 'computer mouse'], prompt_file=PROMPT,
                              api_key_environment='DECISION_TEST_KEY', **kwargs)


@pytest.mark.parametrize('count', [0, 1, 5])
def test_dynamic_independent_questions_and_no_detector_hints(scene, count):
    rgb, detection = scene
    calls, audits, progress = [], [], []
    def send(body, key, timeout):
        calls.append(body)
        assert key == 'private-test-key' and 0 < timeout <= 10
        return json.dumps(response_for(body)).encode()
    adapter = classifier(transport=send, audit=audits.append, progress=progress.append)
    adapter.prepare()
    assert not calls
    original = rgb.copy()
    result = adapter.classify(rgb, [detection] * count)
    assert len(calls) == bool(count)
    assert len(result) == count and np.array_equal(rgb, original)
    if not count:
        assert not progress and not audits
        return
    body = calls[0]
    assert len(body['questions']) == 3 * count
    assert 'misleading-detector-label' not in json.dumps(body)
    assert set(body['questions']) == {
        f'{kind}_{i}' for i in range(1, count+1)
        for kind in ('object_type', 'sorting_category', 'visual_risk')}
    assert f'numbered box {count}' in body['questions'][f'object_type_{count}']['instructions']
    risk_text = json.dumps(body['questions']['visual_risk_1']).lower()
    assert not any(word in risk_text for word in ('knife', 'battery', 'glass', 'liquid', 'sharp'))
    with Image.open(io.BytesIO(base64.b64decode(body['input'][0]['data']))) as image:
        assert image.size == (60, 40) and JpegImagePlugin.get_sampling(image) == 0
        assert np.asarray(image)[15, 2, :2].min() > 150
        assert np.abs(np.asarray(image)[24, 48].astype(int) - [66, 76, 86]).max() <= 3
    for i, item in enumerate(result, 1):
        assert item.label == 'cup' and item.detector_label == detection.label
        assert item.bbox is detection.bbox and item.segmentation_mask is detection.segmentation_mask
        assert item.confidence == detection.confidence
        assert (item.median_depth_m, item.estimated_width_m, item.estimated_height_m) == (.5, .02, .03)
        assert item.semantic_assessment.prompt_object_id == i
    assert audits[-1]['event'] == 'decision_assessments'
    assert [e['state'] for e in progress] == ['started', 'succeeded']


@pytest.mark.parametrize('kind,category,risk,expected,triggers', [
    ('cup', 'trash', 'no_visible_issue', 'trash', ()),
    ('cup', 'lost_item', 'uncertain', 'lost_item', ()),
    ('cup', 'lost_item', 'visible_risk', 'review', ('visible_risk',)),
    ('cup', 'review', 'uncertain', 'review', ('model_review',)),
    ('unknown object', 'trash', 'no_visible_issue', 'review', ('unknown_object',)),
    ('unknown object', 'review', 'visible_risk', 'review',
     ('model_review', 'unknown_object', 'visible_risk')),
])
def test_review_rules_preserve_raw_answers(scene, kind, category, risk, expected, triggers):
    rgb, detection = scene
    adapter = classifier(transport=lambda body, *_: json.dumps(
        response_for(body, kind=kind, category=category, risk=risk)).encode())
    item = adapter.classify(rgb, [detection])[0]
    assert item.sorting_category == expected
    assert item.semantic_assessment.sorting_category.choice == category
    assert item.semantic_assessment.review_triggers == triggers


@pytest.mark.parametrize('bad', ['missing', 'extra', 'type', 'choice', 'confidence',
                                 'nan', 'bool', 'probabilities', 'sum', 'duplicate', 'json'])
def test_invalid_response_fails_atomically_without_retry(scene, bad):
    calls = []
    def send(body, *_):
        calls.append(body)
        payload = response_for(body)
        answer = payload['answers']['visual_risk_2']
        if bad == 'missing': del payload['answers']['visual_risk_2']
        if bad == 'extra': payload['answers']['extra'] = answer
        if bad == 'type': answer['type'] = 'score'
        if bad == 'choice': answer['choice'] = 'safe'
        if bad == 'confidence': del answer['confidence']
        if bad == 'nan': answer['confidence'] = float('nan')
        if bad == 'bool': answer['confidence'] = True
        if bad == 'probabilities': answer['probabilities']['extra'] = 0
        if bad == 'sum': answer['probabilities']['uncertain'] = .7
        if bad == 'duplicate': return b'{"answers":{},"answers":{}}'
        if bad == 'json': return b'not JSON'
        return json.dumps(payload).encode()
    with pytest.raises(InspectionFailure) as error:
        classifier(transport=send).classify(scene[0], [scene[1], scene[1]])
    assert error.value.kind == FailureKind.DETECTOR_RESPONSE and len(calls) == 1


class FakeTime:
    def __init__(self): self.now = 0.
    def clock(self): return self.now
    def sleep(self, seconds): self.now += seconds


@pytest.mark.parametrize('code,expected_calls', [(401, 1), (403, 1), (400, 1), (429, 3), (503, 3)])
def test_bounded_retry_without_leaking_credentials(scene, code, expected_calls):
    calls, events = [], []
    timer = FakeTime()
    def send(body, key, timeout):
        calls.append((body, timeout))
        raise urllib.error.HTTPError('https://example.invalid', code, key, {}, None)
    with pytest.raises(InspectionFailure) as error:
        classifier(transport=send, clock=timer.clock, sleep=timer.sleep,
                   progress=events.append).classify(scene[0], [scene[1]])
    assert len(calls) == expected_calls and error.value.kind == FailureKind.DETECTOR_API
    assert timer.now <= 30 and all(body is calls[0][0] for body, _ in calls)
    assert 'private-test-key' not in str(error.value) + json.dumps(events)


def test_transient_success_and_cancellation_during_backoff(scene):
    timer, calls = FakeTime(), []
    def send(body, *_):
        calls.append(body)
        if len(calls) == 1: raise TimeoutError()
        return json.dumps(response_for(body)).encode()
    result = classifier(transport=send, clock=timer.clock, sleep=timer.sleep).classify(
        scene[0], [scene[1]])
    assert result[0].label == 'cup' and len(calls) == 2
    calls.clear()
    with pytest.raises(InspectionFailure) as error:
        classifier(transport=send, clock=timer.clock, sleep=timer.sleep,
                   cancelled=lambda: bool(calls) and timer.now >= 1.1).classify(scene[0], [scene[1]])
    assert error.value.kind == FailureKind.CANCELLED and len(calls) == 1


def test_deadline_and_cancel_after_transport_prevent_result(scene):
    for cancel in (False, True):
        timer, calls = FakeTime(), []
        def send(body, *_):
            calls.append(body)
            timer.now = 31.
            return json.dumps(response_for(body)).encode()
        with pytest.raises(InspectionFailure) as error:
            classifier(transport=send, clock=timer.clock, sleep=timer.sleep,
                       cancelled=lambda: cancel and bool(calls)).classify(scene[0], [scene[1]])
        assert error.value.kind == (FailureKind.CANCELLED if cancel else FailureKind.DETECTOR_API)
        assert len(calls) == 1


@pytest.mark.parametrize('values', [[], ['cup', 'paper cup'], [''], ['unknown object'], ['cup\x00'],
                                   [f'type {i}' for i in range(255)]])
def test_invalid_vocabulary_is_rejected(values):
    with pytest.raises(ValueError): normalize_object_types(values, ObjectLabelNormalizer())


def test_aliases_and_prompt_contract(tmp_path):
    assert normalize_object_types(['paper cup'], ObjectLabelNormalizer()) == ('cup', 'unknown object')
    with pytest.raises(ValueError): ObjectLabelNormalizer(['a=b', 'b=a'])
    data = yaml.safe_load(PROMPT.read_text())
    data['instructions']['object_type'] = 'What is object {fixed_id}?'
    bad = tmp_path/'prompt.yaml'; bad.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError): load_decision_prompt(bad)
    with pytest.raises(ValueError): build_questions(334, load_decision_prompt(PROMPT), ['cup', 'unknown object'])


def test_default_profiles_share_decision_and_cover_existing_object_kinds():
    normalize = ObjectLabelNormalizer()
    vocabulary = set(load_decision_prompt(PROMPT).default_object_types)
    for name in ('inspect_scene', 'yoloe_real32', 'yoloe_real34', 'yoloe_real53',
                 'yoloe_real32_person_hand', 'yoloe_seg_gemini'):
        params = yaml.safe_load((PROMPT.parent / f'{name}.yaml').read_text())[
            'perception_inspector']['ros__parameters']
        assert params['gemini_model'] == 'gemini-decision-1'
        assert {normalize(value) for value in params['yoloe_classes']} <= vocabulary


def test_low_semantic_confidence_is_recorded_without_new_threshold(scene):
    def send(body, *_):
        response = response_for(body, category='lost_item', risk='uncertain')
        answer = response['answers']['sorting_category_1']
        answer.update(confidence=.34, probabilities={'lost_item': .34, 'trash': .33, 'review': .33})
        return json.dumps(response).encode()
    result = classifier(transport=send).classify(scene[0], [scene[1]])[0]
    assert result.sorting_category == 'lost_item'
    assert result.semantic_assessment.sorting_category.confidence == .34


def test_no_key_no_inference_and_explicit_legacy_selection(monkeypatch, scene):
    monkeypatch.delenv('DECISION_TEST_KEY')
    with pytest.raises(InspectionFailure): classifier().prepare()
    assert classifier().classify(scene[0], []) == ()
    options = dict(object_types=['cup'], prompt_file=PROMPT)
    assert isinstance(make_semantic_classifier(model='gemini-decision-1', **options), DecisionClassifier)
    assert isinstance(make_semantic_classifier(model='gemini-3.5-flash-lite', **options), GeminiClassifier)
    with pytest.raises(ValueError): GeminiDetector('gemini-decision-1')


def test_assessments_follow_filtering_ranking_cache_and_revalidation(scene, synthetic_scene):
    rgb, detection = scene
    adapter = classifier(transport=lambda body, *_: json.dumps(response_for(body, risk='visible_risk')).encode())
    first, second = adapter.classify(rgb, [detection, replace(detection, confidence=.9)])
    # A confidence sort before a distance sort changes order without relabeling prompt IDs.
    source = replace(synthetic_scene['detection'], semantic_assessment=second.semantic_assessment)
    other = replace(source, confidence=.5, semantic_assessment=first.semantic_assessment)
    ranked = rank_detections_by_distance(synthetic_scene['snapshot'], [other, source],
                                        synthetic_scene['transform'])
    assert [m['prompt_object_id'] for m in semantic_object_mappings([r.detection for r in ranked])] == [2, 1]
    cache = DetectionSnapshotCache()
    cached = CachedDetectionSnapshot(snapshot=synthetic_scene['snapshot'],
        detections=(first,), detection_distances_m=(None,),
        capture_transform=synthetic_scene['transform'], color_frame='camera')
    cache.put('source', cached)
    first = cache.get('source').detections[0]
    fresh = replace(detection, label='misleading-detector-label', confidence=.6, semantic_assessment=None)
    matched = match_approved_target(first, [fresh], rgb.shape[:2], .2)
    assert matched.semantic_assessment is first.semantic_assessment
    assert matched.sorting_category == 'review' and matched.confidence == .6
    decision = classify_model(matched.label, matched.confidence, matched.sorting_category, matched.sorting_reason,
        hazardous_labels=frozenset({'knife'}), minimum_confidence=.2,
        trash_destination='trash', lost_item_destination='lost')
    assert decision.category.value == 'review' and decision.destination is None


def test_composite_keeps_one_yoloe_pass_and_warmup_cleanup(scene):
    events = []
    class Local:
        def detect_scene(self, rgb, query):
            events.append('yoloe')
            return [scene[1]]
    adapter = classifier(transport=lambda body, *_: json.dumps(response_for(body)).encode())
    composite = YoloeGeminiDetector(Local(), adapter, vlm_active=lambda value: events.append(value))
    result = composite.detect_with_preprocessing(scene[0], 'private detector query',
                                                 lambda detections: events.append('geometry'))
    assert len(result) == 1 and events.count('yoloe') == 1
    assert events.count(True) == 1 and events.count(False) == 1 and 'geometry' in events


def test_legacy_sorting_guard_accepts_validated_decision_codes(scene):
    adapter = classifier(transport=lambda body, *_: json.dumps(response_for(body, category='lost_item')).encode())
    item = adapter.classify(scene[0], [scene[1]])[0]
    assert item.sorting_reason == 'decision:lost_item'
    decision = classify_model(item.label, item.confidence, item.sorting_category, item.sorting_reason,
        hazardous_labels=frozenset({'knife'}), minimum_confidence=.2,
        trash_destination='trash', lost_item_destination='lost')
    assert decision.category.value == 'lost_item'

@pytest.mark.parametrize('adapter', ['decision', 'classifier', 'detector'])
@pytest.mark.parametrize('failure', ['busy', 'detector_error'])
def test_redaction_failure_prevents_all_outbound_requests(adapter, failure, scene, monkeypatch, tmp_path):
    import fcntl
    from cleany_perception import external_image

    def unexpected(*args, **kwargs):
        pytest.fail('Transport was invoked after redaction failure')

    def unavailable():
        raise RuntimeError('model unavailable')

    lock_path = tmp_path / 'egress.lock'
    monkeypatch.setenv('CLEANY_CAMERA_PRIVACY_GPU_LOCK', str(lock_path))
    with lock_path.open('a') as lease:
        if failure == 'busy':
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        else:
            monkeypatch.setattr(external_image, '_redactor', unavailable)
        with pytest.raises(InspectionFailure, match='withheld') as exc:
            if adapter == 'decision':
                classifier(transport=unexpected).classify(scene[0], [scene[1]])
            elif adapter == 'classifier':
                GeminiClassifier('test-model', response_provider=unexpected).classify(scene[0], [scene[1]])
            else:
                GeminiDetector('test-model', response_provider=unexpected).detect(scene[0], '')
        assert exc.value.kind == FailureKind.DETECTOR_API
