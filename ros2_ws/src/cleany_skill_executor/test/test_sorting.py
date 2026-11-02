from dataclasses import replace
from pathlib import Path

import pytest

from cleany_skill_executor.core.sorting import (
    Category, execute_sort, load_sorting_policy,
)


PROFILE = Path(__file__).parents[1] / 'config' / 'table_sorting_policy.yaml'


def test_model_category_overrides_label_allowlist_without_fallback():
    policy = load_sorting_policy(PROFILE)
    assert policy.classify_model('cup', .9, 'lost_item', 'Reusable cup').destination == 'lost_items_left'
    assert policy.classify_model('cup', .9, '', '').category == Category.REVIEW
    assert policy.classify_model('knife', .99, 'trash', 'Discarded').category == Category.REVIEW
    assert policy.classify_model('paper wrapper', .9, 'trash', 'Discarded packaging').destination == 'trash_right'


def test_table_policy_accepts_simulation_lego_threshold_after_detector_filter():
    policy = load_sorting_policy(PROFILE)
    assert policy.classify_model('lego brick', .098, 'lost_item', 'Toy').category == Category.LOST_ITEM
    assert policy.classify_model('lego brick', .079, 'lost_item', 'Toy').category == Category.REVIEW


@pytest.mark.parametrize('confidence', [0.079, -1.0, 1.1, float('nan')])
def test_uncertain_objects_are_never_discarded(confidence):
    decision = load_sorting_policy(PROFILE).classify_model('cup', confidence, 'trash', 'Disposable cup')
    assert decision.category == Category.REVIEW
    assert decision.destination is None


def test_conflicting_policy_fails_closed():
    policy = load_sorting_policy(PROFILE)
    with pytest.raises(ValueError, match='Conflicting'):
        replace(policy, trash_labels=policy.lost_item_labels)
    with pytest.raises(ValueError, match='distinct'):
        replace(policy, lost_item_destination=policy.trash_destination)


class Port:
    def __init__(self, *, failed_stage='', verified=True):
        self.calls = []
        self.failed_stage = failed_stage
        self.verified = verified

    def call(self, name, *args):
        self.calls.append((name, args))
        if name == self.failed_stage:
            raise RuntimeError(name)

    def pick(self, target):
        self.call('pick', target)
        return 'held'

    def transport(self, held, destination):
        self.call('transport', held, destination)

    def release(self, held, destination):
        self.call('release', held, destination)

    def retreat(self, held, destination):
        self.call('retreat', held, destination)

    def verify_placement(self, target, destination):
        self.call('verify', target, destination)
        return self.verified


@pytest.mark.parametrize('label,destination', [
    ('cup', 'trash_right'), ('mouse', 'lost_items_left'),
])
def test_complete_requires_correct_destination_and_verification(
    label, destination,
):
    port, stages = Port(), []
    decision = load_sorting_policy(PROFILE).classify_model(label, 0.8, 'trash' if label == 'cup' else 'lost_item', 'Model classification')
    assert execute_sort('target', decision, port, stages.append)
    assert stages == [
        'pick', 'transport', 'release', 'retreat', 'verify', 'complete',
    ]
    assert all(args[-1] == destination for _, args in port.calls[1:])


@pytest.mark.parametrize('failed_stage', ['pick', 'transport'])
def test_failure_does_not_release_item(failed_stage):
    port = Port(failed_stage=failed_stage)
    decision = load_sorting_policy(PROFILE).classify_model('cup', 0.8, 'trash', 'Disposable cup')
    with pytest.raises(RuntimeError):
        execute_sort('target', decision, port, lambda _: None)
    assert 'release' not in [name for name, _ in port.calls]


def test_release_command_is_not_placement_success():
    port, stages = Port(verified=False), []
    decision = load_sorting_policy(PROFILE).classify_model('cup', 0.8, 'trash', 'Disposable cup')
    with pytest.raises(RuntimeError, match='verified'):
        execute_sort('target', decision, port, stages.append)
    assert 'complete' not in stages


def test_operator_observation_skips_verification_without_claiming_success():
    port, stages = Port(verified=False), []
    decision = load_sorting_policy(PROFILE).classify_model('cup', 0.8, 'trash', 'Disposable cup')
    assert execute_sort('target', decision, port, stages.append, verify_placement=False)
    assert stages == ['pick', 'transport', 'release', 'retreat', 'complete_unverified']
    assert [name for name, _ in port.calls] == ['pick', 'transport', 'release', 'retreat']


@pytest.mark.parametrize('failed_stage', ['pick', 'transport', 'release', 'retreat'])
def test_operator_observation_does_not_bypass_motion_failure(failed_stage):
    port, stages = Port(failed_stage=failed_stage), []
    decision = load_sorting_policy(PROFILE).classify_model('cup', 0.8, 'trash', 'Disposable cup')
    with pytest.raises(RuntimeError):
        execute_sort('target', decision, port, stages.append, verify_placement=False)
    assert 'complete_unverified' not in stages


def test_review_does_not_command_robot():
    port, stages = Port(), []
    decision = load_sorting_policy(PROFILE).classify_model('unknown', 0.99, 'review', 'Uncertain object')
    assert not execute_sort('target', decision, port, stages.append)
    assert stages == ['review']
    assert port.calls == []
