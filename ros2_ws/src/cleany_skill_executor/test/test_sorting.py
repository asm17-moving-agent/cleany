from dataclasses import replace
from pathlib import Path

import pytest

from cleany_skill_executor.core.sorting import (
    Category, execute_sort, load_sorting_policy,
)


PROFILE = Path(__file__).parents[1] / 'config' / 'sorting_policy.yaml'


def test_model_category_overrides_label_allowlist_without_fallback():
    policy = load_sorting_policy(PROFILE.parent / 'table_sorting_policy.yaml')
    assert policy.classify_model('cup', .9, 'lost_item', 'Reusable cup').destination == 'lost_items_left'
    assert policy.classify_model('cup', .9, '', '').category == Category.REVIEW
    assert policy.classify_model('knife', .99, 'trash', 'Discarded').category == Category.REVIEW
    assert policy.classify_model('paper wrapper', .9, 'trash', 'Discarded packaging').destination == 'trash_right'


@pytest.mark.parametrize('label,category,destination', [
    ('cup', Category.TRASH, 'trash_left'),
    ('  Paper   Cup ', Category.TRASH, 'trash_left'),
    ('wallet', Category.LOST_ITEM, 'lost_items_right'),
    ('phone', Category.LOST_ITEM, 'lost_items_right'),
    ('eraser', Category.LOST_ITEM, 'lost_items_right'),
    ('lego brick', Category.LOST_ITEM, 'lost_items_right'),
    ('crumpled tissue', Category.TRASH, 'trash_left'),
    ('mug', Category.REVIEW, None),
    ('unknown', Category.REVIEW, None),
    ('knife', Category.REVIEW, None),
])
def test_rule_mapping(label, category, destination):
    decision = load_sorting_policy(PROFILE).classify(label, 0.9)
    assert decision.category == category
    assert decision.destination == destination


@pytest.mark.parametrize('confidence', [0.24, -1.0, 1.1, float('nan')])
def test_uncertain_objects_are_never_discarded(confidence):
    decision = load_sorting_policy(PROFILE).classify('cup', confidence)
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
    ('cup', 'trash_left'), ('wallet', 'lost_items_right'),
])
def test_complete_requires_correct_destination_and_verification(
    label, destination,
):
    port, stages = Port(), []
    decision = load_sorting_policy(PROFILE).classify(label, 0.8)
    assert execute_sort('target', decision, port, stages.append)
    assert stages == [
        'pick', 'transport', 'release', 'retreat', 'verify', 'complete',
    ]
    assert all(args[-1] == destination for _, args in port.calls[1:])


@pytest.mark.parametrize('failed_stage', ['pick', 'transport'])
def test_failure_does_not_release_item(failed_stage):
    port = Port(failed_stage=failed_stage)
    decision = load_sorting_policy(PROFILE).classify('cup', 0.8)
    with pytest.raises(RuntimeError):
        execute_sort('target', decision, port, lambda _: None)
    assert 'release' not in [name for name, _ in port.calls]


def test_release_command_is_not_placement_success():
    port, stages = Port(verified=False), []
    decision = load_sorting_policy(PROFILE).classify('cup', 0.8)
    with pytest.raises(RuntimeError, match='verified'):
        execute_sort('target', decision, port, stages.append)
    assert 'complete' not in stages


def test_review_does_not_command_robot():
    port, stages = Port(), []
    decision = load_sorting_policy(PROFILE).classify('unknown', 0.99)
    assert not execute_sort('target', decision, port, stages.append)
    assert stages == ['review']
    assert port.calls == []
def test_table_slots_include_narrow_interior_and_stay_inside_sphere_margin():
    from cleany_skill_executor.core.sorting import table_placement_slots
    slots = table_placement_slots((.34, 0.), (.22, .22), .076)
    assert len(slots) > 1 and (.34, 0.) in slots
    for x, y in slots:
        assert abs(x - .34) + .076 <= .10 + 1e-12
        assert abs(y) + .076 <= .10 + 1e-12
    assert table_placement_slots((.34, 0.), (.15, .15), .076) == []
