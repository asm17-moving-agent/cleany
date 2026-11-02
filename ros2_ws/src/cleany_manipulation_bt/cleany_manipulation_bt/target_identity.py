"""Explicit observation-label aliases for the existing simulation result evaluator.

Aliases name the same supported object type; they do not grant disposal permission.
Model category, uniqueness and geometric continuity are still required separately.
"""

VERIFICATION_LABELS = {
    'cup': 'cup',
    'paper cup': 'cup',
    'disposable paper cup': 'cup',
    'crumpled tissue': 'crumpled tissue',
    'mouse': 'mouse',
    'computer mouse': 'mouse',
    'wireless mouse': 'mouse',
    'lego brick': 'lego brick',
}

TRASH_LABELS = ('cup', 'paper cup', 'disposable paper cup', 'crumpled tissue')
LOST_ITEM_LABELS = ('mouse', 'computer mouse', 'wireless mouse', 'lego brick')


def label_key(label: str) -> str:
    normalized = ' '.join(label.strip().lower().split())
    return VERIFICATION_LABELS.get(normalized, normalized)
