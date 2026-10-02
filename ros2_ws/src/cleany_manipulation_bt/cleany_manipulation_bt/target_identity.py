"""Explicit observation-label aliases for the existing simulation result evaluator.

Aliases name the same supported object type; they do not grant disposal permission.
Model category, uniqueness and geometric continuity are still required separately.
"""

VERIFICATION_LABELS = {
    'cup': 'cup',
    'paper cup': 'cup',
    'disposable paper cup': 'cup',
    'crumpled tissue': 'crumpled tissue',
}


def label_key(label: str) -> str:
    normalized = ' '.join(label.strip().lower().split())
    return VERIFICATION_LABELS.get(normalized, normalized)
