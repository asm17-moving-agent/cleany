"""Normalize semantic object names without changing detector classes or policy."""
from collections.abc import Sequence


DEFAULT_OBJECT_LABEL_ALIASES = (
    'paper cup=cup', 'plastic cup=cup', 'disposable paper cup=cup',
    'used paper cup=cup', 'mug=cup',
    'mouse=computer mouse', 'wireless mouse=computer mouse',
    'can=beverage can', 'soda can=beverage can', 'remote=remote control',
    'unknown=unknown object', 'unidentified object=unknown object',
)


class ObjectLabelNormalizer:
    """Apply explicit, idempotent aliases; unfamiliar object types remain valid.

    Aliases use ``source=type``. They identify object kinds only and never grant
    disposal permission. Cycles and conflicting source aliases are rejected.
    """

    def __init__(self, aliases: Sequence[str] = DEFAULT_OBJECT_LABEL_ALIASES):
        self.aliases: dict[str, str] = {}
        for entry in aliases:
            parts = entry.split('=')
            if len(parts) != 2 or not all(part.strip() for part in parts):
                raise ValueError('Object label aliases require source=type')
            source, target = (self._key(part) for part in parts)
            if source in self.aliases:
                raise ValueError(f'Duplicate object label alias: {source}')
            self.aliases[source] = target
        for source in self.aliases:
            self(source)

    @staticmethod
    def _key(label: str) -> str:
        return ' '.join(label.casefold().split())

    def __call__(self, label: str) -> str:
        key = self._key(label)
        if not key:
            raise ValueError('Object label must not be empty')
        visited: set[str] = set()
        while key in self.aliases and self.aliases[key] != key:
            if key in visited:
                raise ValueError('Object label aliases contain a cycle')
            visited.add(key)
            key = self.aliases[key]
        return key
