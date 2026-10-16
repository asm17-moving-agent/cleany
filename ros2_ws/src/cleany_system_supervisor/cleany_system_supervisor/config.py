"""Load and reject ambiguous or fail-open policy graphs at startup."""
from math import isfinite
from pathlib import Path

import yaml


def validate(config: dict) -> dict:
    def positive(value, name, allow_zero=False):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
            raise ValueError(f'{name} must be finite')
        if value < 0 or (value == 0 and not allow_zero):
            raise ValueError(f'{name} out of range')

    for key in ('startup_grace_s', 'lease_s', 'heartbeat_timeout_s', 'stop_timeout_s'):
        positive(config[key], key)
    sources, caps, policies = config['sources'], config['capabilities'], config['recovery_policies']
    if not sources or not caps or not config['essential_capabilities']:
        raise ValueError('sources, capabilities and essential capabilities must be nonempty')
    if not config['safety_sources'] or any(source not in sources for source in config['safety_sources']):
        raise ValueError('known nonempty safety sources required')
    if not config['recovery_safety_sources'] or any(
            source not in sources for source in config['recovery_safety_sources']):
        raise ValueError('known nonempty recovery safety sources required')
    for name in config['essential_capabilities']:
        if name not in caps:
            raise ValueError(f'unknown essential capability {name}')
    for source, entry in sources.items():
        for key in ('stale_s', 'raise_s', 'clear_s'):
            positive(entry[key], f'{source}.{key}', allow_zero=key == 'raise_s')
        if entry['severity'] not in ('WARNING', 'ERROR', 'CRITICAL'):
            raise ValueError('invalid severity')
        if entry.get('recovery', '') not in ('', *policies):
            raise ValueError('unknown recovery policy')
        if not isinstance(entry['latched'], bool) or not isinstance(entry['locks_operation'], bool):
            raise ValueError('fault latch and lock settings must be booleans')
    codes = [entry['fault_code'] for entry in sources.values()]
    if len(codes) != len(set(codes)):
        raise ValueError('fault codes must be unique')
    visited, visiting = set(), set()

    def visit(name):
        if name not in caps or name in visiting:
            raise ValueError(f'unknown capability or dependency cycle: {name}')
        if name in visited:
            return
        visiting.add(name)
        entry = caps[name]
        if not entry['sources'] and not entry['depends_on']:
            raise ValueError(f'{name} has no health evidence')
        if any(source not in sources for source in entry['sources']):
            raise ValueError(f'{name} references unknown source')
        for dependency in entry['depends_on']:
            visit(dependency)
        visiting.remove(name)
        visited.add(name)
    for name in caps:
        visit(name)
    for name, policy in policies.items():
        for key in ('attempt_timeout_s', 'total_timeout_s', 'verify_timeout_s', 'cancel_timeout_s'):
            positive(policy[key], f'{name}.{key}')
        if type(policy['max_attempts']) is not int or policy['max_attempts'] < 1:
            raise ValueError('max_attempts must be a positive integer')
        if not isinstance(policy['automatic'], bool) or not policy['action'] or policy['action'].startswith('/'):
            raise ValueError('recovery needs a relative action and boolean automatic policy')
    return config


def load(path: str | Path) -> dict:
    with open(path, encoding='utf-8') as stream:
        return validate(yaml.safe_load(stream))
