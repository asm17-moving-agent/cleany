"""Receipt verification works for native symlinks and copied installations."""
import hashlib
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'tools/validation'))
from verify_installed_sources import verify_sources


@pytest.fixture
def installation(tmp_path):
    source, install, share = (tmp_path/name for name in ('source', 'install', 'share'))
    for directory in (source, install, share/'config'):
        directory.mkdir(parents=True)
    (source/'node.py').write_text('raise RuntimeError("must not import target module")\n')
    (install/'node.py').symlink_to(source/'node.py')
    (share/'config/runtime.yaml').write_text('enabled: true\n')
    paths = [('demo/node.py', source/'node.py'), ('config/runtime.yaml', share/'config/runtime.yaml')]
    receipt = {'files': [{'path': 'ros2_ws/src/demo/'+name,
                         'after_sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
                        for name, path in paths]}
    receipt['files'].append({'path': 'docs/TESTING.md', 'after_sha256': 'unused'})
    return receipt, {'module_root': lambda _: install, 'share_root': lambda _: share}, source, install


def test_modules_data_symlinks_and_reported_exclusions(installation):
    receipt, roots, _, _ = installation
    result = verify_sources(receipt, packages={'demo'}, **roots)
    assert len(result['checked']) == 2 and result['hashes_match']
    assert result['skipped'] == ['docs/TESTING.md']


@pytest.mark.parametrize('failure', ['hash', 'missing', 'outside', 'package', 'empty', 'duplicate', 'traversal'])
def test_incomplete_or_mismatched_installation_is_rejected(installation, failure):
    receipt, roots, source, install = installation
    options = {}
    if failure == 'hash': (source/'node.py').write_text('changed')
    if failure == 'missing': (source/'node.py').unlink()
    if failure == 'outside': options['resolved_root'] = install
    if failure == 'package': options['packages'] = {'demo', 'absent'}
    if failure == 'empty': receipt['files'] = []
    if failure == 'duplicate': receipt['files'].append(receipt['files'][0])
    if failure == 'traversal': receipt['files'][0]['path'] = 'ros2_ws/src/demo/demo/../node.py'
    with pytest.raises((ValueError, FileNotFoundError)):
        verify_sources(receipt, **roots, **options)
