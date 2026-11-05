"""Check mirror fallback against the immutable package hash and deadline."""

import hashlib
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
import io
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch
from urllib.error import HTTPError

ROOT = Path(__file__).resolve().parents[3]
loader = SourceFileLoader('ci_apt', str(ROOT / 'containers/ci/apt-get'))
apt = module_from_spec(spec_from_loader(loader.name, loader))
loader.exec_module(apt)

URI = 'https://snapshot.ubuntu.com/ubuntu/20260807T000000Z/pool/main/p/pkg.deb'


class PackageDownloadTest(unittest.TestCase):
    def download(self, cache, payload=b'fixed package'):
        apt.fetch_package(URI, 'pkg.deb', len(payload),
                          'SHA256:' + hashlib.sha256(payload).hexdigest(),
                          time.monotonic() + 10, Path(cache))

    def test_mirror_uses_identical_pool_file_and_ros_is_untouched(self):
        self.assertEqual(apt.package_urls(URI), [
            'https://archive.ubuntu.com/ubuntu/pool/main/p/pkg.deb', URI])
        ros = 'http://snapshots.ros.org/humble/fixed/pkg.deb'
        self.assertEqual(apt.package_urls(ros), [ros])

    def test_verified_mirror_file_is_cached_and_reused(self):
        with tempfile.TemporaryDirectory() as cache, patch.object(
                apt, 'urlopen', return_value=io.BytesIO(b'fixed package')) as request:
            self.download(cache)
            self.download(cache)
            self.assertEqual(request.call_count, 1)
            self.assertEqual((Path(cache) / 'pkg.deb').read_bytes(), b'fixed package')

    def test_missing_mirror_file_falls_back_to_snapshot(self):
        with tempfile.TemporaryDirectory() as cache, patch.object(
                apt, 'urlopen', side_effect=[HTTPError(URI, 404, 'missing', {}, None),
                                            io.BytesIO(b'fixed package')]) as request:
            self.download(cache)
            self.assertEqual(request.call_args_list[1].args[0], URI)

    def test_wrong_hash_never_enters_cache(self):
        with tempfile.TemporaryDirectory() as cache, patch.object(
                apt, 'urlopen', side_effect=lambda *a, **k: io.BytesIO(b'wrong package')):
            with self.assertRaises(RuntimeError):
                self.download(cache)
            self.assertEqual(list(Path(cache).iterdir()), [])

    def test_expired_deadline_makes_no_network_request(self):
        with tempfile.TemporaryDirectory() as cache, patch.object(apt, 'urlopen') as request:
            with self.assertRaises(RuntimeError):
                apt.fetch_package(URI, 'pkg.deb', 1, 'SHA256:unused',
                                  time.monotonic() - 1, Path(cache))
            request.assert_not_called()

    def test_weak_hash_is_rejected(self):
        with tempfile.TemporaryDirectory() as cache, patch.object(apt, 'urlopen') as request:
            with self.assertRaises(ValueError):
                apt.fetch_package(URI, 'pkg.deb', 1, 'MD5Sum:unused',
                                  time.monotonic() + 10, Path(cache))
            request.assert_not_called()
