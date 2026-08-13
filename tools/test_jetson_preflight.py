from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile
from types import ModuleType
import unittest
from unittest.mock import patch


def _load_module() -> ModuleType:
    path = Path(__file__).with_name('jetson_preflight.py')
    spec = importlib.util.spec_from_file_location('jetson_preflight', path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


preflight = _load_module()


class JetsonPreflightTest(unittest.TestCase):
    def test_resolve_command_uses_fallback_when_path_is_not_sourced(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fallback = Path(directory) / 'ros2'
            fallback.touch(mode=0o755)

            with patch.object(preflight.shutil, 'which', return_value=None):
                result = preflight.resolve_command('ros2', [fallback])

        self.assertEqual(result, str(fallback))

    def test_resolve_command_prefers_path(self) -> None:
        with patch.object(
            preflight.shutil,
            'which',
            return_value='/custom/bin/ros2',
        ):
            result = preflight.resolve_command(
                'ros2',
                [Path('/opt/ros/humble/bin/ros2')],
            )

        self.assertEqual(result, '/custom/bin/ros2')

    def test_parse_os_release_removes_quotes(self) -> None:
        result = preflight.parse_os_release(
            'NAME="Ubuntu"\nVERSION_ID="22.04"\n# ignored\n'
        )

        self.assertEqual(result, {'NAME': 'Ubuntu', 'VERSION_ID': '22.04'})

    def test_parse_l4t_release(self) -> None:
        result = preflight.parse_l4t_release(
            '# R36 (release), REVISION: 4.7, GCID: 44060909, BOARD: generic'
        )

        self.assertEqual(result['release'], '36')
        self.assertEqual(result['revision'], '4.7')
        self.assertEqual(result['version'], '36.4.7')
        self.assertEqual(result['gcid'], '44060909')

    def test_parse_nvpmodel_does_not_assume_mode_id(self) -> None:
        result = preflight.parse_nvpmodel('MAXN_SUPER\n0\n')

        self.assertEqual(result['name'], 'MAXN_SUPER')
        self.assertEqual(result['id'], 0)

    def test_parse_dpkg_packages_keeps_version_and_architecture(self) -> None:
        result = preflight.parse_dpkg_packages(
            'Desired=Unknown/Install\n'
            'ii  nvidia-jetpack  6.2.1-b123  arm64  '
            'NVIDIA JetPack meta-package\n'
            'un  ignored        <none>      <none>\n'
        )

        self.assertEqual(
            result,
            [
                {
                    'name': 'nvidia-jetpack',
                    'version': '6.2.1-b123',
                    'architecture': 'arm64',
                }
            ],
        )

    def test_base_checks_reports_individual_failures(self) -> None:
        report = {
            'system': {
                'architecture': 'aarch64',
                'os_release': {'ID': 'ubuntu', 'VERSION_ID': '22.04'},
            },
            'python': {'major_minor': '3.10'},
            'jetson': {
                'l4t': {'version': '36.4.7'},
                'jetpack_packages': [{'name': 'nvidia-jetpack'}],
                'nvpmodel': {'id': 3},
            },
            'cuda': {'version': '12.6'},
            'cudnn_packages': [{'name': 'libcudnn9'}],
            'tensorrt_packages': [],
        }

        checks = preflight.base_checks(report)

        self.assertTrue(checks['architecture_is_aarch64'])
        self.assertTrue(checks['ubuntu_is_22_04'])
        self.assertTrue(checks['python_is_3_10'])
        self.assertFalse(checks['tensorrt_detected'])
        self.assertFalse(all(checks.values()))


if __name__ == '__main__':
    unittest.main()
