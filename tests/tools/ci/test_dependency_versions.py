"""Verify cross-architecture dependency comparison and missing-package failures."""

import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location('verify_versions', ROOT / 'containers/ci/verify-versions.py')
VERSIONS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VERSIONS)


class DependencyVersionsTest(unittest.TestCase):
    def test_python_missing_or_different_version_fails(self):
        errors = VERSIONS.version_errors(
            {'mujoco': '3.11.0', 'google-genai': '2.18.0'},
            {'mujoco': '3.16.0'}, {}, {},
        )
        self.assertEqual(len(errors), 2)
        self.assertTrue(any('missing' in error for error in errors))

    def test_ros_architecture_build_timestamps_do_not_change_upstream_version(self):
        for version in ('2.5.9-1jammy.20260804.210148', '2.5.9-3jammy.20260304.201222'):
            with self.subTest(version=version):
                self.assertEqual(VERSIONS.version_errors(
                    {}, {}, {'ros-humble-moveit-core': '2.5.9'},
                    {'ros-humble-moveit-core': version},
                ), [])

    def test_shared_ros_upstream_mismatch_fails(self):
        errors = VERSIONS.version_errors(
            {}, {}, {'ros-humble-moveit-core': '2.5.9'},
            {'ros-humble-moveit-core': '2.5.10-1jammy.20261005.213935'},
        )
        self.assertEqual(len(errors), 1)

    def test_jetson_only_packages_do_not_become_ci_dependencies(self):
        self.assertEqual(VERSIONS.version_errors(
            {'mujoco': '3.11.0'}, {'mujoco': '3.11.0'},
            {'ros-humble-desktop': '0.10.0'}, {'ros-humble-nav2-common': '1.1.20-1jammy.1'},
        ), [])
