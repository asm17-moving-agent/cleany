"""Exercise image invalidation and the dependency-only build boundary."""
from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location('dependency_image', ROOT / 'tools/ci/dependency_image.py')
IMAGE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(IMAGE)


class DependencyImageTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name) / 'source'
        for name, content in {
            'containers/ci/Dockerfile': 'FROM example@sha256:abc',
            'containers/ci/pip-constraints.txt': 'example==1',
            'containers/ci/install-dependencies': 'install dependencies',
            'tools/ci/dependency_image.py': 'helper version 1',
            'ros2_ws/rosdep/cleany.yaml': 'rules: {}',
            'ros2_ws/src/robot/package.xml': '<package/>',
            'ros2_ws/src/robot/node.py': 'application code',
            'ros2_ws/src/robot/config/runtime.yaml': 'runtime: true',
        }.items():
            self.write(name, content)

    def write(self, name, content):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
        return path

    def key(self, epoch='2026w43', platform='linux/amd64'):
        return IMAGE.dependency_key(self.root, epoch, platform)

    def test_application_changes_do_not_rebuild_the_dependency_image(self):
        before = self.key()
        self.write('ros2_ws/src/robot/node.py', 'changed code')
        self.write('ros2_ws/src/robot/config/runtime.yaml', 'runtime: false')
        self.write('ros2_ws/src/robot/msg/New.msg', 'int32 value')
        self.assertEqual(self.key(), before)

    def test_each_dependency_input_invalidates_the_image(self):
        for name in (
            'containers/ci/Dockerfile', 'containers/ci/pip-constraints.txt',
            'containers/ci/install-dependencies', 'tools/ci/dependency_image.py',
            'ros2_ws/rosdep/cleany.yaml', 'ros2_ws/src/robot/package.xml',
        ):
            with self.subTest(name=name):
                before = self.key()
                path = self.root / name
                path.write_text(path.read_text() + '\nchanged')
                self.assertNotEqual(self.key(), before)

    def test_package_addition_removal_and_rename_invalidate(self):
        original = self.key()
        path = self.write('ros2_ws/src/new/package.xml', '<package/>')
        added = self.key()
        self.assertNotEqual(original, added)
        path.unlink()
        self.assertEqual(self.key(), original)
        path = self.root / 'ros2_ws/src/robot/package.xml'
        path.rename(path.with_name('renamed.xml'))
        self.assertNotEqual(self.key(), original)

    def test_refresh_epoch_and_architecture_are_separate(self):
        self.assertNotEqual(self.key(), self.key(epoch='2026w44'))
        self.assertNotEqual(self.key(), self.key(platform='linux/arm64'))

    def test_touch_does_not_invalidate(self):
        before = self.key()
        os.utime(self.root / 'ros2_ws/src/robot/package.xml', (100, 100))
        self.assertEqual(self.key(), before)

    def test_context_excludes_sources_runtime_configs_and_builds(self):
        self.write('ros2_ws/build/robot/package.xml', 'generated output')
        context = Path(self.directory.name) / 'context'
        IMAGE.prepare_context(self.root, context)
        copied = {p.relative_to(context).as_posix() for p in context.rglob('*') if p.is_file()}
        self.assertIn('ros2_ws/src/robot/package.xml', copied)
        self.assertNotIn('ros2_ws/src/robot/node.py', copied)
        self.assertNotIn('ros2_ws/src/robot/config/runtime.yaml', copied)
        self.assertNotIn('ros2_ws/build/robot/package.xml', copied)
        with self.assertRaises(FileExistsError):
            IMAGE.prepare_context(self.root, context)

    def test_cli_outputs_a_repository_scoped_key(self):
        context = Path(self.directory.name) / 'context'
        result = subprocess.run([
            sys.executable, str(ROOT/'tools/ci/dependency_image.py'),
            '--root', str(self.root), '--context', str(context),
            '--repository', 'Owner/Robot', '--epoch', '2026w43',
        ], capture_output=True, text=True, check=True)
        outputs = dict(line.split('=', 1) for line in result.stdout.splitlines())
        self.assertEqual(outputs['key'], self.key())
        self.assertEqual(outputs['image'], f'ghcr.io/owner/robot-ci:humble-{self.key()}')

    def test_invalid_repository_cannot_inject_image_arguments(self):
        context = Path(self.directory.name) / 'context'
        result = subprocess.run([
            sys.executable, str(ROOT/'tools/ci/dependency_image.py'),
            '--root', str(self.root), '--context', str(context),
            '--repository', 'owner/robot --other-argument',
        ], capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(context.exists())


class ContainerCommandTest(unittest.TestCase):
    def test_arguments_and_failure_status_reach_the_caller(self):
        with tempfile.TemporaryDirectory() as directory:
            fake = Path(directory)/'docker'
            fake.write_text('#!/usr/bin/env python3\nimport sys\nprint(repr(sys.argv[1:]))\nsys.exit(17)\n')
            fake.chmod(0o755)
            result = subprocess.run([str(ROOT/'tools/ci/run'), 'printf', 'one argument with spaces'],
                env=dict(os.environ, PATH=directory+':'+os.environ['PATH']),
                capture_output=True, text=True)
            self.assertEqual(result.returncode, 17)
            self.assertIn("'one argument with spaces'", result.stdout)
            self.assertIn("'cleany-ci'", result.stdout)
