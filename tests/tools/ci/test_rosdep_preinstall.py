"""Check that batched APT preparation preserves rosdep's dependency boundary."""
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import subprocess
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[3]
SPEC = spec_from_file_location('preinstall', ROOT/'containers/ci/preinstall-rosdep-apt.py')
preinstall = module_from_spec(SPEC)
SPEC.loader.exec_module(preinstall)


class RosdepPreinstallTest(unittest.TestCase):
    def test_real_plan_groups_preserve_versions_and_ignore_other_installers(self):
        plan = '''#[apt] Installation commands:
  apt-get install -y python3-pytest
  apt-get install -y libexample:amd64=2:1.0+dfsg-1~22.04
#[pip] Installation commands:
  pip3 install -U google-genai
#[apt] Installation commands:
  sudo -H apt-get install -y python3-pytest ros-humble-nav2-bringup
'''
        self.assertEqual(preinstall.apt_packages(plan), [
            'python3-pytest', 'libexample:amd64=2:1.0+dfsg-1~22.04',
            'ros-humble-nav2-bringup'])

    def test_options_and_alternatives_are_left_to_normal_rosdep(self):
        for line in ('apt-get install -y first || apt-get install -y second',
                     'apt-get install -y --allow-unauthenticated first',
                     'apt-get install -y first; touch /tmp/unwanted'):
            with self.subTest(line=line):
                self.assertEqual(preinstall.apt_packages(line), [])

    def test_batches_keep_every_requirement_and_forward_resolver_arguments(self):
        names = [f'package-{i}' for i in range(preinstall.BATCH_SIZE + 2)]
        plan = '\n'.join(f'apt-get install -y {name}' for name in names)
        with patch.object(preinstall.subprocess, 'run', return_value=
                          subprocess.CompletedProcess([], 0, stdout=plan)) as run:
            preinstall.main(['--from-paths', '/path with spaces', '--ignore-src', '-y'])
        self.assertEqual(run.call_args_list[0].args[0], [
            'rosdep', 'install', '--simulate', '--from-paths',
            '/path with spaces', '--ignore-src', '-y'])
        installed = [name for call in run.call_args_list[1:] for name in call.args[0][3:]]
        self.assertEqual(installed, names)
        self.assertEqual(len(run.call_args_list), 3)

    def test_failed_resolution_does_not_start_installation(self):
        with patch.object(preinstall.subprocess, 'run', side_effect=
                          subprocess.CalledProcessError(1, ['rosdep'])) as run:
            with self.assertRaises(subprocess.CalledProcessError):
                preinstall.main([])
        self.assertEqual(run.call_count, 1)

    def test_failed_installation_stops_preparation(self):
        with patch.object(preinstall.subprocess, 'run', side_effect=[
                subprocess.CompletedProcess([], 0, stdout='apt-get install -y package'),
                subprocess.CalledProcessError(100, ['apt-get'])]):
            with self.assertRaises(subprocess.CalledProcessError):
                preinstall.main([])

    def test_no_apt_requirements_leaves_pip_for_rosdep(self):
        with patch.object(preinstall.subprocess, 'run', return_value=
                          subprocess.CompletedProcess([], 0, stdout='pip3 install -U mujoco')) as run:
            preinstall.main([])
        self.assertEqual(run.call_count, 1)
