"""실행 상한, 자원 격리와 실패 후 전체 검사 완료를 검증합니다."""
from contextlib import redirect_stdout
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('workspace_parallel', ROOT/'tools/ci/workspace_tests.py')
WORKSPACE = importlib.util.module_from_spec(spec)
spec.loader.exec_module(WORKSPACE)


class ParallelWorkspaceTest(unittest.TestCase):
    def test_bounded_overlap_and_failure_do_not_skip_other_packages(self):
        names = ['cleany_navigation', 'cleany_skill_executor', 'cleany_manipulation_bt', 'extra_a', 'extra_b', 'cleany_gazebo_sim']
        lock, barrier = threading.Lock(), threading.Barrier(2)
        active = peak = 0
        seen, environments = [], []

        def runner(root, package, index, group, started):
            nonlocal active, peak
            env = WORKSPACE.package_environment(root, package, index)
            begin = time.monotonic() - started
            with lock:
                seen.append(package)
                environments.append(env)
                first_pair = len(seen) <= 2
                active += 1
                peak = max(peak, active)
            try:
                if first_pair:
                    barrier.wait(timeout=3)
                return {'package': package, 'status': 'failed' if package == 'extra_a' else 'passed',
                        'started_seconds': begin, 'finished_seconds': time.monotonic() - started}
            finally:
                with lock:
                    active -= 1

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertEqual(WORKSPACE.run_groups(root, names, WORKSPACE.GROUPS, 2, runner), 1)
            self.assertEqual(peak, 2)
            self.assertCountEqual(seen, [name for name in names if name != 'cleany_gazebo_sim'])
            for key in ('ROS_DOMAIN_ID', 'ROS_HOME', 'TMPDIR', 'IGN_PARTITION', 'GZ_PARTITION'):
                self.assertEqual(len({env[key] for env in environments}), len(names) - 1, key)
            for group in WORKSPACE.GROUPS:
                result = json.loads((root/'checks'/group/'result.json').read_text())
                self.assertEqual(result['status'], 'failed' if group == 'workspace-rest' else 'passed')
            schedule = json.loads((root/'checks/workspace/schedule.json').read_text())
            self.assertEqual(schedule['workers'], 2)
            self.assertEqual(len(schedule['packages']), len(names) - 1)

    def test_environment_does_not_mutate_parent_and_reserves_fixture_domains(self):
        before = dict(os.environ)
        with tempfile.TemporaryDirectory() as directory:
            env = WORKSPACE.package_environment(Path(directory), 'cleany_system_supervisor', 0)
            self.assertEqual(env['ROS_DOMAIN_ID'], '40')
            self.assertEqual(env['SUPERVISOR_GRAPH_DOMAIN_BASE'], '190')
            self.assertEqual(env['SUPERVISOR_E2E_DOMAIN_BASE'], '200')
            self.assertEqual(env['CTEST_PARALLEL_LEVEL'], '1')
            with self.assertRaises(ValueError):
                WORKSPACE.package_environment(Path(directory), 'overflow', 60)
        self.assertEqual(dict(os.environ), before)

    def test_zero_exit_without_fresh_result_is_failure(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            root = Path(directory)
            stale = root/'checks/navigation/packages/cleany-navigation/result.json'
            stale.parent.mkdir(parents=True)
            stale.write_text(json.dumps({'status': 'passed'}))
            with mock.patch.object(WORKSPACE.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0)):
                result = WORKSPACE.run_package(root, 'cleany_navigation', 0, 'navigation', time.monotonic())
            self.assertEqual(result['status'], 'failed')


if __name__ == '__main__':
    unittest.main()
