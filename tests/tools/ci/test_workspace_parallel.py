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
        names = ['cleany_navigation', 'cleany_skill_executor', 'cleany_manipulation_bt', 'extra_a', 'extra_b', 'cleany_gazebo_sim', 'cleany_base_driver']
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
                active += 1
                peak = max(peak, active)
            try:
                if package in ('cleany_manipulation_bt', 'cleany_skill_executor'):
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
            self.assertCountEqual(seen, names)
            for key in ('ROS_DOMAIN_ID', 'ROS_HOME', 'TMPDIR', 'IGN_PARTITION', 'GZ_PARTITION'):
                self.assertEqual(len({env[key] for env in environments}), len(names), key)
            for group in WORKSPACE.GROUPS:
                result = json.loads((root/'checks'/group/'result.json').read_text())
                self.assertEqual(result['status'], 'failed' if group == 'workspace-rest' else 'passed')
            schedule = json.loads((root/'checks/workspace/schedule.json').read_text())
            self.assertEqual(schedule['workers'], 2)
            records = {p['package']: p for p in schedule['packages']}
            self.assertEqual(len(records), len(names))
            base = records.pop('cleany_base_driver')
            self.assertLessEqual(base['finished_seconds'], min(p['started_seconds'] for p in records.values()))

    def test_environment_does_not_mutate_parent_and_reserves_fixture_domains(self):
        before = dict(os.environ)
        with tempfile.TemporaryDirectory() as directory:
            env = WORKSPACE.package_environment(Path(directory), 'cleany_system_supervisor', 0)
            self.assertEqual(env['ROS_DOMAIN_ID'], '40')
            self.assertEqual(env['SUPERVISOR_GRAPH_DOMAIN_BASE'], '190')
            self.assertEqual(env['SUPERVISOR_E2E_DOMAIN_BASE'], '200')
            self.assertEqual(env['CTEST_PARALLEL_LEVEL'], '1')
            base_env = WORKSPACE.package_environment(Path(directory), 'cleany_base_driver', 1)
            self.assertEqual(base_env['BASE_EKF_TEST_ROS_DOMAIN_ID'], '26')
            with self.assertRaises(ValueError):
                WORKSPACE.package_environment(Path(directory), 'overflow', 60)
        self.assertEqual(dict(os.environ), before)

    def test_long_packages_are_dispatched_before_short_and_new_packages(self):
        names = ['new_package', 'cleany_navigation', 'cleany_gazebo_sim',
                 'cleany_interfaces', 'cleany_base_driver', 'cleany_skill_executor']
        seen = []

        def runner(root, package, index, group, started):
            seen.append(package)
            return {'package': package, 'status': 'passed'}

        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            status = WORKSPACE.run_groups(Path(directory), names, WORKSPACE.GROUPS, 1, runner)

        self.assertEqual(status, 0)
        self.assertEqual(seen, ['cleany_base_driver', 'cleany_skill_executor', 'cleany_navigation',
                                'cleany_gazebo_sim', 'cleany_interfaces', 'new_package'])

    def test_runtime_overlaps_package_and_keeps_separate_result(self):
        names = ['cleany_base_driver', 'cleany_skill_executor', 'cleany_navigation', 'extra']
        watchdog_finished = threading.Event()
        skill_running = threading.Event()
        runtime_finished = threading.Event()

        def runner(root, package, index, group, started):
            if package == 'cleany_base_driver':
                watchdog_finished.set()
            if package == 'cleany_skill_executor':
                skill_running.set()
                self.assertTrue(runtime_finished.wait(timeout=3))
            return {'package': package, 'status': 'passed'}

        def runtime(root, started):
            self.assertTrue(watchdog_finished.is_set())
            self.assertTrue(skill_running.wait(timeout=3))
            runtime_finished.set()
            return {'name': 'ros-runtime', 'status': 'failed'}

        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            root = Path(directory)
            old_result = root/'checks/ros-runtime/result.json'
            old_result.parent.mkdir(parents=True)
            old_result.write_text('{"status":"passed"}')
            status = WORKSPACE.run_groups(root, names, WORKSPACE.GROUPS, 2, runner,
                                          navigation_runner=runtime)
            self.assertEqual(status, 1)
            self.assertFalse(old_result.exists())
            schedule = json.loads((root/'checks/workspace/schedule.json').read_text())
            self.assertCountEqual([p['package'] for p in schedule['packages']], names)
            self.assertTrue(all(p['status'] == 'passed' for p in schedule['packages']))
            self.assertEqual(schedule['ros-runtime']['status'], 'failed')

    def test_runtime_reserves_domains_and_reuses_workspace_build(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            root = Path(directory)
            with mock.patch.object(WORKSPACE.subprocess, 'run',
                                   return_value=subprocess.CompletedProcess([], 0)) as run:
                result = WORKSPACE.run_navigation_runtime(root, time.monotonic())
            command = run.call_args.args[0]
            env = result['environment']
            domain = int(env['CLEANY_TEST_ROS_DOMAIN_BASE'])
            self.assertEqual({domain, domain + 2, domain + 4}, {30, 32, 34})
            self.assertEqual(env['ROS_LOCALHOST_ONLY'], '1')
            self.assertEqual(env['ROS_HOME'], str(root/'test-environments/ros-runtime/ros'))
            self.assertEqual(env['TMPDIR'], str(root/'test-environments/ros-runtime/tmp'))
            self.assertEqual(run.call_args.kwargs['cwd'], ROOT)
            self.assertIn('ros-runtime', command)
            self.assertEqual(command[-7:-3], ['make', '-o', 'build-navigation', 'test-navigation-runtime'])
            self.assertIn(f'NAVIGATION_INSTALL_BASE={root/"install"}', command)
            self.assertEqual(result['status'], 'passed')

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
