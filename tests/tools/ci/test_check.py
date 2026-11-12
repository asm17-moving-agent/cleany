"""실패와 제한시간, 검사 누락이 성공으로 바뀌지 않는지 확인합니다."""
import importlib.util
import json
import os
import signal
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[3]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT/'tools/ci'/f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CHECK = load('check')
WORKSPACE = load('workspace_tests')


class CheckTest(unittest.TestCase):
    def run_check(self, root, code, timeout=2, ignore_interrupt=False):
        return subprocess.run([sys.executable, str(ROOT/'tools/ci/check.py'), '--root', str(root),
            'run', 'sample', '--kind', 'contract', '--timeout', str(timeout),
            '--diagnostic-after', '0.15', '--', sys.executable, '-c', code],
            cwd=ROOT, capture_output=True, text=True, timeout=10,
            preexec_fn=(lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)) if ignore_interrupt else None)

    def test_success_and_failure_keep_exit_code_log_and_reproduction(self):
        for expected in (0, 7):
            with self.subTest(code=expected), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                run = self.run_check(root, f'print("marker"); raise SystemExit({expected})')
                self.assertEqual(run.returncode, expected)
                item = json.loads((root/'sample/result.json').read_text())
                self.assertEqual(item['status'], 'passed' if expected == 0 else 'failed')
                self.assertIn('marker', (root/'sample/output.log').read_text())
                self.assertTrue(item['commit'])
                self.assertIn('marker', item['command'])

    def test_hang_records_diagnostics_and_stops_its_child(self):
        for detached in (False, True):
            with self.subTest(detached=detached), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                code = f'import subprocess,time; p=subprocess.Popen(["sleep","30"],start_new_session={detached}); print(p.pid,flush=True); time.sleep(30)'
                run = self.run_check(root, code, .3)
                self.assertEqual(run.returncode, 124, run.stderr)
                self.assertEqual(json.loads((root/'sample/result.json').read_text())['status'], 'timeout')
                snapshot = json.loads((root/'sample/timeout/processes.json').read_text())
                self.assertTrue(any('sleep 30' in r['command'] for r in snapshot))
                child = int((root/'sample/output.log').read_text().splitlines()[0])
                path = Path('/proc')/str(child)/'stat'
                self.assertTrue(not path.exists() or path.read_text().rsplit(')', 1)[1].split()[0] == 'Z')

    def test_ctest_timeout_is_reported_as_timeout(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.run_check(root, 'print("mppi_critics_tests ***Timeout"); raise SystemExit(1)')
            self.assertEqual(json.loads((root/'sample/result.json').read_text())['status'], 'timeout')

    def test_pytest_can_enable_its_own_faulthandler(self):
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(os.environ, {'PYTHONFAULTHANDLER': '1'}):
            run = self.run_check(Path(directory), 'import faulthandler; assert not faulthandler.is_enabled()')
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)

    def test_background_shell_does_not_disable_child_interrupts(self):
        with tempfile.TemporaryDirectory() as directory:
            run = self.run_check(Path(directory),
                'import signal; assert signal.getsignal(signal.SIGINT) != signal.SIG_IGN',
                ignore_interrupt=True)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)

    def test_missing_and_running_results_are_not_success(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertFalse(CHECK.summarize(root, ['never-started'])[1])
            (root/'partial').mkdir()
            (root/'partial/result.json').write_text(json.dumps({'status': 'running'}))
            self.assertFalse(CHECK.summarize(root, ['partial'])[1])

    def test_corrupt_result_keeps_other_checks_visible(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'corrupt').mkdir()
            (root/'corrupt/result.json').write_text('{')
            (root/'healthy').mkdir()
            (root/'healthy/result.json').write_text(json.dumps({'status': 'passed'}))
            report, ok = CHECK.summarize(root, ['corrupt', 'healthy'])
            self.assertFalse(ok)
            self.assertIn('invalid_result', report)
            self.assertIn('| healthy |', report)

    def test_partition_covers_new_packages_once(self):
        names = ['cleany_navigation', 'cleany_manipulation_bt', 'cleany_skill_executor', 'new_package']
        groups = ['navigation', 'manipulation', 'workspace-rest']
        selected = [p for group in groups for p in WORKSPACE.group_packages(names, group)]
        self.assertCountEqual(selected, names)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for group in groups:
                path = root/group
                path.mkdir()
                (path/'result.json').write_text(json.dumps({'status': 'passed'}))
                (path/'packages.json').write_text(json.dumps({'all': names, 'selected': WORKSPACE.group_packages(names, group)}))
            self.assertTrue(CHECK.summarize(root, groups)[1])
            path = root/'workspace-rest/packages.json'
            path.write_text(json.dumps({'all': names, 'selected': []}))
            self.assertFalse(CHECK.summarize(root, groups)[1])


if __name__ == '__main__':
    unittest.main()
