"""Exercise bounded retries against real child-process exit codes."""

from pathlib import Path
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[3]


class RetryCommandTest(unittest.TestCase):
    def run_command(self, failures, code=100):
        with tempfile.TemporaryDirectory() as directory:
            counter = Path(directory) / 'attempts'
            child = Path(directory) / 'command.py'
            child.write_text(
                'from pathlib import Path\nimport sys\n'
                f'p=Path({str(counter)!r})\n'
                'n=int(p.read_text())+1 if p.exists() else 1\n'
                'p.write_text(str(n))\n'
                f'raise SystemExit({code} if n <= {failures} else 0)\n'
            )
            result = subprocess.run(
                [sys.executable, str(ROOT / 'containers/ci/retry-command.py'),
                 '--delay', '0', '--', sys.executable, str(child)],
                capture_output=True, text=True,
            )
            return result, int(counter.read_text())

    def test_transient_download_failure_recovers(self):
        result, attempts = self.run_command(failures=1)
        self.assertEqual(result.returncode, 0)
        self.assertEqual(attempts, 2)

    def test_persistent_failure_keeps_its_exit_code_and_stops(self):
        result, attempts = self.run_command(failures=10, code=7)
        self.assertEqual(result.returncode, 7)
        self.assertEqual(attempts, 3)

    def test_success_is_not_repeated(self):
        result, attempts = self.run_command(failures=0)
        self.assertEqual(result.returncode, 0)
        self.assertEqual(attempts, 1)

    def test_zero_attempts_cannot_bypass_the_command(self):
        result = subprocess.run(
            [sys.executable, str(ROOT / 'containers/ci/retry-command.py'),
             '--attempts', '0', '--', sys.executable, '-c', 'raise SystemExit(0)'],
            capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 2)

    def test_constraints_and_argument_boundaries_reach_the_child(self):
        result = subprocess.run(
            [sys.executable, str(ROOT / 'containers/ci/retry-command.py'), '--delay', '0', '--',
             sys.executable, '-c',
             'import os,sys; assert os.environ["PIP_CONSTRAINT"] == "fixed-pins.txt"; '
             'assert sys.argv[1:] == ["one argument with spaces"]', 'one argument with spaces'],
            env=dict(os.environ, PIP_CONSTRAINT='fixed-pins.txt'),
            capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
