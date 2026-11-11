"""Exercise CI image selection without a registry, containers or APT access."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import textwrap
import unittest

ROOT = Path(__file__).resolve().parents[3]


class DependencyReuseTest(unittest.TestCase):
    def run_prepare(self, *, exact=False, valid=True, build_status=0):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            docker = root / 'docker'
            docker.write_text(f'#!{sys.executable}\n' + textwrap.dedent('''\
                import json, os, sys
                from pathlib import Path
                args = sys.argv[1:]
                with Path(os.environ['CALLS']).open('a') as log:
                    log.write(json.dumps(args) + '\\n')
                if args[0] == 'pull':
                    status = int(not int(os.environ['EXACT'])) if args[1] == os.environ['IMAGE'] else 0
                elif args[0] == 'run':
                    status = int(not int(os.environ['VALID']))
                elif args[0] == 'build':
                    status = int(os.environ['BUILD_STATUS'])
                elif args[0] == 'tag':
                    status = 0
                elif args[:2] == ['image', 'inspect']:
                    print('sha256:checked-image')
                    status = 0
                else:
                    status = 99
                raise SystemExit(status)
                '''))
            docker.chmod(0o755)
            timeout = root / 'timeout'
            timeout.write_text(f'#!{sys.executable}\n' + textwrap.dedent('''\
                import os, sys
                args = sys.argv[1:]
                while args[0].startswith('-'):
                    args.pop(0)
                args.pop(0)
                os.execvp(args[0], args)
                '''))
            timeout.chmod(0o755)
            section = (ROOT / '.github/workflows/ci.yml').read_text().split(
                '      - name: Prepare dependency image\n', 1)[1].split('\n      - name:', 1)[0]
            program = textwrap.dedent(section.split('        run: |\n', 1)[1])
            env = dict(os.environ, PATH=f'{root}:' + os.environ['PATH'],
                       IMAGE='ghcr.io/owner/robot-ci:current', COMPATIBLE_IMAGE='ghcr.io/owner/robot-ci@sha256:pinned',
                       DEPENDENCY_KEY='current', RUNNER_TEMP=str(root), GITHUB_REPOSITORY='owner/robot',
                       GITHUB_OUTPUT=str(root / 'outputs'), GITHUB_STEP_SUMMARY=str(root / 'summary'),
                       CALLS=str(root / 'calls'), EXACT=str(int(exact)), VALID=str(int(valid)),
                       BUILD_STATUS=str(build_status))
            result = subprocess.run(['bash', '-c', program], env=env, capture_output=True,
                                    text=True, timeout=10)
            calls = [json.loads(line) for line in (root / 'calls').read_text().splitlines()]
            return result, calls, (root / 'outputs').read_text()

    def test_exact_hit_needs_no_candidate_or_rebuild(self):
        result, calls, outputs = self.run_prepare(exact=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(outputs, 'hit=true\n')
        self.assertEqual([args[0] for args in calls], ['pull', 'image'])

    def test_candidate_is_checked_before_it_is_tagged_and_published(self):
        result, calls, outputs = self.run_prepare()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(outputs, 'hit=false\n')
        self.assertEqual([args[0] for args in calls], ['pull', 'pull', 'run', 'tag', 'image'])
        command = next(args[-1] for args in calls if args[0] == 'run')
        self.assertIn('rosdep check --from-paths /ci-context/ros2_ws/src', command)
        self.assertIn('python3 /ci-context/containers/ci/verify-versions.py', command)
        self.assertIn('set -eo pipefail', command)

    def test_missing_dependencies_reject_candidate_and_require_a_real_build(self):
        result, calls, outputs = self.run_prepare(valid=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('build', [args[0] for args in calls])
        self.assertNotIn('tag', [args[0] for args in calls])

    def test_failed_build_cannot_be_reported_as_a_usable_image(self):
        result, calls, outputs = self.run_prepare(valid=False, build_status=17)
        self.assertEqual(result.returncode, 17)
        self.assertNotIn('image', [args[0] for args in calls])
