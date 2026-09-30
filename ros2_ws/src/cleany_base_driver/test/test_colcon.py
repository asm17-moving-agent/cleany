"""Bridge colcon's unittest discovery to the package's pytest tests."""
import os
from pathlib import Path
import subprocess
import unittest


class PytestSuite(unittest.TestCase):
    def test_driver_core_profiles_and_mock_graph(self):
        root = Path(__file__).parent
        env = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
        result = subprocess.run(
            ['/usr/bin/python3', '-m', 'pytest', '-q',
             str(root / 'test_core.py'), str(root / 'test_profiles.py'),
             str(root / 'test_mock_gate.py'), str(root / 'test_graph.py')],
            cwd=root.parent, env=env, text=True, capture_output=True,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
