"""Verify exclusive timing and CPU/wait separation with deterministic clocks."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('isaac_profiling',
    Path(__file__).resolve().parents[2]/'tools/validation/isaac/profiling.py')
profiling = importlib.util.module_from_spec(spec)
spec.loader.exec_module(profiling)
analysis_spec = importlib.util.spec_from_file_location('isaac_profile_analysis',
    Path(__file__).resolve().parents[2]/'tools/validation/isaac/analyze_profile.py')
analysis = importlib.util.module_from_spec(analysis_spec)
analysis_spec.loader.exec_module(analysis)
native_spec = importlib.util.spec_from_file_location('isaac_native_analysis',
    Path(__file__).resolve().parents[2]/'tools/validation/isaac/analyze_native.py')
native = importlib.util.module_from_spec(native_spec)
native_spec.loader.exec_module(native)


class LoopProfilerTest(unittest.TestCase):
    def test_native_nested_zones_are_not_double_counted(self):
        rows = native.exclusive_main_zones([
            {'name': 'Child', 'ts': 2., 'dur': 5.},
            {'name': 'Leaf', 'ts': 3., 'dur': 2.},
            {'name': 'Outside', 'ts': 20., 'dur': 30.},
            {'name': 'Cleany::PhysicsStep', 'ts': 1., 'dur': 10.}])
        self.assertEqual({r['name']: r['exclusive_us'] for r in rows},
                         {'Cleany::PhysicsStep': 5., 'Child': 3., 'Leaf': 2.})

    def test_exclusive_stages_and_wall_cpu_gap(self):
        clock = {'wall': 0., 'cpu': 0.}
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(profiling.time, 'perf_counter', side_effect=lambda: clock['wall']), \
                patch.object(profiling.time, 'thread_time', side_effect=lambda: clock['cpu']):
            path = Path(folder)/'profile.jsonl'
            recorder = profiling.LoopProfiler(path, 500, .003)
            clock.update(wall=.001, cpu=.0005)
            recorder.mark('command')
            clock.update(wall=.003, cpu=.0015)
            recorder.mark('physics')
            recorder.before_sleep(0.)
            clock.update(wall=.004, cpu=.0015)
            recorder.mark('pacing')
            recorder.finish()
            recorder.close()
            rows = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual(len(rows), 1)
        row = rows[0]
        self.assertEqual(row['compute_overrun_loops'], 1)
        self.assertEqual(row['rtf'], .5)
        self.assertAlmostEqual(sum(s['wall']['sum_s'] for s in row['stages'].values()), .004)
        self.assertAlmostEqual(row['stages']['physics']['wall']['sum_s'], .002)
        self.assertAlmostEqual(row['stages']['physics']['thread_cpu']['sum_s'], .001)
        self.assertEqual(row['stages']['pacing']['thread_cpu']['sum_s'], 0.)
        self.assertAlmostEqual(row['unattributed_wall_s'], 0.)

    def test_transition_windows_do_not_contaminate_navigation(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            directory = root/'evaluations'
            directory.mkdir()
            (directory/'mission.json').write_text(json.dumps({'passed': True, 'states': [
                {'wall_time': 0., 'state': 'NAVIGATE_TO_TARGET'},
                {'wall_time': 10., 'state': 'WORKING'}]}))
            rows = []
            for start, end, physics in ((1., 6., 2.), (8., 13., 4.)):
                rows.append({'wall_start': start, 'wall_end': end, 'wall_s': 5.,
                    'simulation_s': 2.5, 'loops': 1250, 'compute_overrun_loops': 10,
                    'requested_sleep_s': 0., 'unattributed_wall_s': 0.,
                    'stages': {'physics_step': {'wall': {'sum_s': physics, 'count': 1250, 'max_s': .004},
                        'thread_cpu': {'sum_s': physics/2}}}})
            (directory/'loop-profile.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            sample = {'process_sample_wall_time': 5., 'sample_interval_s': 2.,
                'task_processes': [{'pid': 10, 'role': 'isaac', 'name': 'python3',
                    'cpu_percent_one_core': 200., 'rss_mib': 100., 'thread_schedstat': []}],
                'other_cpu_total_percent_one_core': 50., 'gpu_csv': '10, 0, 100, 1000, 20'}
            (directory/'system-profile.jsonl').write_text(json.dumps(sample)+'\n')
            result = analysis.analyze(root)
            (directory/'native-profile.json').write_text(json.dumps({'wall_start': 2., 'wall_end': 4.}))
            captured = analysis.analyze(root)
        self.assertEqual(result['phases']['NAVIGATION']['windows'], 1)
        self.assertEqual(result['phases']['NAVIGATION']['stages']['physics_step']['wall_percent'], 40.)
        self.assertEqual(result['phases']['TRANSITION']['windows'], 1)
        self.assertEqual(result['navigation_cpu_by_role_percent_one_core']['isaac'], 200.)
        self.assertEqual(captured['phases']['NATIVE_CAPTURE']['windows'], 1)
        self.assertNotIn('NAVIGATION', captured['phases'])


if __name__ == '__main__':
    unittest.main()
