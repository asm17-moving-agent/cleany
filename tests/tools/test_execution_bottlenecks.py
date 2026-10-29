"""Historical accounting must not count nested verification or tracking twice."""
import importlib.util
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location('execution_bottlenecks',
    Path(__file__).resolve().parents[2]/'tools/validation/analyze_execution_bottlenecks.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_component_budget_excludes_nested_model_and_duplicate_tracking():
    events = [dict(data=dict(stage='synchronized_return_trajectory_prepared',
                            preparation_wall_sec=2., model_verification_wall_sec=.1, duration_sec=5.)),
              dict(data=dict(stage='synchronized_return_tracking', duration_sec=5.)),
              dict(data=dict(stage='return_planned', planning_wall_sec=.5,
                             direct_validation_wall_sec=.4))]
    assert module.components(events, 10.) == dict(
        planning=.5, preparation=2., sensor=0., command=5., unseparated=2.5)
    with pytest.raises(ValueError, match='exceeds'):
        module.components(events, 1.)


def test_selector_parent_timer_is_excluded(tmp_path):
    log = tmp_path/'hardware-support.log'
    log.write_text('[INFO] [100.1] stage=GRASP_IK: phase=GPU_IK state=done wall_sec=0.1\n'
                   '[INFO] [100.2] stage=GRASP_IK: phase=CANDIDATE_IK state=done wall_sec=0.2\n')
    events = module.read_events(log)
    assert len(events) == 1 and events[0]['data']['phase'] == 'GPU_IK'
