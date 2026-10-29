"""Count retained Nav2 log diagnostics without conflating normal planning with recovery."""
from __future__ import annotations

import re


FAILURE_COUNTS = (
    'planner_failures', 'controller_failures', 'path_progress_timeouts',
    'recovery_behaviors', 'costmap_clears', 'continuous_footprint_refinement_failures',
)


def summarize_navigation_log(log: str, *, require_no_recovery: bool) -> dict:
    patterns = {
        'planner_failures': r'failed to create plan|Planning algorithm [^\n]* failed to generate a valid path',
        'controller_failures': r'Optimizer fail to compute path|MPPI: no collision-free emitted rollout|'
                               r'Controller patience exceeded|Failed to make progress',
        'path_progress_timeouts': r'Path progress timeout:',
        'recovery_behaviors': r'Running (?:spin|backup|wait|drive_on_heading)',
        'costmap_clears': r'Received request to clear',
        'validated_standstill_cycles': r'MPPI validated standstill:',
        'continuous_footprint_refinements': r'Continuous-footprint lattice refinement: [1-9][0-9]* poses',
        'continuous_footprint_refinement_failures': r'Continuous-footprint lattice refinement: 0 poses',
    }
    result = {name: len(re.findall(pattern, log, re.IGNORECASE)) for name, pattern in patterns.items()}
    result['required_no_recovery'] = require_no_recovery
    return result
