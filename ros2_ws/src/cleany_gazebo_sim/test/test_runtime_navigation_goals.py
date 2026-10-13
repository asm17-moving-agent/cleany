"""Opt-in regressions of the two observed study-cafe navigation goals."""
from pathlib import Path
import pytest


@pytest.mark.parametrize('case', ['distant', 'chair_gap'])
def test_saved_map_navigation_goal(case, runtime_test_options, request, monkeypatch):
    if runtime_test_options.profile != 'fortress':
        pytest.skip('These study-cafe goal regressions are validated on Humble/Fortress')
    root = Path(__file__).resolve().parents[4]
    monkeypatch.syspath_prepend(str(root / 'tools/validation/navigation'))
    from run_goal_regression import run_goal
    output = Path(request.config.getoption('--navigation-output')).resolve() / case
    result = run_goal(output, case, runtime_test_options.profile,
                      request.config.getoption('--navigation-rtf'),
                      domain=210 if case == 'distant' else 211)
    assert result['passed'], f'{result["checks"]}; evidence: {output}'
