"""The preview resolves fingers while preserving measured actuator positions."""
from pathlib import Path
import pytest
from cleany_description.runtime_state import initial_positions, resolve_positions


def description():
    return (Path(__file__).parents[1]/'urdf/cleany.urdf').read_text()


def test_nominal_and_measured_preview_resolve_all_passive_fingers():
    xml = description()
    initial = initial_positions(xml)
    assert len(initial) == 14
    for fraction in (0., .25, .5, .75, 1.):
        state = dict(initial)
        for side in ('left', 'right'):
            state[side+'_gripper_joint'] = 2.4173120563240813-fraction*(2.4173120563240813+.3745329776277857)
        resolved = resolve_positions(xml, state)
        assert len(resolved) == 30
        assert all(resolved[name] == value for name, value in state.items())


def test_partial_or_nonfinite_feedback_is_not_presented_as_measured():
    xml = description()
    with pytest.raises(ValueError, match='fourteen'):
        resolve_positions(xml, {'left_gripper_joint': 0.})
    state = initial_positions(xml)
    state['right_gripper_joint'] = float('nan')
    with pytest.raises(ValueError, match='finite'):
        resolve_positions(xml, state)
