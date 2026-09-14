import xml.etree.ElementTree as ET

import pytest

from cleany_moveit_config.simulation_collision import ignore_simulation_mast


URDF = '<robot><link name="base_link"/><link name="top_base_link"/><link name="arm"/><link name="jaw"/></robot>'
SRDF = '<robot><disable_collisions link1="arm" link2="jaw" reason="Adjacent"/></robot>'


def test_mast_override_is_simulation_only_and_idempotent():
    result = ignore_simulation_mast(URDF, SRDF, use_sim_time=True)
    assert ignore_simulation_mast(URDF, result, use_sim_time=True) == result
    entries = ET.fromstring(result).findall('disable_collisions')
    assert len(entries) == 4
    added = [entry for entry in entries if entry.get('reason') == 'SimulationOnlyMastOverride']
    assert len(added) == 3
    assert all(entry.get('link1') == 'top_base_link' for entry in added)
    assert {entry.get('link2') for entry in added} == {'base_link', 'arm', 'jaw'}
    assert len(ET.fromstring(SRDF).findall('disable_collisions')) == 1


def test_mast_override_rejects_real_time_and_missing_mast():
    with pytest.raises(ValueError, match='simulation time'):
        ignore_simulation_mast(URDF, SRDF, use_sim_time=False)
    with pytest.raises(ValueError, match='top_base_link'):
        ignore_simulation_mast('<robot/>', SRDF, use_sim_time=True)
