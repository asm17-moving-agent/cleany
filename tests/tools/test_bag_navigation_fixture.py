import importlib.util
from pathlib import Path
import math
import pytest

PATH=Path(__file__).resolve().parents[2]/'tools/validation/native_lattice_mppi/extract_bag_fixture.py'
spec=importlib.util.spec_from_file_location('bag_fixture',PATH)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_cost_conversion_never_lowers_original_cost_and_preserves_safety_classes():
    for cost in range(1,253):
        display=1+97*(cost-1)//251
        restored=module.occupancy_to_cost(display)
        assert cost <= restored <= cost+2
    assert [module.occupancy_to_cost(v) for v in [-1,0,99,100]] == [255,0,253,254]
    with pytest.raises(ValueError):module.occupancy_to_cost(101)


def test_map_to_odom_composition_uses_parent_rotation():
    assert module.compose([1,2,math.pi/2],[2,0,0]) == pytest.approx([1,4,math.pi/2])
