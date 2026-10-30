"""Fixed-finger clearance independent of ROS, IK and physical motors."""
from pathlib import Path
import math

import pytest

from cleany_skill_executor.core.fixed_jaw_clearance import (
    Bounds, ClearanceConfig, TargetBox, closing_clearance, correction_for_clearance,
)
from cleany_skill_executor.core.physical_grasp_selection import PoseValue
from cleany_skill_executor.fixed_jaw_geometry import load_fixed_jaw_bounds


JAW = Bounds((.008, -.006, -.005), (.016, .06, .005))
IDENTITY = (0., 0., 0., 1.)


def box(center=(0., 0., 0.), size=(.04, .03, .04), orientation=IDENTITY):
    return TargetBox(PoseValue(center, orientation), size)


def sweep(x):
    return [PoseValue((x, .08, 0.), IDENTITY), PoseValue((x, 0., 0.), IDENTITY)]


def jaw_description(arm='right', tcp_x=0., tcp_y=-.1, mesh_origin_x=0., scale='1 1 1'):
    return f'''<robot name="clearance_test">
      <link name="{arm}_gripper_frame"><collision>
        <origin xyz="{mesh_origin_x} 0 0"/><geometry>
        <mesh filename="package://cleany_description/meshes/Fixed_Jaw_part2.ply.convex.stl" scale="{scale}"/>
        </geometry></collision></link><link name="{arm}_grasp_tcp"/>
      <joint name="tcp" type="fixed"><parent link="{arm}_gripper_frame"/>
      <child link="{arm}_grasp_tcp"/><origin xyz="{tcp_x} {tcp_y} 0"/></joint></robot>'''


def mesh_path(_):
    return Path(__file__).resolve().parents[2] / 'cleany_description/meshes/Fixed_Jaw_part2.ply.convex.stl'


def test_existing_asymmetric_bias_is_credited_before_any_extra_correction():
    # 40mm opening, 8mm fixed-jaw offset and existing +10mm bias.
    gap = closing_clearance(JAW, box().corners(), sweep(.02 - .008 + .010))
    assert gap == pytest.approx(.010)
    assert correction_for_clearance(gap, ClearanceConfig(.005, .010)) == 0.


def test_only_the_remaining_clearance_deficit_moves_both_endpoints():
    corners = box().corners()
    poses = sweep(.014)
    gap = closing_clearance(JAW, corners, poses)
    assert gap == pytest.approx(.002)
    extra = correction_for_clearance(gap, ClearanceConfig(.005, .010))
    assert extra == pytest.approx(.003)
    assert closing_clearance(JAW, corners, sweep(.014 + extra)) == pytest.approx(.005)


def test_swept_approach_detects_contact_even_when_the_grasp_endpoint_misses():
    corners = box(center=(0., -.03, 0.), size=(.04, .01, .02)).corners()
    assert closing_clearance(JAW, corners, sweep(.01)[-1:]) is None
    # Here the object is farther along the approach; move it behind the final
    # tool to make the fixed finger pass it during entry.
    corners = box(center=(0., .09, 0.), size=(.04, .01, .02)).corners()
    assert closing_clearance(JAW, corners, sweep(.01)[-1:]) is None
    assert closing_clearance(JAW, corners, sweep(.01)) < 0


@pytest.mark.parametrize('center', [(0., 0., .2), (.2, 0., 0.), (-.2, 0., 0.)])
def test_no_aperture_containment_requirement_when_fixed_finger_is_clear(center):
    gap = closing_clearance(JAW, box(center=center).corners(), sweep(.02))
    assert correction_for_clearance(gap, ClearanceConfig(.005, .010)) == 0.


def test_rotated_target_width_is_projected_in_the_tool_frame():
    rotated = box(size=(.02, .06, .04), orientation=(0., 0., math.sqrt(.5), math.sqrt(.5)))
    gap = closing_clearance(JAW, rotated.corners(), sweep(.025))
    assert gap == pytest.approx(.003)
    assert correction_for_clearance(gap, ClearanceConfig(.005, .010)) == pytest.approx(.002)


def test_correction_limit_does_not_silently_clamp_an_unsafe_target():
    with pytest.raises(ValueError, match='limit=10.000mm'):
        correction_for_clearance(-.006, ClearanceConfig(.005, .010))


def test_negative_fixed_finger_side_uses_the_same_separation_contract():
    jaw = Bounds((-.016, -.006, -.005), (-.008, .06, .005))
    assert closing_clearance(jaw, box().corners(), sweep(-.014), -1) == pytest.approx(.002)
    assert closing_clearance(jaw, box().corners(), sweep(-.017), -1) == pytest.approx(.005)


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_mesh_bounds_are_loaded_from_the_runtime_tcp_for_each_arm(arm):
    bounds = load_fixed_jaw_bounds(jaw_description(arm), arm, ['Fixed_Jaw_part2.ply.convex.stl'], mesh_path)
    assert bounds.minimum == pytest.approx((.00786306895, -.00638794303, -.01388974860))
    assert bounds.maximum == pytest.approx((.03708603978, .06106966585, .01379468013))
    calibrated = load_fixed_jaw_bounds(jaw_description(arm, tcp_x=.004, mesh_origin_x=.002),
                                     arm, ['Fixed_Jaw_part2.ply.convex.stl'], mesh_path)
    assert calibrated.minimum[0] == pytest.approx(bounds.minimum[0] - .002)


@pytest.mark.parametrize('description,meshes', [
    (jaw_description(), ['missing.stl']),
    (jaw_description(scale='1 nan 1'), ['Fixed_Jaw_part2.ply.convex.stl']),
    (jaw_description().replace('type="fixed"', 'type="revolute"'), ['Fixed_Jaw_part2.ply.convex.stl']),
])
def test_missing_geometry_or_invalid_runtime_transforms_fail_closed(description, meshes):
    with pytest.raises(ValueError):
        load_fixed_jaw_bounds(description, 'right', meshes, mesh_path)


@pytest.mark.parametrize('minimum,maximum', [(-.001, .01), (.005, float('nan')), (.005, -.01)])
def test_invalid_clearance_limits_are_rejected(minimum, maximum):
    with pytest.raises(ValueError):
        ClearanceConfig(minimum, maximum)


def test_nonfinite_clearance_cannot_silently_skip_correction():
    with pytest.raises(ValueError, match='Nonfinite'):
        correction_for_clearance(float('nan'), ClearanceConfig(.005, .010))
