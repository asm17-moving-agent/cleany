"""Geometry contracts for the optional holonomic primitive asset."""
from pathlib import Path
import importlib.util
import json
import math

ROOT=Path(__file__).resolve().parents[2]
SPEC=importlib.util.spec_from_file_location('symmetric_translation',ROOT/'tools/validation/native_lattice_mppi/symmetric_translation.py')
MODULE=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
SOURCE=json.loads((ROOT/'ros2_ws/src/cleany_navigation/config/lattice/omni-r10cm-balanced.json').read_text())
CANDIDATE=MODULE.build(SOURCE)


def test_asset_is_reproducible_and_keeps_heading_and_grid_contract():
    saved=json.loads((ROOT/'ros2_ws/src/cleany_navigation/config/lattice/omni-r10cm-symmetric-translation.json').read_text())
    assert saved==CANDIDATE
    assert saved['lattice_metadata']['heading_angles']==SOURCE['lattice_metadata']['heading_angles']
    assert saved['lattice_metadata']['number_of_trajectories']==224
    assert [p['trajectory_id'] for p in saved['primitives']]==list(range(224))
    for p in saved['primitives']:
        x,y,yaw=p['poses'][-1]
        assert abs(x/.05-round(x/.05))<1e-8
        assert abs(y/.05-round(y/.05))<1e-8
        assert abs(MODULE.wrap(yaw-saved['lattice_metadata']['heading_angles'][p['end_angle_index']]))<1e-8


def test_all_eight_body_directions_have_equal_reflected_lengths_and_opposites():
    for idx,yaw in enumerate(SOURCE['lattice_metadata']['heading_angles']):
        moves=[p for p in CANDIDATE['primitives'] if p['start_angle_index']==idx and p['end_angle_index']==idx and p['trajectory_length']>0]
        assert len(moves)==8
        by_direction={}
        for p in moves:
            x,y,_=p['poses'][-1]
            angle=MODULE.wrap(math.atan2(y,x)-yaw)
            direction=round(angle/(math.pi/4))%8
            assert abs(MODULE.wrap(angle-direction*math.pi/4))<1e-8
            by_direction[direction]=p
            assert all(abs(MODULE.wrap(a[2]-yaw))<1e-8 for a in p['poses'])
            points=[[0.,0.,yaw],*p['poses']]
            assert math.isclose(sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:])),p['trajectory_length'])
            assert max(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:]))<=.025000001
        assert len(by_direction)==8
        for k,p in by_direction.items():
            assert math.isclose(p['trajectory_length'],by_direction[(-k)%8]['trajectory_length'])
            assert math.isclose(p['trajectory_length'],by_direction[(k+4)%8]['trajectory_length'])


def test_curve_and_in_place_rotation_geometry_is_unchanged():
    def turns(doc):
        return [{k:v for k,v in p.items() if k!='trajectory_id'} for p in doc['primitives'] if p['start_angle_index']!=p['end_angle_index']]
    assert turns(SOURCE)==turns(CANDIDATE)
    assert len(turns(CANDIDATE))==96


def test_audit_distinguishes_cardinal_symmetry_from_oblique_heading_gaps():
    rows=MODULE.audit(SOURCE)['headings']
    assert all(p['right_ids'] for p in rows[0]['left_turns'])
    assert not any(p['right_ids'] for p in rows[1]['left_turns'])
    a,b=sorted(abs(v) for v in rows[1]['rotation_changes_deg'])
    assert math.isclose(a,18.43494882292201)
    assert math.isclose(b,26.56505117707799)
