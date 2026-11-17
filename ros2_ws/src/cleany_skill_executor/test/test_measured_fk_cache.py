"""Measured FK reuse follows exact kinematic dependencies, not time or tolerance."""
import math

import pytest

from cleany_skill_executor.core.cartesian import CartesianPose
from cleany_skill_executor.core.measured_fk_cache import MeasuredFkCache


DESCRIPTION = '''<robot name="cache_test">
<joint name="mount" type="fixed"><parent link="base_link"/><child link="mount"/></joint>
<joint name="shoulder" type="revolute"><parent link="mount"/><child link="wrist"/>
<origin xyz="0 0 0.1"/><axis xyz="0 0 1"/></joint>
<joint name="tcp" type="fixed"><parent link="wrist"/><child link="left_grasp_tcp"/>
<origin xyz="0.1 0 0"/></joint>
<joint name="jaw" type="prismatic"><parent link="wrist"/><child link="finger"/>
<axis xyz="1 0 0"/></joint></robot>'''
POSE = CartesianPose((.1, 0., .1), (0., 0., 0., 1.))


def key(cache, *, description=DESCRIPTION, frame='base_link', link='left_grasp_tcp',
        names=('shoulder', 'jaw'), positions=(.1, .2), recovery=0):
    return cache.key(description, frame, link, names, positions, recovery)


def populated():
    cache = MeasuredFkCache()
    cache.put(key(cache), POSE)
    return cache


def test_measured_fk_cache_ignores_unrelated_jaw_and_input_order():
    cache = populated()
    assert cache.get(key(cache, names=('jaw', 'shoulder'), positions=(.4, .1))) == POSE


@pytest.mark.parametrize('changed', [
    {'positions': (math.nextafter(.1, math.inf), .2)},
    {'description': DESCRIPTION.replace('0 0 0.1', '0 0 0.2')},
    {'frame': 'mount'}, {'link': 'finger'}, {'recovery': 1},
])
def test_measured_fk_cache_invalidates_every_geometric_or_recovery_change(changed):
    cache = populated()
    assert cache.get(key(cache, **changed)) is None


def test_measured_fk_cache_tracks_jaw_when_requested_link_depends_on_it():
    cache = MeasuredFkCache()
    cache.put(key(cache, link='finger'), POSE)
    assert cache.get(key(cache, link='finger', positions=(.1, .3))) is None


@pytest.mark.parametrize('changed', [
    {'names': ('jaw',), 'positions': (.2,)},
    {'names': ('shoulder', 'shoulder'), 'positions': (.1, .1)},
    {'positions': (.1,)}, {'positions': (math.nan, .2)},
    {'description': ''}, {'link': 'unknown'},
    {'description': DESCRIPTION.replace('<axis xyz="0 0 1"/>', '<mimic joint="jaw"/>')},
])
def test_measured_fk_cache_unsupported_or_incomplete_input_bypasses_reuse(changed):
    cache = populated()
    assert key(cache, **changed) is None
    assert cache.get(None) is None
    assert cache.get(key(cache)) is None


def test_measured_fk_cache_does_not_install_a_result_for_a_superseded_key():
    cache = MeasuredFkCache()
    old_key = key(cache)
    current = key(cache, recovery=1)
    cache.put(old_key, POSE)
    assert cache.get(current) is None
