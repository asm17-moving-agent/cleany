from pathlib import Path
import json
import numpy as np
from PIL import Image
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]

@pytest.mark.parametrize('backend', ['physical', 'mock'])
@pytest.mark.parametrize('profile, standoff, height', [
    ('canonical', .55, .0684), ('slender-cup-95mm', .55, .095)])
def test_cleanup_avoids_blocked_front_and_preserves_chair_geometry(
        tmp_path, monkeypatch, profile, standoff, height, backend):
    monkeypatch.syspath_prepend(str(ROOT/'tools/validation/isaac'))
    from prepare_cleanup import prepare_cleanup
    (tmp_path/'config').mkdir(); (tmp_path/'assets').mkdir()
    room=dict(world='facility18',map_file='assets/map.yaml',target=[0.,0.,1.57079632679],target_id='seat-test',park_joints={})
    (tmp_path/'config/room.json').write_text(json.dumps(room))
    geometry=[dict(kind='collision',name='desk_envelope_collision',model='desk',
        transform=[[1.,0.,0.,0.],[0.,1.,0.,1.5],[0.,0.,1.,.36],[0.,0.,0.,1.]],shape=dict(type='box',size=[1.2,.77,.72]))]
    (tmp_path/'config/world_geometry.json').write_text(json.dumps(geometry))
    (tmp_path/'config/targets.yaml').write_text(yaml.safe_dump(dict(seats={'seat-test':room['target']})))
    grid=np.full((200,200),254,np.uint8)
    # Front/back edge approaches blocked; side approach has 0.55m table clearance.
    for y in np.arange(.25,2.5,.025):
        for x in np.arange(-.6,.625,.025): grid[199-int((y+2)/.025),int((x+2.5)/.025)]=0
    Image.fromarray(grid).save(tmp_path/'assets/map.pgm')
    (tmp_path/'assets/map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.025,origin=[-2.5,-2.,0.])))
    result=prepare_cleanup(tmp_path,ROOT,at_desk=True,profile=profile,backend=backend)
    assert result['cleanup_backend'] == backend
    if backend == 'mock':
        assert result['target'] == room['target']
        assert result['cleanup']['navigation_target_source'] == 'existing seat target'
        assert result['spawn'][:2] == room['target'][:2]
    else:
        assert abs(result['target'][0]) == pytest.approx(.6+standoff)
    assert abs(result['cleanup']['physical_approach_target'][0]) == pytest.approx(.6+standoff)
    assert result['cleanup']['approach_map_clearance_m'] >= .5
    assert result['cleanup']['original_navigation_target'] == room['target']
    assert json.loads((tmp_path/'config/world_geometry.json').read_text()) == geometry
    cup=result['cleanup']['payload_position']
    assert result['cleanup']['payload']['collision']['size_m'][-1] == pytest.approx(height)
    # Variant height changes the center, never the support surface/bottom gap.
    assert cup[2]-height/2 == pytest.approx(.722)
    if profile != 'canonical':
        assert result['physics_hz'] == 1000
        assert result['cleanup']['navigation_parameters']['global_costmap'][
            'global_costmap']['ros__parameters']['footprint_padding'] == .04
        assert result['cleanup']['navigation_parameters']['local_costmap'][
            'local_costmap']['ros__parameters']['footprint_padding'] == .04
        follow = result['cleanup']['navigation_parameters']['controller_server']['ros__parameters']['FollowPath']
        assert follow['PathAlignCritic']['threshold_to_consider'] == .1
        tolerance = result['cleanup']['navigation_parameters']['controller_server'][
            'ros__parameters']['goal_checker']['xy_goal_tolerance']
        assert 0. < follow['goal_rotate_enter_radius'] <= follow['goal_rotate_exit_radius'] <= tolerance <= .03
        assert 'vx_max' not in follow and 'CostCritic' not in follow
        assert (tmp_path/'config/coordinator.yaml').is_file()
        assert result['cleanup']['fixture_variant']['baseline_height_m'] == pytest.approx(.0684)
    assert abs(cup[0]) < .6 and 1.115 < cup[1] < 1.885
    radius=result['cleanup']['payload']['collision']['size_m'][0]/2
    assert abs(cup[0])+radius+.015 < .6
    assert 1.115+radius < cup[1] < 1.885-radius
    Image.fromarray(np.zeros_like(grid)).save(tmp_path/'assets/map.pgm')
    (tmp_path/'config/room.json').write_text(json.dumps(room))
    with pytest.raises(ValueError,match='No desk edge'):
        prepare_cleanup(tmp_path,ROOT,profile=profile)


def test_profile_rejects_unsupported_object_before_writing(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT/'tools/validation/isaac'))
    from prepare_cleanup import prepare_cleanup
    with pytest.raises(ValueError, match='requires its declared object'):
        prepare_cleanup(tmp_path, ROOT, object_name='tissue', profile='slender-cup-95mm')
    assert not list(tmp_path.iterdir())


def test_cleanup_rejects_chair_even_when_map_center_has_clearance(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT/'tools/validation/isaac'))
    from prepare_cleanup import prepare_cleanup
    (tmp_path/'config').mkdir()
    (tmp_path/'assets').mkdir()
    room = dict(world='facility18', map_file='assets/map.yaml', target=[0., 0., np.pi/2],
                target_id='seat-test', park_joints={})
    (tmp_path/'config/room.json').write_text(json.dumps(room))
    (tmp_path/'config/targets.yaml').write_text(yaml.safe_dump(dict(seats={'seat-test':room['target']})))
    geometry = [dict(kind='collision', name='desk_envelope_collision', model='desk',
        transform=[[1.,0.,0.,0.],[0.,1.,0.,1.5],[0.,0.,1.,.36],[0.,0.,0.,1.]],
        shape=dict(type='box',size=[1.2,.77,.72])),
        dict(kind='collision', name='back', model='office_chair_test',
        transform=[[1.,0.,0.,0.],[0.,1.,0.,.85],[0.,0.,1.,.7],[0.,0.,0.,1.]],
        shape=dict(type='box',size=[.44,.05,.5]))]
    original = json.dumps(geometry)
    (tmp_path/'config/world_geometry.json').write_text(original)
    # The saved map omits the chair; its desk distance alone admits the front pose.
    grid = np.full((200,200),254,np.uint8)
    for y in np.arange(1.125,1.875,.025):
        for x in np.arange(-.6,.6,.025):
            grid[199-int((y+2)/.025),int((x+2.5)/.025)] = 0
    Image.fromarray(grid).save(tmp_path/'assets/map.pgm')
    (tmp_path/'assets/map.yaml').write_text(yaml.safe_dump(
        dict(image='map.pgm',resolution=.025,origin=[-2.5,-2.,0.])))
    result = prepare_cleanup(tmp_path,ROOT,profile='slender-cup-95mm',backend='physical')
    assert abs(result['target'][0]) == pytest.approx(1.15)
    rejected = result['cleanup']['approach_chair_check']['rejected']
    assert rejected[0]['point'] == pytest.approx([0.,.565])
    assert rejected[0]['chairs'] == ['office_chair_test']
    assert (tmp_path/'config/world_geometry.json').read_text() == original
