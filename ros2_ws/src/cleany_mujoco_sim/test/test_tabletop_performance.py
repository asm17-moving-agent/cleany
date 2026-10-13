from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from cleany_mujoco_sim import scene_loader
from cleany_mujoco_sim.tabletop_performance import (
    apply_tabletop_performance, load_performance_profile, TabletopPerformance,
)

PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / 'config/tabletop_performance.yaml'


def test_baseline_is_exact_noop():
    assert load_performance_profile(Path('/not/required'), 'baseline') is None
    assert apply_tabletop_performance('scene', 'robot', None) == ('scene', 'robot', ())


@pytest.mark.parametrize('name,size', [('tabletop_collision', 4096), ('tabletop_fast', 2048)])
def test_profile_config(name, size):
    assert load_performance_profile(CONFIG, name) == TabletopPerformance(2.0, size)


@pytest.mark.parametrize('radius,shadow', [(.5, 2048), (float('nan'), 2048), (True, 2048), (2, 32), (2, True)])
def test_unsafe_config_is_rejected(tmp_path, radius, shadow):
    import yaml
    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump({'schema_version': 1, 'profiles': {
        'tabletop_fast': {'collision_keep_radius_m': radius, 'shadow_map_size': shadow}}}))
    with pytest.raises(ValueError):
        load_performance_profile(path, 'tabletop_fast')


def test_unknown_profile_is_rejected():
    with pytest.raises(ValueError, match='Unknown'):
        load_performance_profile(CONFIG, 'fast_typo')


def test_prebuilt_scene_cannot_silently_ignore_profile():
    with pytest.raises(ValueError, match='materialized'):
        scene_loader.resolve_control_scene_path(Path('/scene.xml'), performance_profile='tabletop_fast')


@pytest.fixture(scope='module')
def scenes():
    # Source config is used even before installing the newly added YAML file.
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(scene_loader, '_package_share', lambda package: PACKAGE.parent / package)
        paths = {profile: scene_loader.materialize_control_scene(
            PACKAGE / 'scenes/study_cafe_grasp_execution.xml.in',
            sorting_bins_config=PACKAGE / 'config/robot_top_bins.yaml',
            performance_profile=profile,
        ) for profile in ('baseline', 'tabletop_collision', 'tabletop_fast')}
    return paths


def test_only_distant_static_masks_and_shadow_quality_change(scenes):
    roots = {name: ET.parse(path).getroot() for name, path in scenes.items()}
    original = roots['baseline']
    for name in ('tabletop_collision', 'tabletop_fast'):
        fast = roots[name]
        original_robot = ET.parse(original.find('include').get('file'))
        fast_robot = ET.parse(fast.find('include').get('file'))
        assert ET.tostring(original_robot.getroot()) == ET.tostring(fast_robot.getroot())
        disabled = []
        for before, after in zip(original.iter(), fast.iter(), strict=True):
            if before.tag == 'include':
                continue  # Only the temporary include path differs.
            if before.tag == 'quality':
                assert after.get('shadowsize') == ('4096' if name == 'tabletop_collision' else '2048')
                after.set('shadowsize', before.get('shadowsize'))
            if before.tag == 'geom' and before.attrib != after.attrib:
                assert after.get('contype') == after.get('conaffinity') == '0'
                assert before.get('name').startswith(('demo_desk_', 'desk_monitor_', 'desk_partition_', 'wall_'))
                disabled.append(before.get('name'))
                after.set('contype', before.get('contype'))
                after.set('conaffinity', before.get('conaffinity'))
            assert before.tag == after.tag and before.attrib == after.attrib
        assert len(disabled) > 100
        assert not any(item.startswith('demo_desk_43__') for item in disabled)


def test_compiled_dynamics_and_near_contacts_are_unchanged(scenes):
    models = [mujoco.MjModel.from_xml_path(str(scenes[name])) for name in ('baseline', 'tabletop_fast')]
    baseline, fast = models
    assert baseline.ngeom == fast.ngeom and baseline.npair == fast.npair
    for field in ('geom_size', 'geom_friction', 'geom_condim', 'body_mass', 'body_inertia',
                  'actuator_ctrlrange', 'actuator_forcerange', 'pair_friction', 'pair_solref', 'pair_solimp'):
        np.testing.assert_array_equal(getattr(baseline, field), getattr(fast, field))
    assert baseline.opt.timestep == fast.opt.timestep == .001
    assert baseline.opt.iterations == fast.opt.iterations == 100
    assert baseline.opt.noslip_iterations == fast.opt.noslip_iterations == 20
    data = [mujoco.MjData(model) for model in models]
    for model, state in zip(models, data):
        mujoco.mj_resetDataKeyframe(model, state, model.key('handeye_ros2_control_home').id)
        mujoco.mj_step(model, state, nstep=500)
    np.testing.assert_allclose(data[0].qpos, data[1].qpos, atol=1e-10, rtol=0)
    assert data[0].ncon == data[1].ncon


def test_profile_rejects_mobile_scene(scenes):
    scene = ET.parse(scenes['baseline']).getroot()
    scene.remove(scene.find('equality'))
    model = Path(scene.find('include').get('file')).read_text()
    with pytest.raises(ValueError, match='fixed-base'):
        apply_tabletop_performance(ET.tostring(scene, encoding='unicode'), model,
                                  load_performance_profile(CONFIG, 'tabletop_fast'))


def test_bounds_keep_near_and_large_bodies_and_explicit_pairs(scenes):
    root = ET.parse(scenes['baseline']).getroot()
    model = Path(root.find('include').get('file')).read_text()
    robot = ET.fromstring(model)
    origin = [float(x) for x in robot.find("./worldbody/body[@name='chassis']").get('pos').split()]
    for suffix, x, size in [('near', .5, '.1 .1 .1'), ('large', 3, '4 4 4'), ('far', 5, '.1 .1 .1'), ('paired', 5, '.1 .1 .1')]:
        body = ET.SubElement(root.find('worldbody'), 'body', name=f'demo_desk_{suffix}',
                             pos=f'{origin[0]+x} {origin[1]} 0')
        ET.SubElement(body, 'geom', name=f'probe_{suffix}', type='box', size=size, contype='1', conaffinity='1')
    ET.SubElement(root.find('contact'), 'pair', geom1='probe_paired', geom2='floor')
    _, _, disabled = apply_tabletop_performance(ET.tostring(root, encoding='unicode'), model,
                                                load_performance_profile(CONFIG, 'tabletop_fast'))
    assert 'probe_far' in disabled
    assert not {'probe_near', 'probe_large', 'probe_paired'}.intersection(disabled)
