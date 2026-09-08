from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace

import yaml

from cleany_interfaces.srv import VerifyPlacement
from cleany_mujoco_sim.placement_verifier import DEFAULT_LABEL_BODIES, PlacementVerifier
from cleany_mujoco_sim.sorting_scene import load_bins
from cleany_mujoco_sim.study_cafe_scene import load_study_cafe_layout


def test_default_oracle_labels_match_learned_profile_policy_and_scene():
    source = Path(__file__).parents[2]
    perception = source / 'cleany_perception' / 'config'
    for name in ('yoloe_s_sam2_tiny.yaml', 'inspect_scene.yaml'):
        parameters = yaml.safe_load((perception / name).read_text())['perception_inspector']['ros__parameters']
        assert parameters['yoloe_classes'] == list(DEFAULT_LABEL_BODIES)
    layout = load_study_cafe_layout(
        source / 'cleany_mujoco_sim' / 'config' / 'study_cafe_layout.yaml')
    assert set(DEFAULT_LABEL_BODIES.values()) == {
        f'study_cafe_{item.name}' for item in layout.tabletop_objects
    }
    policy = yaml.safe_load((source / 'cleany_skill_executor' / 'config' / 'sorting_policy.yaml').read_text())
    assert {'cup', 'crumpled tissue'} <= set(policy['rules']['trash'])
    assert {'wallet', 'lego brick'} <= set(policy['rules']['lost_item'])


def verifier(*, destination='trash_right', z=0.22, size=(0.04, 0.04, 0.04),
             after_stamp=500_000_000, now=1_450_000_000, drift=0.0):
    bins = {b.name: b for b in load_bins(
        Path(__file__).parents[1] / 'config' / 'robot_top_bins.yaml')}
    bin_ = bins[destination]
    parameters = dict(minimum_samples=5, maximum_age_sec=1.0,
                      maximum_drift_m=0.003, settled_duration_sec=0.4)
    node = SimpleNamespace(
        _bins=bins, _bodies={'cup': 'study_cafe_cup'},
        _samples=defaultdict(list, study_cafe_cup=[
            (1_000_000_000 + i * 100_000_000,
             (bin_.center_xy[0] + i * drift, bin_.center_xy[1], z), size)
            for i in range(5)
        ]),
        get_parameter=lambda key: SimpleNamespace(value=parameters[key]),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=now)),
    )
    return PlacementVerifier._verify(node, VerifyPlacement.Request(
        label='cup', destination_id='trash_right', after_stamp_ns=after_stamp,
    ), VerifyPlacement.Response())


def test_settled_object_inside_correct_bin_is_verified():
    assert verifier().success


def test_wrong_bin_and_rim_hover_are_not_verified():
    assert not verifier(destination='lost_items_left').success
    assert not verifier(z=0.30).success
    assert not verifier(size=(0.30, 0.04, 0.04)).success


def test_stale_pre_release_or_moving_samples_are_not_verified():
    assert not verifier(now=4_000_000_000).success
    assert not verifier(after_stamp=1_100_000_000).success
    assert not verifier(drift=0.001).success
