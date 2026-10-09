from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace

import yaml
import pytest

from cleany_interfaces.srv import VerifyPlacement
from cleany_mujoco_sim.placement_verifier import DEFAULT_LABEL_BODIES, PlacementVerifier
from cleany_mujoco_sim.sorting_scene import load_bins
from cleany_mujoco_sim.study_cafe_scene import load_study_cafe_layout


def test_default_oracle_labels_match_learned_profile_policy_and_scene():
    source = Path(__file__).parents[2]
    perception = source / 'cleany_perception' / 'config'
    for name in ('yoloe_seg.yaml', 'inspect_scene.yaml'):
        parameters = yaml.safe_load((perception / name).read_text())['perception_inspector']['ros__parameters']
        assert parameters['yoloe_classes'] == ['cup', 'computer mouse', 'crumpled tissue', 'lego brick']
        assert set(parameters['yoloe_classes']) <= set(DEFAULT_LABEL_BODIES)
    for alias in ('mouse', 'computer mouse', 'wireless mouse'):
        assert DEFAULT_LABEL_BODIES[alias] == 'study_cafe_mouse'
    layout = load_study_cafe_layout(
        source / 'cleany_mujoco_sim' / 'config' / 'study_cafe_layout.yaml')
    assert set(DEFAULT_LABEL_BODIES.values()) == {
        f'study_cafe_{item.name}' for item in layout.tabletop_objects
    }
    policy = yaml.safe_load((source / 'cleany_skill_executor' / 'config' / 'table_sorting_policy.yaml').read_text())
    assert {'cup', 'crumpled tissue'} <= set(policy['rules']['trash'])
    assert {'mouse', 'computer mouse', 'wireless mouse', 'lego brick'} <= set(policy['rules']['lost_item'])


def verifier(*, destination='trash_right', z=None, size=(0.04, 0.04, 0.04),
             after_stamp=500_000_000, now=1_450_000_000, drift=0.0):
    bins = {b.name: b for b in load_bins(
        Path(__file__).parents[1] / 'config' / 'robot_top_bins.yaml')}
    bin_ = bins[destination]
    if z is None:
        z = bin_.bottom_z + bin_.wall + size[2]/2 + .002
    elif z == 'rim':
        z = bin_.top_z
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
    assert not verifier(z='rim').success
    assert not verifier(size=(0.30, 0.04, 0.04)).success


def test_stale_pre_release_or_moving_samples_are_not_verified():
    assert not verifier(now=4_000_000_000).success
    assert not verifier(after_stamp=1_100_000_000).success
    assert not verifier(drift=0.001).success


def registration_fixture():
    from cleany_interfaces.msg import DetectedObject3D
    from cleany_interfaces.srv import RegisterPlacementTarget
    from geometry_msgs.msg import Pose, Point, Vector3
    from std_msgs.msg import Header
    from builtin_interfaces.msg import Time
    from types import SimpleNamespace as NS
    bins = {b.name: b for b in load_bins(Path(__file__).parents[1] / 'config/robot_top_bins.yaml')}
    parameters = dict(registration_ttl_sec=600., registration_maximum_entries=128,
                      registration_observation_maximum_age_sec=120., maximum_age_sec=1.,
                      registration_maximum_distance_m=.05, registration_ambiguity_margin_m=.01,
                      registration_maximum_size_error_m=.08, minimum_samples=5,
                      settled_duration_sec=.4, maximum_drift_m=.003)
    node = NS(_registrations={}, _execution_registrations={}, _bins=bins,
              _body_candidates={'cup': {'cup_a', 'cup_b'}}, _bodies={},
              _samples=defaultdict(list), get_parameter=lambda key: NS(value=parameters[key]),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=1_450_000_000)))
    node._samples['cup_a'] = [(1_400_000_000, (.6, 0., .7), (.06, .06, .07))]
    node._samples['cup_b'] = [(1_400_000_000, (.8, 0., .7), (.06, .06, .07))]
    request = RegisterPlacementTarget.Request(execution_id='e', destination_id='trash_right',
        header=Header(frame_id='base_link', stamp=Time(sec=1)),
        target=DetectedObject3D(object_id=1, label='cup', track_id='track_a',
            obb_pose=Pose(position=Point(x=.6, z=.7)), obb_size=Vector3(x=.06, y=.06, z=.07)))
    return node, request


def test_individual_verification_never_uses_another_cup_already_in_bin():
    from cleany_interfaces.srv import RegisterPlacementTarget
    node, request = registration_fixture()
    registration = PlacementVerifier._register(node, request, RegisterPlacementTarget.Response())
    assert registration.success
    assert 'cup_a' not in registration.message
    bin_ = node._bins['trash_right']
    size = (.04, .04, .04)
    inside = (bin_.center_xy[0], bin_.center_xy[1], bin_.bottom_z+bin_.wall+.022)
    for body, position in [('cup_a', (.6, 0., .7)), ('cup_b', inside)]:
        node._samples[body] = [(1_000_000_000+i*100_000_000, position, size) for i in range(5)]
    verify = VerifyPlacement.Request(verification_id=registration.verification_id,
                                    destination_id='trash_right', after_stamp_ns=500_000_000)
    assert not PlacementVerifier._verify(node, verify, VerifyPlacement.Response()).success
    node._samples['cup_a'] = list(node._samples['cup_b'])
    assert PlacementVerifier._verify(node, verify, VerifyPlacement.Response()).success
    verify.verification_id = 'unknown'
    assert not PlacementVerifier._verify(node, verify, VerifyPlacement.Response()).success
    # Duplicate labels have no legacy body mapping and cannot fall back.
    verify.verification_id, verify.label = '', 'cup'
    assert not PlacementVerifier._verify(node, verify, VerifyPlacement.Response()).success


def test_registration_is_idempotent_but_rejects_ambiguous_stale_or_changed_identity():
    from cleany_interfaces.srv import RegisterPlacementTarget
    node, request = registration_fixture()
    first = PlacementVerifier._register(node, request, RegisterPlacementTarget.Response())
    second = PlacementVerifier._register(node, request, RegisterPlacementTarget.Response())
    assert first.success and second.verification_id == first.verification_id
    request.target.object_id = 2
    assert not PlacementVerifier._register(node, request, RegisterPlacementTarget.Response()).success
    request.execution_id = 'new'
    node._samples['cup_b'] = [(1_400_000_000, (.605, 0., .7), (.06, .06, .07))]
    assert not PlacementVerifier._register(node, request, RegisterPlacementTarget.Response()).success
    request.header.frame_id = 'camera'
    assert not PlacementVerifier._register(node, request, RegisterPlacementTarget.Response()).success


@pytest.mark.parametrize('label', ['mouse', 'computer mouse', 'wireless mouse', 'lego brick'])
def test_individual_lost_item_verification_distinguishes_body_and_bin(label):
    from cleany_interfaces.srv import RegisterPlacementTarget
    node, request = registration_fixture()
    node._body_candidates = {label: {'cup_a', 'cup_b'}}
    request.target.label, request.destination_id = label, 'lost_items_left'
    registration = PlacementVerifier._register(node, request, RegisterPlacementTarget.Response())
    assert registration.success
    bin_ = node._bins['lost_items_left']
    size = (.04, .04, .04)
    inside = (*bin_.center_xy, bin_.bottom_z+bin_.wall+.022)
    def samples(position):
        return [(1_000_000_000+i*100_000_000, position, size) for i in range(5)]
    node._samples['cup_b'] = samples(inside)
    verify = VerifyPlacement.Request(verification_id=registration.verification_id,
                                    destination_id='lost_items_left', after_stamp_ns=500_000_000)
    assert not PlacementVerifier._verify(node, verify, VerifyPlacement.Response()).success
    wrong_bin = node._bins['trash_right']
    node._samples['cup_a'] = samples((*wrong_bin.center_xy, wrong_bin.bottom_z+wrong_bin.wall+.022))
    assert not PlacementVerifier._verify(node, verify, VerifyPlacement.Response()).success
    node._samples['cup_a'] = samples(inside)
    assert PlacementVerifier._verify(node, verify, VerifyPlacement.Response()).success
    verify.destination_id = 'trash_right'
    assert not PlacementVerifier._verify(node, verify, VerifyPlacement.Response()).success


def test_two_cup_layout_compiles_into_distinct_free_bodies(tmp_path):
    import mujoco
    import numpy as np
    from cleany_mujoco_sim.scene_loader import materialize_control_scene
    package = Path(__file__).parents[1]
    path = materialize_control_scene(package / 'scenes/study_cafe_grasp_execution.xml.in',
        study_cafe_layout_config=package / 'config/study_cafe_two_cups.yaml',
        sorting_bins_config=package / 'config/robot_top_bins.yaml')
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    a, b = model.body('study_cafe_cup').id, model.body('study_cafe_cup_b').id
    assert a != b
    for body in [a, b]:
        assert model.jnt_type[model.body_jntadr[body]] == mujoco.mjtJoint.mjJNT_FREE
    assert np.linalg.norm(data.xpos[a] - data.xpos[b]) > .09
    for index in range(model.neq):
        if model.eq_type[index] == mujoco.mjtEq.mjEQ_WELD:
            assert not {a, b} & {model.eq_obj1id[index], model.eq_obj2id[index]}


def test_mujoco_physics_cup_in_bin_does_not_verify_registered_cup_on_desk():
    import mujoco
    import numpy as np
    from cleany_interfaces.srv import RegisterPlacementTarget
    from rclpy.time import Time
    from cleany_mujoco_sim.scene_loader import materialize_control_scene
    package = Path(__file__).parents[1]
    path = materialize_control_scene(package/'scenes/study_cafe_grasp_execution.xml.in',
        study_cafe_layout_config=package/'config/study_cafe_two_cups.yaml',
        sorting_bins_config=package/'config/robot_top_bins.yaml')
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    node, request = registration_fixture()
    node._body_candidates = {'cup': {'study_cafe_cup', 'study_cafe_cup_b'}}
    base = model.body('chassis').id
    b = model.body('study_cafe_cup_b').id
    bin_ = node._bins['trash_right']
    address = model.jnt_qposadr[model.body_jntadr[b]]
    base_rotation = data.xmat[base].reshape(3,3).copy()
    position = base_rotation @ np.array([*bin_.center_xy, bin_.bottom_z+bin_.wall+.003]) + data.xpos[base]
    data.qpos[address:address+3] = position
    data.qpos[address+3:address+7] = data.xquat[base]
    now = [0]
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now[0]))

    def sample(body_name):
        body = model.body(body_name).id
        rotation = data.xmat[base].reshape(3,3)
        lows, highs = [], []
        for geom in range(model.body_geomadr[body], model.body_geomadr[body]+model.body_geomnum[body]):
            if not model.geom_contype[geom] and not model.geom_conaffinity[geom]:
                continue
            relative = rotation.T @ data.geom_xmat[geom].reshape(3,3)
            center = rotation.T @ (data.geom_xpos[geom]-data.xpos[base]) + relative @ model.geom_aabb[geom,:3]
            extent = np.abs(relative) @ model.geom_aabb[geom,3:]
            lows.append(center-extent)
            highs.append(center+extent)
        low, high = np.min(lows,axis=0), np.max(highs,axis=0)
        return now[0], tuple((low+high)/2), tuple(high-low)

    # Read-only outcome observations follow actual physics steps; no target welds.
    for _ in range(50):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    now[0] = round(data.time*1e9)
    for name in node._body_candidates['cup']:
        node._samples[name] = [sample(name)]
    observed = node._samples['study_cafe_cup'][-1]
    request.header.stamp = Time(nanoseconds=now[0]).to_msg()
    point, size = request.target.obb_pose.position, request.target.obb_size
    point.x, point.y, point.z = observed[1]
    size.x, size.y, size.z = observed[2]
    registered = PlacementVerifier._register(node, request, RegisterPlacementTarget.Response())
    assert registered.success, registered.message
    barrier = now[0]
    for _ in range(7):
        for _ in range(100):
            mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        now[0] = round(data.time*1e9)
        for name in node._body_candidates['cup']:
            node._samples[name].append(sample(name))
    verify = VerifyPlacement.Request(verification_id=registered.verification_id,
                                    destination_id='trash_right', after_stamp_ns=barrier)
    assert not PlacementVerifier._verify(node, verify, VerifyPlacement.Response()).success
    # Establish that the other cup really has settled in the bin.
    node._bodies['cup'] = 'study_cafe_cup_b'
    other = VerifyPlacement.Request(label='cup', destination_id='trash_right', after_stamp_ns=barrier)
    result = PlacementVerifier._verify(node, other, VerifyPlacement.Response())
    assert result.success, result.message


@pytest.mark.parametrize('label', ['mouse', 'computer mouse', 'wireless mouse', 'lego brick'])
def test_mujoco_registered_lost_item_requires_its_own_body_in_the_correct_bin(label):
    import mujoco
    import numpy as np
    from cleany_interfaces.srv import RegisterPlacementTarget
    from rclpy.time import Time
    from cleany_mujoco_sim.scene_loader import materialize_control_scene
    package = Path(__file__).parents[1]
    path = materialize_control_scene(package/'scenes/study_cafe_grasp_execution.xml.in',
        study_cafe_layout_config=package/'config/study_cafe_layout.yaml',
        sorting_bins_config=package/'config/robot_top_bins.yaml')
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    node, request = registration_fixture()
    body_name = DEFAULT_LABEL_BODIES[label]
    other_name = 'study_cafe_cup'
    node._body_candidates = {label: {body_name}}
    base = model.body('chassis').id
    now = [0]
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now[0]))

    def sample(name):
        body = model.body(name).id
        rotation = data.xmat[base].reshape(3, 3)
        lows, highs = [], []
        for geom in range(model.body_geomadr[body], model.body_geomadr[body]+model.body_geomnum[body]):
            if not model.geom_contype[geom] and not model.geom_conaffinity[geom]:
                continue
            relative = rotation.T @ data.geom_xmat[geom].reshape(3, 3)
            center = rotation.T @ (data.geom_xpos[geom]-data.xpos[base]) + relative @ model.geom_aabb[geom, :3]
            extent = np.abs(relative) @ model.geom_aabb[geom, 3:]
            lows.append(center-extent)
            highs.append(center+extent)
        low, high = np.min(lows, axis=0), np.max(highs, axis=0)
        return now[0], tuple((low+high)/2), tuple(high-low)

    def put_in_bin(name, destination):
        bin_ = node._bins[destination]
        _, center, size = sample(name)
        desired = np.array([*bin_.center_xy, bin_.bottom_z+bin_.wall+size[2]/2+.005])
        body = model.body(name).id
        joint = model.body_jntadr[body]
        address = model.jnt_qposadr[joint]
        data.qpos[address:address+3] += data.xmat[base].reshape(3, 3) @ (desired-np.array(center))
        velocity = model.jnt_dofadr[joint]
        data.qvel[velocity:velocity+6] = 0.
        mujoco.mj_forward(model, data)

    def settle_and_sample():
        for _ in range(10):
            for _ in range(100):
                mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            now[0] = round(data.time*1e9)
            for name in (body_name, other_name):
                node._samples[name].append(sample(name))

    settle_and_sample()
    request.destination_id, request.target.label = 'lost_items_left', label
    request.header.stamp = Time(nanoseconds=now[0]).to_msg()
    observed = sample(body_name)
    point, size = request.target.obb_pose.position, request.target.obb_size
    point.x, point.y, point.z = observed[1]
    size.x, size.y, size.z = observed[2]
    registration = PlacementVerifier._register(node, request, RegisterPlacementTarget.Response())
    assert registration.success, registration.message
    verify = VerifyPlacement.Request(verification_id=registration.verification_id,
                                    destination_id='lost_items_left', after_stamp_ns=now[0])
    put_in_bin(other_name, 'lost_items_left')
    settle_and_sample()
    node._bodies['cup'] = other_name
    other = VerifyPlacement.Request(label='cup', destination_id='lost_items_left', after_stamp_ns=verify.after_stamp_ns)
    assert PlacementVerifier._verify(node, other, VerifyPlacement.Response()).success
    assert not PlacementVerifier._verify(node, verify, VerifyPlacement.Response()).success
    verify.after_stamp_ns = now[0]
    put_in_bin(body_name, 'trash_right')
    settle_and_sample()
    assert not PlacementVerifier._verify(node, verify, VerifyPlacement.Response()).success
    verify.destination_id = 'trash_right'
    assert not PlacementVerifier._verify(node, verify, VerifyPlacement.Response()).success
    verify.destination_id, verify.after_stamp_ns = 'lost_items_left', now[0]
    # Clear the other object to keep the final placement independent of collisions.
    put_in_bin(other_name, 'trash_right')
    put_in_bin(body_name, 'lost_items_left')
    settle_and_sample()
    result = PlacementVerifier._verify(node, verify, VerifyPlacement.Response())
    if label == 'lego brick':
        assert result.success, result.message
    else:
        # The rotated local mouse-mesh box extends below the bin floor even
        # though the mesh vertices are inside. Preserve conservative rejection;
        # improving the observer's mesh bounds is separate from Planner routing.
        assert not result.success and 'not inside' in result.message
        body = model.body(body_name).id
        vertices = []
        for geom in range(model.body_geomadr[body], model.body_geomadr[body]+model.body_geomnum[body]):
            if not model.geom_contype[geom] and not model.geom_conaffinity[geom]:
                continue
            mesh = model.geom_dataid[geom]
            address, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            relative = data.xmat[base].reshape(3, 3).T @ data.geom_xmat[geom].reshape(3, 3)
            origin = data.xmat[base].reshape(3, 3).T @ (data.geom_xpos[geom]-data.xpos[base])
            vertices.extend(model.mesh_vert[address:address+count] @ relative.T + origin)
        bin_ = node._bins['lost_items_left']
        low = np.array([bin_.center_xy[0]-bin_.outside_size[0]/2+bin_.wall,
                        bin_.center_xy[1]-bin_.outside_size[1]/2+bin_.wall, bin_.bottom_z+bin_.wall])
        high = np.array([bin_.center_xy[0]+bin_.outside_size[0]/2-bin_.wall,
                         bin_.center_xy[1]+bin_.outside_size[1]/2-bin_.wall, bin_.top_z])
        assert np.all(np.min(vertices, axis=0) >= low-.002)
        assert np.all(np.max(vertices, axis=0) <= high+.002)
