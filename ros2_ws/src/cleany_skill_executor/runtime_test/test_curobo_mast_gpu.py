"""Actual pinned cuRobo queries; no ROS controllers or robot motion."""
import os

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('CLEANY_RUN_CUROBO') != '1',
                               reason='Requires pinned CUDA environment')


@pytest.fixture
def checker():
    from curobo.geom.sdf.world import WorldCollisionConfig
    from curobo.geom.sdf.world_mesh import WorldMeshCollision
    from curobo.geom.types import WorldConfig
    from curobo.types.base import TensorDeviceType
    from cleany_skill_executor.curobo_mast_checker import MastWorldCollision

    tensor = TensorDeviceType()
    environment = WorldMeshCollision(WorldCollisionConfig(tensor_args=tensor,
        world_model=WorldConfig.from_dict({}), cache={'obb': 128, 'mesh': 128}))
    kin = {'collision_link_names': ['left_arm', 'head_pan_link'],
           'collision_spheres': {'left_arm': [{}], 'head_pan_link': [{}]}}
    return MastWorldCollision(environment, {'mast': {
        'dims': [.2, .2, .2], 'pose': [0., 0., 0., 1., 0., 0., 0.]}}, kin, tensor)


def query(checker, *, swept=False):
    import torch
    from curobo.geom.sdf.world import CollisionQueryBuffer

    values = checker.tensor_args.to_device([[[[.12, 0., 0., .03], [.12, 0., 0., .03]],
                                           [[.14, 0., 0., .03], [.14, 0., 0., .03]]]])
    values.requires_grad_(True)
    buffer = CollisionQueryBuffer.initialize_from_shape(values.shape, checker.tensor_args,
                                                       checker.collision_types)
    args = dict(weight=checker.tensor_args.to_device([1.]),
                activation_distance=checker.tensor_args.to_device([.01]), return_loss=True)
    if swept:
        args.update(speed_dt=checker.tensor_args.to_device([.1]), sweep_steps=4,
                    enable_speed_metric=False)
    method = checker.get_swept_sphere_distance if swept else checker.get_sphere_distance
    return values, buffer, args, method


@pytest.mark.parametrize('swept', [False, True])
def test_mast_checks_arm_not_head_mount_and_keeps_gradients(checker, swept):
    import torch

    values, buffer, args, method = query(checker, swept=swept)
    original = values.detach().clone()
    costs = method(values, buffer, **args)
    assert costs[..., 0].max().item() > 0
    assert costs[..., 1].abs().max().item() == 0
    costs.sum().backward()
    assert values.grad[..., 0, :3].abs().max().item() > 0
    assert values.grad[..., 1, :].abs().max().item() == 0
    assert torch.equal(original, values)


def test_scene_update_preserves_mast_and_head_world_collision(checker):
    from curobo.geom.types import WorldConfig

    values, buffer, args, method = query(checker)
    assert method(values, buffer, **args)[..., 1].max().item() == 0
    checker.clear_cache()
    checker.load_collision_model(WorldConfig.from_dict({'cuboid': {'wall': {
        'dims': [.2, .2, .2], 'pose': [0., 0., 0., 1., 0., 0., 0.]}}}))
    costs = method(values, buffer, **args)
    assert costs[..., 1].max().item() > 0
    checker.clear_cache()
    checker.load_collision_model(WorldConfig.from_dict({}))
    costs = method(values, buffer, **args)
    assert costs[..., 0].max().item() > 0 and costs[..., 1].max().item() == 0


def test_cuda_graph_replay_uses_new_sphere_positions(checker):
    import torch

    values, buffer, args, method = query(checker)
    values = values.detach()
    stream = torch.cuda.Stream()
    stream.wait_stream(torch.cuda.current_stream())
    with torch.cuda.stream(stream):
        for _ in range(3):
            method(values, buffer, **args)
    torch.cuda.current_stream().wait_stream(stream)
    graph = torch.cuda.CUDAGraph()
    with torch.cuda.graph(graph):
        result = method(values, buffer, **args)
    graph.replay()
    assert result[..., 0].max().item() > 0
    values[..., :3] = 2.
    graph.replay()
    assert result.max().item() == 0
