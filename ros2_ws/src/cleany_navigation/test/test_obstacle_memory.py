from dataclasses import replace

import numpy as np
import pytest

from cleany_navigation.obstacle_memory import MemoryGeometry, ObstacleMemory


@pytest.fixture
def memory():
    return ObstacleMemory(MemoryGeometry(40, 40, 0.1, -2.0, -2.0, 0.0, 2.0, 0.1, 'map-a'))


def observe(memory, source='depth', hit=True, endpoint=(1.05, 0.05, 1.05), sensor=(0.05, 0.05, 1.05)):
    memory.observe(source, np.array(sensor), np.array([endpoint]), np.array([hit]), 100.0)


def test_unseen_and_out_of_view_obstacles_persist(memory):
    assert np.all(memory.grid() == -1)
    observe(memory)
    observe(memory, endpoint=(-1.05, 0.05, 1.05))
    assert memory.grid()[20, 30] == 100
    assert memory.grid()[35, 35] == -1


def test_observed_free_space_clears_but_keeps_history(memory):
    observe(memory)
    for _ in range(3):
        observe(memory, hit=False, endpoint=(1.5, 0.05, 1.05))
    assert memory.grid()[20, 30] == 0
    assert memory.hit_count[1, 20, 30] == 1
    assert memory.last_seen[1, 20, 30] == 100


def test_lidar_cannot_erase_depth_and_floor_ray_cannot_erase_high_obstacle(memory):
    observe(memory)
    for _ in range(8):
        observe(memory, source='lidar', hit=False, endpoint=(1.5, 0.05, 1.05))
        observe(memory, hit=False, endpoint=(1.5, 0.05, -0.2))
    assert memory.grid()[20, 30] == 100


def test_hits_win_over_crossing_free_rays_in_same_frame(memory):
    memory.observe('lidar', np.array([0.05, 0.05, 1.05]),
                   np.array([[1.05, 0.05, 1.05], [1.5, 0.05, 1.05]]), np.array([True, False]), 100)
    assert memory.grid()[20, 30] == 100


def test_invalid_returns_do_not_clear_and_map_edges_do_not_wrap(memory):
    observe(memory)
    observe(memory, endpoint=(float('nan'), 0, 0))
    observe(memory, endpoint=(5.0, 0.05, 0.5))
    assert memory.grid()[20, 30] == 100
    assert memory.grid()[0, 0] == -1


def test_snapshot_restores_all_evidence_and_rejects_other_map(memory, tmp_path):
    observe(memory)
    observe(memory, source='lidar', endpoint=(-1.05, 0.05, 0.35))
    path = tmp_path/'memory.npz'
    memory.save(path)
    restored = ObstacleMemory.restore(path, memory.geometry)
    np.testing.assert_array_equal(memory.hit_count, restored.hit_count)
    np.testing.assert_allclose(memory.occupied_centers(), restored.occupied_centers(include_expired=True))
    assert np.all(restored.grid() == -1)
    assert restored.grid(block_expired=True)[20, 30] == 100
    with pytest.raises(ValueError, match='does not match'):
        ObstacleMemory.restore(path, replace(memory.geometry, map_id='map-b'))


def test_map_reload_retains_obstacles_but_updates_snapshot_identity(memory, tmp_path):
    observe(memory)
    expected = memory.evidence.copy()
    new = replace(memory.geometry, map_id='chair-removed-map')
    memory.rebind_map(new)
    np.testing.assert_array_equal(memory.evidence, expected)
    path = tmp_path/'reloaded.npz'
    memory.save(path)
    restored = ObstacleMemory.restore(path, new)
    np.testing.assert_allclose(memory.occupied_centers(), restored.occupied_centers(include_expired=True))
    assert not restored.occupied_centers().size
    with pytest.raises(ValueError, match='geometry changed'):
        memory.rebind_map(replace(new, origin_x=-1.))
    assert memory.geometry == new


def test_voxel_centers_preserve_two_heights_and_source_specific_clearing(memory):
    observe(memory, endpoint=(1.05, .05, .35), sensor=(.05, .05, .35))
    observe(memory, endpoint=(1.05, .05, 1.35), sensor=(.05, .05, 1.35))
    np.testing.assert_allclose(np.sort(memory.occupied_centers()[:, 2]), [.35, 1.35])
    for _ in range(3):
        observe(memory, hit=False, endpoint=(1.5, .05, .35), sensor=(.05, .05, .35))
    np.testing.assert_allclose(memory.occupied_centers(), [[1.05, .05, 1.35]])


def test_unsampled_depth_hits_are_retained_and_win_over_clearing(memory):
    memory.observe('depth', np.array([.05, .05, 1.05]), np.array([[1.5, .05, 1.05]]),
                   np.array([False]), 100, np.array([[1.05, .05, 1.05]]))
    np.testing.assert_allclose(memory.occupied_centers(), [[1.05, .05, 1.05]])


def test_expiry_is_per_voxel_and_another_sensor_cannot_clear_unknown(memory):
    observe(memory)
    memory.observe('depth', np.array([.05, .05, .35]), np.array([[1.05, .05, .35]]), np.array([True]), 108)
    assert memory.expire(111, 10) == 1
    assert memory.grid()[20, 30] == 100  # fresh lower voxel survives
    assert memory.grid(min_z=1., max_z=1.5)[20, 30] == -1
    assert memory.grid(min_z=1., max_z=1.5, block_expired=True)[20, 30] == 100
    np.testing.assert_allclose(np.sort(memory.occupied_centers(include_expired=True)[:, 2]), [.35, 1.05])
    observe(memory, source='lidar', hit=False, endpoint=(1.5, .05, 1.05))
    assert memory.grid(min_z=1., max_z=1.5)[20, 30] == -1
    memory.observe('depth', np.array([.05, .05, 1.05]), np.array([[1.5, .05, 1.05]]), np.array([False]), 112)
    assert memory.grid(min_z=1., max_z=1.5, block_expired=True)[20, 30] == 0


def test_expiry_does_not_make_unobserved_space_free_and_zero_disables(memory):
    observe(memory)
    assert memory.expire(1000, 0) == 0
    assert memory.grid()[20, 30] == 100
    assert memory.expire(1000, 10) == 1
    assert memory.grid()[20, 30] == -1
    assert memory.grid(block_expired=True)[20, 30] == 100
    assert memory.grid(block_expired=True)[35, 35] == -1


def test_projection_includes_intersecting_volumes_and_excludes_overhead(memory):
    observe(memory, endpoint=(1.05, .05, 1.35))
    assert memory.grid(min_z=.05, max_z=1.2)[20, 30] != 100
    assert memory.grid(min_z=.05, max_z=1.31)[20, 30] == 100


def test_local_voxels_match_global_subset_including_expired(memory):
    observe(memory)
    observe(memory, endpoint=(-1.05, .05, 1.05))
    memory.expire(111, 10)
    all_points = memory.occupied_centers(include_expired=True)
    center = np.array([1., 0.])
    expected = all_points[np.linalg.norm(all_points[:, :2]-center, axis=1) <= .5]
    np.testing.assert_allclose(memory.occupied_centers(center=center, radius=.5, include_expired=True), expected)


def test_far_hit_still_clears_near_ray_and_only_marks_local_hits():
    from cleany_navigation.obstacle_memory import local_observation
    sensor, center = np.array([.1, .1, 1.]), np.array([0., 0.])
    points, hits, full = local_observation(sensor, np.array([[5., .1, 1.], [1., .1, 1.]]),
        np.array([True, True]), np.array([[5., .1, 1.], [1., .1, 1.]]), center, 2.)
    assert not hits[0] and hits[1]
    assert np.linalg.norm(points[0, :2]) == pytest.approx(2.)
    np.testing.assert_allclose(full, [[1., .1, 1.]])
