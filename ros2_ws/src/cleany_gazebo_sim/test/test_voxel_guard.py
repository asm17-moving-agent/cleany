import numpy as np
import pytest
from cleany_gazebo_sim.body_geometry import BodyBox
from cleany_gazebo_sim.voxel_guard import GuardConfig, VoxelGuard, planar_pose, voxel_box_hits


def body(center=(0., 0., .5), half=(.1, .1, .1), rotation=None):
    return BodyBox('body', np.array(center), np.eye(3) if rotation is None else rotation, np.array(half))


def test_voxel_extent_counts_even_when_center_is_outside():
    assert voxel_box_hits(np.array([[.125, 0., .5]]), np.full(3, .06), body())[0]
    assert not voxel_box_hits(np.array([[.15, 0., .5]]), np.full(3, .06), body())[0]


def test_rotated_box_sat_rejects_broad_phase_false_positive():
    rotation = planar_pose(0, 0, np.pi/4)[:3, :3]
    box = body(half=(.3, .03, .05), rotation=rotation)
    hits = voxel_box_hits(np.array([[.15, .15, .5], [.15, -.15, .5]]), np.full(3, .02), box)
    np.testing.assert_array_equal(hits, [True, False])


def test_height_separation_and_translation_stop_and_slowdown():
    guard = VoxelGuard([body()], GuardConfig())
    size = np.full(3, .02)
    out, report = guard.evaluate(np.array([[.2, 0., 1.5]]), size, np.eye(4), np.array([.15, 0, 0]), np.zeros(3))
    assert out[0] == .15
    out, report = guard.evaluate(np.array([[.19, 0., .5]]), size, np.eye(4), np.array([.15, 0, 0]), np.zeros(3))
    assert report['decision'] == 'STOP_OBSTACLE' and np.all(out == 0)
    out, report = guard.evaluate(np.array([[.27, 0., .5]]), size, np.eye(4), np.array([.15, 0, 0]), np.zeros(3))
    assert report['decision'] == 'SLOW_OBSTACLE' and 0 < out[0] < .15


def test_rotation_sweep_and_measured_motion_survive_zero_request():
    guard = VoxelGuard([body(center=(.5, 0, .5), half=(.02, .02, .02))], GuardConfig())
    point = np.array([[.5*np.cos(.15), .5*np.sin(.15), .5]])
    out, report = guard.evaluate(point, np.full(3, .02), np.eye(4), np.zeros(3), np.array([0, 0, .3]))
    assert report['decision'] == 'STOP_OBSTACLE'


def test_nonfinite_input_and_invalid_size_rejected():
    guard = VoxelGuard([body()], GuardConfig())
    with pytest.raises(ValueError):
        guard.evaluate(np.array([[np.nan, 0, 0]]), np.ones(3), np.eye(4), np.zeros(3), np.zeros(3))
    with pytest.raises(ValueError):
        voxel_box_hits(np.zeros((1, 3)), np.zeros(3), body())
