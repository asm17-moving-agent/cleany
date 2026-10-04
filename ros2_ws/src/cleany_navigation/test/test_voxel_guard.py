import numpy as np
import pytest
from itertools import product
from cleany_navigation.body_geometry import BodyBox
from cleany_navigation.voxel_guard import GuardConfig, VoxelGuard, planar_pose, twist_pose, voxel_box_hits


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


@pytest.mark.parametrize('velocity', [(0., 0., 0.), (.15, -.04, .3), (-.1, .06, -.2)])
def test_optimized_sweep_matches_corner_projection_sat(velocity):
    """An independent corner-projection oracle protects the swept body volume."""
    rng = np.random.default_rng(28)
    signs = np.array(list(product((-1., 1.), repeat=3)))
    roll = .31
    rotation = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)],
                         [0, np.sin(roll), np.cos(roll)]])
    boxes = [body(half=(.3, .07, .12), rotation=rotation), body(center=(.4, -.1, .3))]
    guard = VoxelGuard(boxes, GuardConfig())
    points = rng.uniform([-.6, -.6, .1], [.9, .6, .9], (160, 3))
    size = np.array([.04, .06, .08])
    base = planar_pose(.1, -.1, .47)
    velocity = np.array(velocity)
    speed = np.linalg.norm(velocity[:2])
    extra = .5
    horizon = guard.config.reaction_s+max(speed/.4, abs(velocity[2])/.6)+extra
    times = [0.] if np.all(velocity == 0) else np.linspace(0, horizon, int(np.ceil(horizon/.05))+1)
    padding = guard.config.margin+(speed+abs(velocity[2])*guard.radius)*.05/2
    expected = np.zeros(len(points), dtype=bool)
    voxel_corners = points[:, None, :]+signs*(size/2)
    for seconds in times:
        pose = base@twist_pose(velocity, seconds)
        for box in boxes:
            r = pose[:3, :3]@box.rotation
            center = pose[:3, :3]@box.center+pose[:3, 3]
            corners = (signs*(box.half_size+padding))@r.T+center
            axes = [*np.eye(3), *r.T, *(np.cross(a, b) for a in np.eye(3) for b in r.T)]
            overlap = np.ones(len(points), dtype=bool)
            for axis in axes:
                if np.linalg.norm(axis) <= 1e-10:
                    continue
                a, b = corners@axis, voxel_corners@axis
                overlap &= (b.max(axis=1) >= a.min()-1e-10) & (b.min(axis=1) <= a.max()+1e-10)
            expected |= overlap
    np.testing.assert_array_equal(guard.sweep(points, size, base, velocity, extra), expected)


def test_sweep_rejects_invalid_base_rotation():
    guard = VoxelGuard([body()], GuardConfig())
    base = np.eye(4)
    base[0, 0] = 2
    with pytest.raises(ValueError):
        guard.sweep(np.zeros((1, 3)), np.ones(3), base, np.zeros(3))
