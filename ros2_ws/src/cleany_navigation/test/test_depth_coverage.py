import math
import numpy as np
from cleany_navigation.depth_coverage import DepthView, SweptDepthCoverage
from cleany_navigation.body_geometry import BodyBox
from cleany_navigation.voxel_guard import GuardConfig, planar_pose


def view(depth=None, stamp=1., pose=None):
    return DepthView(stamp, np.full((80, 100), 4.) if depth is None else depth,
                     (50., 50., 50., 40.), np.eye(4) if pose is None else pose)


def test_whole_cube_requires_every_pixel_and_rejects_occlusion_and_invalid_depth():
    center = np.array([[0., 0., 2.]])
    assert view().covers(center, .1, np.eye(4)).all()
    for value in (0., np.nan, np.inf, -np.inf, 1.9, 2.06):
        depth = np.full((80, 100), 4.); depth[40, 50] = value
        assert not view(depth).covers(center, .1, np.eye(4)).any()
    # A single clear ray does not clear the surrounding cube.
    depth = np.zeros((80, 100)); depth[40, 50] = 4.
    assert not view(depth).covers(center, .1, np.eye(4)).any()


def test_fov_edge_behind_camera_and_pose_at_acquisition_are_respected():
    centers = np.array([[0., 0., 2.], [1.96, 0., 2.], [0., 0., -2.]])
    assert view().covers(centers, .1, np.eye(4)).tolist() == [True, False, False]
    moved = np.eye(4); moved[0, 3] = 5.
    assert not view().covers(centers[:1], .1, moved).any()
    assert view(pose=moved).covers(centers[:1], .1, moved).all()


def test_small_direction_changes_pass_only_with_full_swept_coverage():
    box = BodyBox('body', np.array([0., 0., 2.]), np.eye(3), np.full(3, .1))
    coverage = SweptDepthCoverage([box], GuardConfig())
    coverage.observe(view())
    for angle in (30, 25):
        v = np.array([.1*math.cos(math.radians(angle)), .1*math.sin(math.radians(angle)), 0.])
        assert coverage.evaluate(1.1, np.eye(4), v, v)['covered']
    assert not coverage.evaluate(1.6, np.eye(4), v, v)['covered']
    coverage.observe(view(stamp=2.))
    assert coverage.evaluate(2., np.eye(4), v, v)['covered']
    assert not coverage.evaluate(1., np.eye(4), v, v)['covered']


def test_unknown_side_and_measured_braking_cannot_be_ignored():
    box = BodyBox('body', np.array([1.5, 0., 2.]), np.eye(3), np.full(3, .1))
    coverage = SweptDepthCoverage([box], GuardConfig())
    coverage.observe(view())
    # New side space beyond the image boundary remains unknown even with zero
    # requested velocity, since the body still has measured inertia.
    result = coverage.evaluate(1.1, np.eye(4), np.zeros(3), np.array([.15, 0., .3]))
    assert not result['covered']
    coverage.clear()
    assert coverage.evaluate(1.1, np.eye(4), np.zeros(3), np.zeros(3))['reason'] == 'NO_FRESH_DEPTH'


def test_new_obstacle_or_invalid_pixel_revokes_older_clear_observation():
    box = BodyBox('body', np.array([0., 0., 2.]), np.eye(3), np.full(3, .1))
    for new_depth in (1., np.nan, np.inf):
        coverage = SweptDepthCoverage([box], GuardConfig())
        coverage.observe(view())
        v = np.array([.1, 0., 0.])
        assert coverage.evaluate(1., np.eye(4), v, v)['covered']
        coverage.observe(view(np.full((80, 100), new_depth), stamp=1.1))
        assert not coverage.evaluate(1.1, np.eye(4), v, v)['covered']


def test_future_observation_and_excessive_cell_budget_never_authorize():
    box = BodyBox('body', np.array([0., 0., 2.]), np.eye(3), np.full(3, .1))
    coverage = SweptDepthCoverage([box], GuardConfig(), max_cells=1)
    coverage.observe(view(stamp=2.))
    v = np.array([.1, 0., 0.])
    assert not coverage.evaluate(1., np.eye(4), v, v)['covered']
    coverage.observe(view(stamp=2.))
    assert coverage.evaluate(2., np.eye(4), v, v)['reason'] == 'CELL_BUDGET'


def test_pose_jump_invalidates_transient_free_space():
    box = BodyBox('body', np.array([0., 0., 2.]), np.eye(3), np.full(3, .1))
    coverage = SweptDepthCoverage([box], GuardConfig())
    coverage.observe(view())
    v = np.array([.1, 0., 0.])
    assert coverage.evaluate(1., np.eye(4), v, v)['covered']
    assert coverage.evaluate(1.1, planar_pose(.3, 0., 0.), v, v)['reason'] == 'POSE_JUMP'
    assert not coverage.views


def test_tof_region_reuses_finite_depth_and_matches_conservative_braking_model():
    from cleany_tof.core import Settings
    from cleany_tof.proximity import TofRegion
    from cleany_navigation.depth_coverage import create_tof_coverage
    s = Settings(calibrated=True, sensor_z=1.8, sensor_pitch=-math.pi/2,
                 half_length=.1, half_width=.1)
    body = BodyBox('body', np.array([0., 0., 2.]), np.eye(3), np.full(3, .1))
    c = create_tof_coverage([body], GuardConfig(), s, cell_size=.05)
    v = np.array([.03, 0., 0.])
    region = TofRegion(s)
    c.observe(view(stamp=1.))
    assert c.evaluate(1.01, np.eye(4), v, v, region=region.intersects)['covered']
    assert c.guard.config.reaction_s >= s.reaction_s+s.source_timeout_s
    assert c.guard.config.linear_deceleration <= s.deceleration/2
    assert c.guard.config.margin >= s.margin+3*s.max_sigma_m
    # A new occluder or invalid pixels revoke the independent evidence.
    c.observe(view(np.zeros((80, 100)), stamp=1.02))
    assert not c.evaluate(1.03, np.eye(4), v, v, region=region.intersects)['covered']
    assert not c.evaluate(1.3, np.eye(4), v, v, region=region.intersects)['covered']


def test_expanded_tof_envelope_is_not_treated_as_physical_inside_body():
    from cleany_tof.core import Settings
    from cleany_navigation.depth_coverage import create_tof_coverage
    body = BodyBox('body', np.array([0., 0., 2.]), np.eye(3), np.full(3, .1))
    c = create_tof_coverage([body], GuardConfig(), Settings(), cell_size=.1)
    assert c.body_boxes == [body] and len(c.guard.boxes) == 2
    c.observe(view(np.zeros((80, 100))))
    assert not c.evaluate(1.01, np.eye(4), np.zeros(3), np.zeros(3),
                          region=lambda centers, size: np.ones(len(centers), dtype=bool))['covered']
