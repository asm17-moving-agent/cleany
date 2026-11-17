"""Independent synthetic pose checks for offline kinematic calibration."""

import importlib.util
from pathlib import Path
import sys

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


SOURCE = Path(__file__).resolve().parents[2] / 'tools/kinematics_calibration/model.py'
SPEC = importlib.util.spec_from_file_location('kinematic_model', SOURCE)
model = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = model
SPEC.loader.exec_module(model)


def chain():
    origins = np.tile(np.eye(4), (5, 1, 1))
    origins[:, :3, 3] = [[0, 0, .1], [.02, 0, .05], [0, .16, .02], [0, .18, .01], [0, .06, 0]]
    return model.Chain(list('abcde'), origins, np.array([[0, 0, 1], [1, 0, 0], [1, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=float))


def test_nominal_fk_agrees_with_direct_transform_product():
    robot = chain()
    q = np.array([[.3, -.4, .7, -.2, .1], [-.5, .1, .2, -.1, -.7]])
    original = q.copy()
    expected = []
    for angles in q:
        transform = np.eye(4)
        for origin, axis, angle in zip(robot.origins, robot.axes, angles):
            rotation = np.eye(4)
            rotation[:3, :3] = cv2.Rodrigues(axis * angle)[0]
            transform = transform @ origin @ rotation
        expected.append(transform)
    np.testing.assert_allclose(robot.fk(q, np.zeros(25)), expected, atol=1e-14)
    np.testing.assert_array_equal(q, original)


def test_geometry_corrections_predict_unseen_synthetic_views():
    robot = chain()
    truth = np.zeros(25)
    truth[3:6] = [.02, -.01, .04]
    truth[9:12] = [0, .2, 1.2]
    truth[12:15] = [.025, -.02, .015]
    truth[15:23] = [.02, -.015, -.025, .01, .015, .02, -.01, -.02]
    truth[23:25] = [.003, -.002]
    rng = np.random.default_rng(73)
    angles = rng.uniform(-.7, .7, (25, 5))
    board_points = np.array([[x, y, 0.] for x in (-.06, 0, .06) for y in (-.06, 0, .06)])
    camera_matrix = np.array([[640., 0, 320], [0, 640, 240], [0, 0, 1]])
    observations = []
    for i, wrist in enumerate(robot.fk(angles, truth)):
        camera_board = np.linalg.inv(wrist @ model.pose(truth[:6])) @ model.pose(truth[6:12])
        r = Rotation.from_matrix(camera_board[:3, :3]).as_rotvec()
        pixels = cv2.projectPoints(board_points, r, camera_board[:3, 3], camera_matrix, np.zeros(5))[0].reshape(-1, 2)
        observations.append(model.Observation(str(i), angles[i], board_points, pixels))
    problem = model.Problem(robot, observations, camera_matrix, np.zeros(5))
    seed = truth[:12].copy()
    for family in model.PARAMETER_COUNTS:
        solution = problem.fit(family, list(range(20)), seed, max_evaluations=240)
        assert solution.success
        seed = solution.x
        if family == 'extrinsics':
            baseline = problem.evaluate(solution.x, list(range(20, 25)))['rms_px']
    held = problem.evaluate(solution.x, list(range(20, 25)))
    assert held['all_points_in_front']
    assert held['rms_px'] < .2
    assert held['rms_px'] < baseline / 10


def test_shifted_axis_rotates_about_its_line_not_the_frame_origin():
    robot = chain()
    parameters = np.zeros(31)
    parameters[29] = .004
    angles = np.array([[0., 0., 0., 0., .7]])
    pivot = np.array([0., 0., -.004])
    before = np.eye(4)
    after = np.eye(4)
    before[:3, 3], after[:3, 3] = pivot, -pivot
    rotation = np.eye(4)
    rotation[:3, :3] = cv2.Rodrigues(np.array([0., .7, 0.]))[0]
    expected = np.eye(4)
    for origin in robot.origins:
        expected = expected @ origin
    expected = expected @ before @ rotation @ after
    np.testing.assert_allclose(robot.fk(angles, parameters)[0], expected, atol=1e-14)
