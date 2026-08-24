import numpy as np

from cleany_grasping.core.models import PointCloud, RawGrasp
from cleany_grasping.core.selector import GraspConfig, select_grasp


class FakePredictor:
    def __init__(self, candidates):
        self.candidates = tuple(candidates)
        self.workspace = None

    def predict(self, target_cloud, context_cloud, workspace_bounds):
        del target_cloud, context_cloud
        self.workspace = workspace_bounds
        return self.candidates


def cloud(points):
    points = np.asarray(points, dtype=float)
    return PointCloud(points, np.zeros_like(points))


def grasp(x, score, width=0.04, rotation=None, depth=0.0):
    return RawGrasp(
        np.eye(3) if rotation is None else rotation,
        np.asarray(x, dtype=float),
        width,
        depth,
        score,
    )


def test_selects_highest_scoring_target_contact_after_width_filter() -> None:
    target = cloud(((-0.02, -0.02, 0.5), (0.02, 0.02, 0.55)))
    predictor = FakePredictor(
        (
            grasp((0.0, 0.0, 0.52), 0.8),
            grasp((0.0, 0.0, 0.52), 0.99, width=0.2),
            grasp((0.3, 0.0, 0.52), 0.9),
        )
    )

    result = select_grasp(predictor, target, target)

    assert result is not None
    assert result.score == 0.8
    assert np.allclose(result.approach_direction, (1.0, 0.0, 0.0))
    assert np.allclose(predictor.workspace, (-0.06, 0.06, -0.06, 0.06, 0.46, 0.59))


def test_canonical_rotation_is_converted_to_tcp_axes() -> None:
    target = cloud(((-0.1, -0.1, -0.1), (0.1, 0.1, 0.1)))
    conversion = np.array(((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)))

    result = select_grasp(
        FakePredictor((grasp((0.0, 0.0, 0.0), 0.7),)),
        target,
        target,
        GraspConfig(canonical_to_tcp_rotation=conversion),
    )

    assert result is not None
    assert np.allclose(result.rotation, conversion)
    assert np.allclose(result.approach_direction, (0.0, 1.0, 0.0))
    assert result.required_opening_m == 0.04


def test_returns_none_instead_of_inventing_candidate() -> None:
    target = cloud(((-0.01, -0.01, 0.5), (0.01, 0.01, 0.52)))
    assert select_grasp(FakePredictor(()), target, target) is None


def test_nms_keeps_best_of_near_duplicate_candidates() -> None:
    target = cloud(((-0.1, -0.1, -0.1), (0.1, 0.1, 0.1)))
    predictor = FakePredictor(
        (grasp((0.0, 0.0, 0.0), 0.9), grasp((0.001, 0.0, 0.0), 0.8))
    )
    result = select_grasp(predictor, target, target)
    assert result is not None and result.score == 0.9
