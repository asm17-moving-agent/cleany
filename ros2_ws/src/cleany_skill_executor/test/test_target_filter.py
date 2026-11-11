from dataclasses import dataclass
import math

from cleany_skill_executor.core.target_filter import (
    RejectionReason,
    select_closest_eligible,
)


@dataclass
class Detection:
    object_id: int
    label: str
    median_depth_m: float
    estimated_width_m: float
    estimated_height_m: float


def test_denylist_and_size_are_applied_before_nearest_selection() -> None:
    detections = [
        Detection(1, 'computer keyboard', 0.30, 0.40, 0.12),
        Detection(2, 'blue cloth towel', 0.35, 0.08, 0.05),
        Detection(3, 'large box', 0.40, 0.20, 0.10),
        Detection(4, 'small can', 0.45, 0.06, 0.09),
        Detection(5, 'small bottle', 0.55, 0.05, 0.08),
    ]

    result = select_closest_eligible(detections)

    assert result.selected_index == 3
    assert result.decisions[0].reason == RejectionReason.DENYLIST
    assert result.decisions[1].reason == RejectionReason.DENYLIST
    assert result.decisions[2].reason == RejectionReason.TOO_LARGE
    assert result.decisions[3].eligible is True


def test_invalid_depth_is_retained_with_explicit_rejection_reason() -> None:
    result = select_closest_eligible(
        [Detection(1, 'can', math.nan, math.nan, math.nan)]
    )

    assert result.selected_index is None
    assert result.decisions[0].reason == RejectionReason.INVALID_DEPTH


def test_full_view_orders_all_eligible_objects_by_depth_then_id():
    from types import SimpleNamespace
    values = [SimpleNamespace(object_id=object_id, label='can',
        median_depth_m=depth, estimated_width_m=.04, estimated_height_m=.08,
        x_min=x, y_min=10., x_max=x+20., y_max=30.)
        for object_id, depth, x in [(3, .5, 600.), (2, .5, 20.), (4, .7, 300.)]]
    result = select_closest_eligible(values)
    assert result.eligible_indices == (1, 0, 2)
    assert all(item.eligible for item in result.decisions)
    restricted = select_closest_eligible(values, workspace_roi_pixels=[140.,90.,500.,430.])
    assert restricted.eligible_indices == ()


def test_configured_denylist_extends_default_policy() -> None:
    detection = Detection(1, 'ceramic mug', 0.4, 0.08, 0.06)
    result = select_closest_eligible(
        [detection], denylist=('keyboard', 'mug')
    )
    assert result.decisions[0].reason == RejectionReason.DENYLIST


def test_workspace_excludes_nearer_robot_even_when_mislabeled():
    from types import SimpleNamespace
    common = dict(label='paper bag', estimated_width_m=.04, estimated_height_m=.08)
    robot = SimpleNamespace(**common, object_id=1, median_depth_m=.25,
                            x_min=0., y_min=250., x_max=49., y_max=419.)
    box = SimpleNamespace(**common, object_id=2, median_depth_m=.5,
                          x_min=267., y_min=128., x_max=376., y_max=291.)
    result = select_closest_eligible([robot, box], workspace_roi_pixels=[140.,90.,500.,430.])
    assert result.selected_index == 1
    assert result.decisions[0].reason == RejectionReason.OUTSIDE_WORKSPACE
    box.x_max = 501.
    assert select_closest_eligible([box], workspace_roi_pixels=[140.,90.,500.,430.]).selected_index is None
